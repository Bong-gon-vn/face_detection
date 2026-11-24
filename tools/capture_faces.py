#!/usr/bin/env python3
"""Receive FD1 diagnostics from the DISPLAY ESP32 USB serial port.

The script never uploads images. Raw files, converted PNGs and contact sheets
remain in --out. Close PlatformIO Serial Monitor before opening this program.
"""
import argparse
import base64
import binascii
from datetime import datetime
import json
from pathlib import Path
import re
import sys
import time

import serial
from serial.tools import list_ports
from PIL import Image, ImageChops, ImageDraw, ImageOps, ImageStat

MAX_FILE = 4 * 1024 * 1024
SAFE_NAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,95}\Z")


def fnv1a(data, value=2166136261):
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def export_previews(folder, filenames):
    """Only complete, checksum-verified files are converted."""
    images = []
    metrics = {}
    for name in filenames:
        path = folder / name
        if path.suffix in (".jpg", ".ppm"):
            with Image.open(path) as source:
                img = source.convert("RGB")
        elif path.suffix == ".rgb565":
            raw = path.read_bytes()
            if len(raw) != 240 * 320 * 2:
                raise ValueError("LCD buffer must be 240x320 RGB565 big-endian")
            rgb = bytearray(240 * 320 * 3)
            for i in range(240 * 320):
                value = (raw[2*i] << 8) | raw[2*i+1]
                r, g, b = (value >> 11) & 31, (value >> 5) & 63, value & 31
                rgb[3*i:3*i+3] = bytes(((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)))
            img = Image.frombytes("RGB", (240, 320), bytes(rgb))
        else:
            continue
        img.save(path.with_suffix(".png"))
        images.append((path.stem, img))
        gray = img.convert("L")
        histogram = gray.histogram()
        pixels = img.width * img.height
        metrics[name] = {
            "width": img.width, "height": img.height,
            "mean_gray": round(ImageStat.Stat(gray).mean[0], 2),
            "dark_pixels_le_15_percent": round(100*sum(histogram[:16])/pixels, 2),
            "bright_pixels_ge_240_percent": round(100*sum(histogram[240:])/pixels, 2),
        }
        if img.size == (112, 112):
            img.resize((448, 448), Image.Resampling.NEAREST).save(folder / (path.stem + "_zoom4x.png"))

    # Both are the same sensor JPEG decoded by different software. Amplification
    # is for inspection, not a sharper replacement image.
    native = folder / "01_qvga_original.png"
    esp = folder / "02_qvga_esp_rgb_320x240.png"
    if native.exists() and esp.exists():
        with Image.open(native) as a, Image.open(esp) as b:
            diff = ImageChops.difference(a.convert("RGB"), b.convert("RGB"))
            diff.point(lambda value: min(255, value * 4)).save(folder / "05_decoder_difference_x4.png")
    if images:
        cell_w, cell_h, columns = 380, 360, 3
        rows = (len(images)+columns-1)//columns
        sheet = Image.new("RGB", (cell_w*columns, cell_h*rows), "#20242b")
        draw = ImageDraw.Draw(sheet)
        for i, (name, img) in enumerate(images):
            x, y = (i % columns)*cell_w, (i//columns)*cell_h
            draw.text((x+12, y+8), name, fill="white")
            draw.text((x+12, y+24), f"{img.width} x {img.height} | scaled for this sheet", fill="#bbbbbb")
            thumb = ImageOps.contain(img, (cell_w-24, cell_h-58), Image.Resampling.NEAREST)
            sheet.paste(thumb, (x+(cell_w-thumb.width)//2, y+48))
        sheet.save(folder / "contact_sheet.png")
    (folder / "image_metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")


class Receiver:
    def __init__(self, output):
        self.output = output
        self.sid = None
        self.folder = None
        self.files = {}
        self.errors = []
        self.last_activity = time.monotonic()

    def fail_file(self, file, reason):
        if not file["failed"]:
            self.errors.append(f'{file["name"]}: {reason}')
            print(f'  ERROR {file["name"]}: {reason}', flush=True)
        file["failed"] = True

    def finish(self, status):
        if self.sid is None:
            return False
        names = []
        for file in self.files.values():
            if not file["closed"]:
                file["handle"].close()
                self.fail_file(file, "incomplete transfer; kept as .part")
            if file["complete"]:
                names.append(file["name"])
        if not names:
            self.errors.append("No complete files received")
        try:
            export_previews(self.folder, names)
        except (OSError, ValueError) as exc:
            self.errors.append(f"Preview conversion: {exc}")
        summary = {"session": self.sid, "device_status": status,
                   "complete_files": names, "errors": self.errors}
        (self.folder / "transfer.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f'\n{status}: {self.folder.resolve()}', flush=True)
        for error in self.errors:
            print(f"  {error}")
        metadata = self.folder / "capture.json"
        if metadata.exists():
            try:
                info = json.loads(metadata.read_text(encoding="utf-8"))
                for mode in ("qvga", "vga"):
                    if mode in info:
                        print(f'  {mode}: {info[mode]["face_status"]}')
                if info.get("error"):
                    print(f'  Camera: {info["error"]}')
            except (ValueError, KeyError) as exc:
                print(f"  Could not read capture metadata: {exc}")
        success = status == "OK" and not self.errors
        self.sid = None
        return success

    def line(self, raw):
        # Boot messages, ESP-IDF logs and non-FD1 lines are ignored.
        line = raw.decode("ascii", errors="replace").strip()
        if not line.startswith("FD1 "):
            return None
        tokens = line.split()
        if len(tokens) < 3 or not re.fullmatch(r"[0-9a-fA-F]{8}", tokens[2]):
            return None
        kind, sid = tokens[1], tokens[2].lower()
        if kind == "SESSION":
            if self.sid is not None:
                self.finish("INTERRUPTED")
            self.sid = sid
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self.folder = self.output / f"{stamp}_{sid}"
            self.folder.mkdir(parents=True, exist_ok=False)
            self.files, self.errors = {}, []
            self.last_activity = time.monotonic()
            print(f"\nCapturing {sid}. Keep your face still for about 8 seconds.", flush=True)
            return None
        if sid != self.sid:
            return None
        self.last_activity = time.monotonic()
        try:
            if kind == "FILE" and len(tokens) == 6:
                fid, name, size = int(tokens[3]), tokens[4], int(tokens[5])
                if (fid in self.files or len(self.files) >= 40 or not SAFE_NAME.fullmatch(name)
                        or Path(name).suffix not in (".jpg", ".ppm", ".json", ".rgb565")
                        or any(f["name"] == name for f in self.files.values())
                        or not 0 < size <= MAX_FILE):
                    raise ValueError("Invalid or duplicate file header")
                partial = self.folder / (name + ".part")
                self.files[fid] = {"name": name, "size": size, "offset": 0,
                                   "hash": 2166136261, "failed": False, "complete": False,
                                   "closed": False, "handle": partial.open("xb"), "partial": partial}
                print(f"  Receiving {name} ({size:,} bytes)", flush=True)
            elif kind == "DATA" and len(tokens) == 6:
                fid, offset = int(tokens[3]), int(tokens[4])
                file = self.files.get(fid)
                if file is None:
                    raise ValueError("Data without file header")
                if file["failed"] or file["closed"]:
                    return None
                try:
                    data = base64.b64decode(tokens[5], validate=True)
                except binascii.Error:
                    self.fail_file(file, "invalid base64")
                    return None
                if offset != file["offset"] or not data or offset+len(data) > file["size"]:
                    self.fail_file(file, "lost, duplicate or out-of-order data")
                    return None
                file["handle"].write(data)
                file["hash"] = fnv1a(data, file["hash"])
                file["offset"] += len(data)
            elif kind == "END" and len(tokens) == 5:
                file = self.files.get(int(tokens[3]))
                if file is None:
                    raise ValueError("File end without header")
                if file["closed"]:
                    return None
                file["handle"].close()
                file["closed"] = True
                if file["offset"] != file["size"] or file["hash"] != int(tokens[4], 16):
                    self.fail_file(file, "length/checksum mismatch; kept as .part")
                if not file["failed"]:
                    file["partial"].rename(self.folder / file["name"])
                    file["complete"] = True
                    print(f'  Saved {file["name"]}', flush=True)
            elif kind == "NOTE":
                print("  Device: " + " ".join(tokens[3:]), flush=True)
            elif kind == "DONE" and len(tokens) == 4:
                return self.finish(tokens[3])
        except (ValueError, OSError) as exc:
            self.errors.append(str(exc))
            print(f"  Protocol/file error: {exc}", flush=True)
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Display ESP32 port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", type=Path, default=Path("captures"))
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument("--once", action="store_true", help="Exit after one completed diagnostic session")
    parser.add_argument("--timeout", type=float, default=120, help="Seconds without protocol data before aborting a session")
    args = parser.parse_args()
    if args.list:
        for port in list_ports.comports():
            print(f"{port.device}: {port.description}")
        return 0
    if not args.port:
        parser.error("Use --port COMx (display ESP32), or --list")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    receiver = Receiver(args.out)
    connection = serial.Serial(port=None, baudrate=args.baud, timeout=1)
    # Set before open to avoid intentionally resetting the ESP32 with DTR/RTS.
    connection.dtr = False
    connection.rts = False
    connection.port = args.port
    try:
        connection.open()
        print(f"Listening on {args.port} at {args.baud}. Hold BOOT for 2 seconds on the display ESP32.", flush=True)
        print("Close Serial Monitor; transfer can take 2-3 minutes at 115200 baud.", flush=True)
        pending = bytearray()
        while True:
            pending.extend(connection.read(connection.in_waiting or 1))
            while b"\n" in pending:
                raw, _, remaining = pending.partition(b"\n")
                pending = bytearray(remaining)
                result = receiver.line(raw)
                if result is not None and args.once:
                    return 0 if result else 1
            if len(pending) > 4096:
                pending.clear()  # discard malformed line or startup noise
            if receiver.sid and time.monotonic()-receiver.last_activity > args.timeout:
                receiver.finish("TIMEOUT")
                if args.once:
                    return 1
    except KeyboardInterrupt:
        receiver.finish("CANCELLED")
        return 130
    except (serial.SerialException, OSError) as exc:
        receiver.finish("DISCONNECTED")
        print(f"Serial/file error: {exc}", file=sys.stderr)
        return 1
    finally:
        connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
