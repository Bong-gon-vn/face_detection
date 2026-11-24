#!/usr/bin/env python3
"""LAN face server. Python standard library only. Run: python3 tools/lan_server.py
API key: tools/server_secrets.json (ignored by Git), or FACE_API_KEY environment variable.
Data paths are relative to this script, never relative to the shell's directory.
"""
import argparse
import base64
import csv
import hashlib
import hmac
import html
import json
import math
import os
import re
import secrets
import struct
import threading
import time
from datetime import datetime, timezone, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

DEVICE_API_KEY = ""
SERVER_SECRETS = Path(__file__).resolve().parent / "server_secrets.json"
HOST = "0.0.0.0"
PORT = 8080
MODEL_VERSION = 0x03331501
DIM = 512
THRESHOLD = 0.60  # Must equal FACE_THRESHOLD in s3_cam/include/my_library.h.
PENDING = "unconfirmed"
APPROVED = "confirmed"
STATUS_COLUMN = "Confirmation status"
TZ = timezone(timedelta(hours=7))
ROOT = Path(__file__).resolve().parent / "server_data"
USER_FIELDS = ["uid", "image", "embedding", "model_version", STATUS_COLUMN,
               "device_id", "api_key_valid", "registered_at", "approved_at", "revision"]
LOG_FIELDS = ["event_id", "uid", "time", "device_time", "score", "matched",
              "reason", "device_id", "api_key_valid", "model_version", "threshold"]
LOCK = threading.RLock()
USERS = {}
EVENTS = {}
ENROLLMENTS = {}
# Admin session is independent of device API key. Printed on local terminal only.
ADMIN_TOKEN = secrets.token_urlsafe(32)
CSRF_TOKEN = secrets.token_urlsafe(32)
SIGNING_KEY = secrets.token_bytes(32)


def now_iso():
    return datetime.now(TZ).isoformat(timespec="seconds")


def atomic_csv(path, fields, rows):
    temporary = path.with_suffix(".tmp")
    with temporary.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def load_database():
    ROOT.mkdir(parents=True, exist_ok=True)
    (ROOT / "images").mkdir(exist_ok=True)
    for filename, fields, target, key in (
        ("users.csv", USER_FIELDS, USERS, "uid"),
        ("attendance.csv", LOG_FIELDS, EVENTS, "event_id"),
    ):
        path = ROOT / filename
        if not path.exists():
            atomic_csv(path, fields, [])
        with path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames != fields:
                raise ValueError(f"{path}: columns differ; back up or migrate this CSV first")
            for row in reader:
                target[row[key]] = row


def save_users():
    atomic_csv(ROOT / "users.csv", USER_FIELDS, USERS.values())


def log_event(event_id, uid, device, score, matched, reason, device_time=""):
    if event_id in EVENTS:
        return EVENTS[event_id]
    row = dict(zip(LOG_FIELDS, [event_id, uid, now_iso(), device_time,
                               "" if score is None else score, int(matched), reason,
                               device, 1, MODEL_VERSION, THRESHOLD]))
    # Atomic replacement avoids a partially written final record after a crash.
    updated = list(EVENTS.values()) + [row]
    atomic_csv(ROOT / "attendance.csv", LOG_FIELDS, updated)
    EVENTS[event_id] = row
    print(f"ATTEND uid={uid} matched={int(matched)} score={score} reason={reason}", flush=True)
    return row


def uid_value(value):
    if not isinstance(value, str) or not re.fullmatch(r"(?:[0-9A-F]{8}|[0-9A-F]{14}|[0-9A-F]{20})", value):
        raise ValueError("uid must be uppercase HEX, 4, 7 or 10 bytes, without separators")
    return value


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,96}", value):
        raise ValueError("invalid identifier")
    return value


def decode_embedding(encoded, version):
    if version != MODEL_VERSION:
        raise ValueError("model_version mismatch")
    if not isinstance(encoded, str):
        raise ValueError("missing embedding_b64")
    raw = base64.b64decode(encoded, validate=True)
    if len(raw) != DIM * 4:
        raise ValueError("embedding must contain 512 float32 values")
    values = struct.unpack("<512f", raw)
    if not all(math.isfinite(x) for x in values):
        raise ValueError("non-finite embedding")
    norm = sum(x * x for x in values)
    if not 0.99 <= norm <= 1.01:
        raise ValueError("embedding is not normalized")
    return list(values)


def make_ticket(uid, device, event_id, revision):
    data = json.dumps([uid, device, event_id, revision, int(time.time()) + 120], separators=(",", ":")).encode()
    payload = base64.urlsafe_b64encode(data).decode().rstrip("=")
    signature = hmac.new(SIGNING_KEY, payload.encode(), hashlib.sha256).hexdigest()
    return payload + "." + signature


def read_ticket(ticket):
    if not isinstance(ticket, str) or len(ticket) > 1024:
        raise ValueError("invalid ticket")
    payload, signature = ticket.rsplit(".", 1)
    expected = hmac.new(SIGNING_KEY, payload.encode(), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(expected, signature):
        raise ValueError("invalid ticket signature")
    return json.loads(base64.urlsafe_b64decode(payload + "=" * (-len(payload) % 4)))


def cleanup_enrollments():
    expired = [key for key, item in ENROLLMENTS.items() if time.monotonic() - item["created"] > 600]
    for key in expired:
        ENROLLMENTS.pop(key)
        (ROOT / "images" / (key + ".part")).unlink(missing_ok=True)


class Handler(BaseHTTPRequestHandler):
    server_version = "FaceLAN/1"

    def log_message(self, fmt, *args):
        # Do not print URL query values, cookies, API keys or image bodies.
        pass

    def reply(self, code, body, content_type="application/json; charset=utf-8", headers=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
        elif isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        for key, value in (headers or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def read_body(self, limit):
        if self.headers.get("Transfer-Encoding"):
            raise ValueError("chunked requests not supported; supply Content-Length")
        length = int(self.headers.get("Content-Length", "0"))
        if not 0 < length <= limit:
            raise ValueError("invalid body length")
        self.connection.settimeout(15)
        body = self.rfile.read(length)
        if len(body) != length:
            raise ValueError("incomplete body")
        return body

    def device(self):
        provided = self.headers.get("X-API-Key", "")
        if not hmac.compare_digest(provided, DEVICE_API_KEY):
            return None
        return identifier(self.headers.get("X-Device-ID", ""))

    def is_admin(self):
        for cookie in self.headers.get("Cookie", "").split(";"):
            name, sep, value = cookie.strip().partition("=")
            if sep and name == "face_admin" and hmac.compare_digest(value, ADMIN_TOKEN):
                return True
        return False

    def do_GET(self):
        try:
            route = urlparse(self.path)
            if route.path == "/health":
                return self.reply(200, {"ok": True})
            if route.path == "/login":
                token = parse_qs(route.query).get("token", [""])[0]
                if hmac.compare_digest(token, ADMIN_TOKEN):
                    return self.reply(303, "", headers={"Location": "/", "Set-Cookie":
                        f"face_admin={ADMIN_TOKEN}; HttpOnly; SameSite=Strict; Path=/"})
                return self.reply(403, {"error": "use the admin link printed in the PC terminal"})
            if not self.is_admin():
                return self.reply(403, "Open the admin URL printed in the terminal running the server.", "text/plain; charset=utf-8")
            with LOCK:
                if route.path == "/":
                    return self.reply(200, self.page(), "text/html; charset=utf-8")
                if route.path.startswith("/image/"):
                    uid = uid_value(route.path.split("/")[-1])
                    row = USERS.get(uid)
                    if row:
                        return self.reply(200, (ROOT / row["image"]).read_bytes(), "image/jpeg")
                if route.path in ("/users.csv", "/attendance.csv"):
                    return self.reply(200, (ROOT / route.path[1:]).read_bytes(), "text/csv; charset=utf-8")
            self.reply(404, {"error": "not_found"})
        except (ValueError, KeyError, OSError) as exc:
            self.reply(400, {"error": str(exc)})

    def page(self):
        escape = html.escape
        rows = []
        for uid, row in USERS.items():
            status = row[STATUS_COLUMN]
            action = "approve" if status == PENDING else "revoke"
            caption = "Approve" if action == "approve" else "Revoke"
            rows.append(f'<tr><td>{uid}</td><td><img src="/image/{uid}" width="160"></td>'
                        f'<td>{escape(status)}</td><td>{escape(row["registered_at"])}</td><td>'
                        f'<form method="post" action="/admin/{action}">'
                        f'<input type="hidden" name="uid" value="{uid}">'
                        f'<input type="hidden" name="revision" value="{escape(row["revision"])}">'
                        f'<input type="hidden" name="csrf" value="{CSRF_TOKEN}">'
                        f'<button>{caption}</button></form></td></tr>')
        logs = "".join("<tr>" + "".join(f"<td>{escape(str(row[field]))}</td>" for field in
                                      ("uid", "time", "score", "matched", "reason")) + "</tr>"
                       for row in list(EVENTS.values())[-50:][::-1])
        return f'''<!doctype html><html lang="en"><meta charset="utf-8"><title>Face LAN</title>
<style>body{{font:16px system-ui;margin:28px}}table{{border-collapse:collapse}}td,th{{border:1px solid #ccc;padding:10px}}button{{padding:10px;cursor:pointer}}img{{display:block}}</style>
<h1>Enrollment and attendance</h1><p><a href="/">Refresh</a> · <a href="/users.csv">Download users.csv</a> · <a href="/attendance.csv">Download attendance.csv</a></p>
<p>Re-enrolling the same UID replaces the image and embedding and resets status to unconfirmed. Verify the right image before approving.</p>
<table><tr><th>UID</th><th>Enrolled image</th><th>Confirmation status</th><th>Registered at</th><th>Action</th></tr>{''.join(rows)}</table>
<h2>Last 50 attendance events</h2><table><tr><th>UID</th><th>Time</th><th>Cosine</th><th>Matched</th><th>Reason</th></tr>{logs}</table></html>'''

    def do_POST(self):
        try:
            route = urlparse(self.path)
            if route.path.startswith("/admin/"):
                if not self.is_admin():
                    return self.reply(403, {"error": "admin_required"})
                form = parse_qs(self.read_body(4096).decode())
                if not hmac.compare_digest(form.get("csrf", [""])[0], CSRF_TOKEN):
                    return self.reply(403, {"error": "csrf"})
                uid = uid_value(form.get("uid", [""])[0])
                with LOCK:
                    row = USERS[uid]
                    if form.get("revision", [""])[0] != row["revision"]:
                        return self.reply(409, "Image changed. Go back and refresh the page before approving.", "text/plain; charset=utf-8")
                    if route.path not in ("/admin/approve", "/admin/revoke"):
                        return self.reply(404, {"error": "not_found"})
                    updated = dict(row)
                    updated[STATUS_COLUMN] = APPROVED if route.path.endswith("/approve") else PENDING
                    updated["approved_at"] = now_iso() if updated[STATUS_COLUMN] == APPROVED else ""
                    # Invalidate outstanding verification tickets after any admin change.
                    updated["revision"] = secrets.token_hex(16)
                    USERS[uid] = updated
                    try:
                        save_users()
                    except OSError:
                        USERS[uid] = row
                        raise
                return self.reply(303, "", headers={"Location": "/"})
            device = self.device()
            if device is None:
                return self.reply(401, {"error": "invalid_api_key"})
            with LOCK:
                cleanup_enrollments()
                if route.path == "/api/enrollment/image":
                    return self.upload_image(device, parse_qs(route.query), self.read_body(4096))
                data = json.loads(self.read_body(16384))
                if not isinstance(data, dict):
                    raise ValueError("JSON object required")
                if route.path == "/api/lookup":
                    return self.lookup(device, data)
                if route.path == "/api/attendance":
                    return self.attendance(device, data)
                if route.path == "/api/enrollment/start":
                    event_id = identifier(data.get("event_id"))
                    for key, item in ENROLLMENTS.items():
                        if item["event_id"] == event_id and item["device"] == device:
                            return self.reply(200, {"enrollment_id": key})
                    if len(ENROLLMENTS) >= 32:
                        return self.reply(503, {"error": "too_many_enrollments"})
                    key = secrets.token_hex(16)
                    ENROLLMENTS[key] = {"created": time.monotonic(), "device": device,
                                        "event_id": event_id, "offset": 0, "complete": False}
                    print(f"ENROLL start device={device}", flush=True)
                    return self.reply(200, {"enrollment_id": key})
                if route.path == "/api/enrollment/metadata":
                    key = identifier(data.get("enrollment_id"))
                    item = ENROLLMENTS[key]
                    if item["device"] != device:
                        return self.reply(403, {"error": "wrong_device"})
                    uid = uid_value(data.get("uid"))
                    embedding = decode_embedding(data.get("embedding_b64"), data.get("model_version"))
                    size = data.get("image_size")
                    if type(size) is not int or not 4 <= size <= 131072:
                        raise ValueError("invalid image_size")
                    if "uid" in item:
                        if uid != item["uid"] or embedding != item["embedding"] or size != item["size"]:
                            return self.reply(409, {"error": "metadata_changed"})
                    else:
                        item.update(uid=uid, embedding=embedding, size=size)
                        (ROOT / "images" / (key + ".part")).write_bytes(b"")
                    return self.reply(200, {"offset": item["offset"]})
            self.reply(404, {"error": "not_found"})
        except (ValueError, TypeError, KeyError, struct.error) as exc:
            self.reply(400, {"error": str(exc)})
        except OSError as exc:
            print(f"Storage/network error: {exc}", flush=True)
            self.reply(500, {"error": "server_storage_or_io_error"})

    def lookup(self, device, data):
        uid = uid_value(data.get("uid"))
        event_id = identifier(data.get("event_id"))
        if event_id in EVENTS:
            prior = EVENTS[event_id]
            if prior["uid"] == uid and prior["device_id"] == device and prior["reason"] in ("unregistered", "pending"):
                return self.reply(200, {"status": prior["reason"], "server_time": int(time.time())})
            return self.reply(409, {"status": "event_finished"})
        row = USERS.get(uid)
        status = "unregistered" if row is None else "pending" if row[STATUS_COLUMN] != APPROVED else "approved"
        if status != "approved":
            log_event(event_id, uid, device, None, 0, status)
            return self.reply(200, {"status": status, "server_time": int(time.time())})
        values = json.loads(row["embedding"])
        encoded = base64.b64encode(struct.pack("<512f", *values)).decode()
        decode_embedding(encoded, int(row["model_version"]))
        return self.reply(200, {"status": "approved", "embedding_b64": encoded,
            "model_version": MODEL_VERSION, "threshold": THRESHOLD, "server_time": int(time.time()),
            "ticket": make_ticket(uid, device, event_id, row["revision"])})

    def attendance(self, device, data):
        uid = uid_value(data.get("uid"))
        event_id = identifier(data.get("event_id"))
        prior = EVENTS.get(event_id)
        if prior:
            if prior["uid"] != uid or prior["device_id"] != device:
                return self.reply(409, {"error": "event_conflict"})
            return self.reply(200, {"saved": True, "matched": int(prior["matched"]), "reason": prior["reason"]})
        ticket_uid, ticket_device, ticket_event, revision, expiry = read_ticket(data.get("ticket"))
        if (ticket_uid, ticket_device, ticket_event) != (uid, device, event_id):
            raise ValueError("ticket does not match request")
        score = data.get("score")
        if score is not None and (type(score) not in (float, int) or not math.isfinite(score) or not -1.001 <= score <= 1.001):
            raise ValueError("invalid score")
        matched = data.get("matched")
        if type(matched) is not int or matched not in (0, 1):
            raise ValueError("matched must be 0 or 1")
        reason = identifier(data.get("reason"))
        expected = int(score is not None and score >= THRESHOLD and reason == "face_result")
        if matched != expected:
            raise ValueError("score/result/threshold mismatch")
        row = USERS.get(uid)
        if time.time() > expiry:
            matched, reason = 0, "ticket_expired"
        elif row is None or row[STATUS_COLUMN] != APPROVED or row["revision"] != revision:
            matched, reason = 0, "approval_changed"
        device_time = data.get("device_time", 0)
        if type(device_time) is not int:
            raise ValueError("device_time must be epoch seconds")
        log_event(event_id, uid, device, score, matched, reason, device_time)
        return self.reply(200, {"saved": True, "matched": matched, "reason": reason})

    def upload_image(self, device, query, chunk):
        key = identifier(query.get("id", [""])[0])
        offset = int(query.get("offset", ["-1"])[0])
        item = ENROLLMENTS[key]
        if item["device"] != device or "uid" not in item:
            return self.reply(403, {"error": "invalid_enrollment"})
        if offset < 0 or offset + len(chunk) > item["size"]:
            raise ValueError("invalid image offset")
        path = ROOT / "images" / (key + (".jpg" if item["complete"] else ".part"))
        if offset < item["offset"]:
            with path.open("rb") as handle:
                handle.seek(offset)
                if handle.read(len(chunk)) != chunk or offset + len(chunk) > item["offset"]:
                    return self.reply(409, {"error": "chunk_conflict"})
        elif offset == item["offset"] and not item["complete"]:
            with path.open("r+b") as handle:
                handle.seek(offset)
                handle.write(chunk)
                handle.flush()
                os.fsync(handle.fileno())
            item["offset"] += len(chunk)
        else:
            return self.reply(409, {"error": "unexpected_offset", "offset": item["offset"]})
        if item["offset"] == item["size"] and not item["complete"]:
            raw = path.read_bytes()
            if not raw.startswith(b"\xff\xd8") or not raw.endswith(b"\xff\xd9"):
                raise ValueError("invalid JPEG markers")
            destination = path.with_suffix(".jpg")
            os.replace(path, destination)
            old = USERS.get(item["uid"])
            USERS[item["uid"]] = dict(zip(USER_FIELDS, [item["uid"], "images/" + destination.name,
                json.dumps(item["embedding"], separators=(",", ":")), MODEL_VERSION, PENDING,
                device, 1, now_iso(), "", key]))
            try:
                save_users()
            except OSError:
                if old is None:
                    USERS.pop(item["uid"], None)
                else:
                    USERS[item["uid"]] = old
                os.replace(destination, path)
                raise
            item["complete"] = True
            print(f"ENROLL saved uid={item['uid']} status={PENDING}", flush=True)
        return self.reply(200, {"offset": item["offset"], "complete": item["complete"], "status": PENDING})


def main():
    global ROOT, DEVICE_API_KEY
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--data", type=Path, default=ROOT)
    args = parser.parse_args()
    ROOT = args.data.resolve()
    # Credentials are loaded at runtime, never embedded in this tracked source.
    DEVICE_API_KEY = os.environ.get("FACE_API_KEY", "")
    if not DEVICE_API_KEY and SERVER_SECRETS.exists():
        try:
            config = json.loads(SERVER_SECRETS.read_text(encoding="utf-8"))
            DEVICE_API_KEY = config["device_api_key"]
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise SystemExit(f"Failed to read {SERVER_SECRETS.name}: {error}")
    if not DEVICE_API_KEY:
        from getpass import getpass
        entered = getpass("Enter current API key (matches secrets.h, input hidden): ").strip()
        if not entered or not entered.isascii():
            raise SystemExit("API key must be a non-empty ASCII string matching secrets.h.")
        DEVICE_API_KEY = entered
        # Exclusive create protects an existing configuration from accidental overwrite.
        try:
            fd = os.open(SERVER_SECRETS, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump({"device_api_key": DEVICE_API_KEY}, handle, indent=2)
                handle.write("\n")
        except OSError as error:
            raise SystemExit(f"Failed to save key configuration: {error}")
        print(f"Saved key to {SERVER_SECRETS.name}; this file is ignored by .gitignore.")
    if not isinstance(DEVICE_API_KEY, str) or not DEVICE_API_KEY or not DEVICE_API_KEY.isascii():
        raise SystemExit("API key must be a non-empty ASCII string.")
    load_database()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    print(f"Server: http://{args.host}:{args.port}")
    print(f"Admin on PC: http://127.0.0.1:{args.port}/login?token={ADMIN_TOKEN}")
    print(f"Database: {ROOT / 'users.csv'}")
    print(f"Attendance: {ROOT / 'attendance.csv'}")
    print("Set LAN_SERVER_URL in secrets.h to the PC's LAN IP; do not use 127.0.0.1 on the ESP.")
    print("CSVs are held in memory by the server; use the web page to approve. To edit CSVs manually: stop the server first.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
    finally:
        server.server_close()


# ===== WIFI DIAGNOSTIC CAPTURE =====
# Uses the existing ROOT, LOCK, Handler and device authentication.
import zlib
from urllib.parse import urlencode

CAPTURE_SESSIONS = {}


def capture_session_name(value):
    if not isinstance(value, str):
        raise ValueError("invalid capture session")
    if not re.fullmatch(r"[0-9a-f]{8}", value):
        raise ValueError("invalid capture session")
    return value


def capture_file_name(value):
    if not isinstance(value, str):
        raise ValueError("invalid filename")
    if not re.fullmatch(r"[A-Za-z0-9_-][A-Za-z0-9_.-]{0,95}", value):
        raise ValueError("invalid filename")
    if Path(value).suffix not in (
        ".jpg", ".jpeg", ".png", ".ppm", ".rgb565", ".json"
    ):
        raise ValueError("unsupported capture file")
    return value


def capture_fnv(path):
    value = 2166136261

    with path.open("rb") as handle:
        while True:
            block = handle.read(8192)
            if not block:
                break

            for byte in block:
                value = ((value ^ byte) * 16777619) & 0xFFFFFFFF

    return value


def capture_png(path, width, height, pixels):
    if width <= 0 or height <= 0:
        raise ValueError("invalid image dimensions")

    if len(pixels) != width * height * 3:
        raise ValueError("invalid RGB size")

    def chunk(kind, payload):
        checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", checksum)
        )

    stride = width * 3
    scanlines = b"".join(
        b"\x00" + pixels[y * stride:(y + 1) * stride]
        for y in range(height)
    )

    header = struct.pack(
        ">IIBBBBB", width, height, 8, 2, 0, 0, 0
    )

    output = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(scanlines))
        + chunk(b"IEND", b"")
    )

    path.write_bytes(output)


def capture_convert(path):
    """Create viewable PNG without Pillow or other pip packages."""
    if path.suffix == ".ppm":
        raw = path.read_bytes()

        # Matches the P6 files produced by the S3 diagnostic firmware.
        header = re.match(
            rb"P6\n([0-9]+) ([0-9]+)\n255\n", raw
        )
        if not header:
            raise ValueError("unsupported PPM header")

        width = int(header.group(1))
        height = int(header.group(2))
        pixels = raw[header.end():]

        output = path.with_suffix(".png")
        capture_png(output, width, height, pixels)
        return output.name

    if path.suffix == ".rgb565":
        raw = path.read_bytes()
        width, height = 240, 320

        if len(raw) != width * height * 2:
            raise ValueError("invalid LCD framebuffer size")

        pixels = bytearray(width * height * 3)

        for index in range(width * height):
            value = (raw[2 * index] << 8) | raw[2 * index + 1]

            red = (value >> 11) & 31
            green = (value >> 5) & 63
            blue = value & 31

            pixels[3 * index] = (red << 3) | (red >> 2)
            pixels[3 * index + 1] = (green << 2) | (green >> 4)
            pixels[3 * index + 2] = (blue << 3) | (blue >> 2)

        output = path.with_suffix(".png")
        capture_png(output, width, height, pixels)
        return output.name

    return None


BaseLANHandler = Handler


class Handler(BaseLANHandler):
    def page(self):
        page = super().page()
        link = '<p><a href="/captures">View Wi-Fi diagnostic images</a></p>'
        return page.replace("</h1>", "</h1>" + link, 1)

    def do_POST(self):
        route = urlparse(self.path)

        if not route.path.startswith("/api/capture/"):
            return super().do_POST()

        try:
            device = self.device()

            if device is None:
                return self.reply(401, {"error": "invalid_api_key"})

            if route.path == "/api/capture/data":
                query = parse_qs(route.query)

                session = capture_session_name(
                    query.get("session", [""])[0]
                )
                file_id = int(query.get("file_id", ["-1"])[0])
                offset = int(query.get("offset", ["-1"])[0])
                body = self.read_body(4096)
                data = {}
            else:
                data = json.loads(self.read_body(4096))

                if not isinstance(data, dict):
                    raise ValueError("JSON object required")

                session = capture_session_name(
                    data.get("session")
                )

            key = (device, session)

            with LOCK:
                # Expire in-memory transfer state; never delete saved files.
                now = time.monotonic()

                expired = [
                    item_key
                    for item_key, item in CAPTURE_SESSIONS.items()
                    if now - item["touched"] > 900
                ]

                for item_key in expired:
                    CAPTURE_SESSIONS.pop(item_key, None)

                if route.path == "/api/capture/start":
                    if key not in CAPTURE_SESSIONS:
                        if len(CAPTURE_SESSIONS) >= 16:
                            return self.reply(
                                503, {"error": "too_many_captures"}
                            )

                        folder = ROOT / "captures" / device / session

                        # Do not overwrite an older session after a server restart.
                        if folder.exists():
                            return self.reply(
                                409, {"error": "session_exists_retry_capture"}
                            )

                        folder.mkdir(parents=True)

                        CAPTURE_SESSIONS[key] = {
                            "folder": folder,
                            "files": {},
                            "touched": now,
                            "created": now_iso(),
                            "finished": False,
                        }

                        print(
                            f"CAPTURE start: {folder}",
                            flush=True
                        )

                    return self.reply(200, {"ok": True})

                item = CAPTURE_SESSIONS.get(key)

                if item is None:
                    return self.reply(
                        404, {"error": "capture_session_missing"}
                    )

                item["touched"] = now
                files = item["files"]
                folder = item["folder"]

                if route.path == "/api/capture/file":
                    file_id = data.get("file_id")
                    name = capture_file_name(data.get("name"))
                    size = data.get("size")

                    if type(file_id) is not int or not 0 <= file_id <= 1000:
                        raise ValueError("invalid file_id")

                    if type(size) is not int or not 0 < size <= 2 * 1024 * 1024:
                        raise ValueError("invalid file size")

                    if file_id in files:
                        previous = files[file_id]

                        if previous["name"] != name or previous["size"] != size:
                            return self.reply(
                                409, {"error": "file_metadata_conflict"}
                            )

                        return self.reply(200, {"ok": True})

                    if item["finished"]:
                        raise ValueError("capture already finished")

                    if len(files) >= 32:
                        raise ValueError("too many files")

                    if any(value["name"] == name for value in files.values()):
                        raise ValueError("duplicate filename")

                    if sum(value["size"] for value in files.values()) + size > 8 * 1024 * 1024:
                        raise ValueError("capture too large")

                    temporary = folder / (name + ".part")
                    temporary.write_bytes(b"")

                    files[file_id] = {
                        "name": name,
                        "size": size,
                        "offset": 0,
                        "complete": False,
                        "hash": None,
                    }

                    return self.reply(200, {"ok": True})

                if route.path == "/api/capture/data":
                    file = files.get(file_id)

                    if file is None:
                        raise ValueError("file not declared")

                    if offset < 0 or offset + len(body) > file["size"]:
                        raise ValueError("invalid offset")

                    filename = file["name"]
                    if not file["complete"]:
                        filename += ".part"

                    path = folder / filename

                    if offset < file["offset"]:
                        # A repeated HTTP request must contain identical bytes.
                        if offset + len(body) > file["offset"]:
                            raise ValueError("overlapping chunk")

                        with path.open("rb") as handle:
                            handle.seek(offset)

                            if handle.read(len(body)) != body:
                                raise ValueError("chunk conflict")

                    elif offset == file["offset"] and not file["complete"]:
                        with path.open("r+b") as handle:
                            handle.seek(offset)
                            handle.write(body)
                            handle.flush()

                        file["offset"] += len(body)

                    else:
                        raise ValueError("unexpected offset")

                    return self.reply(
                        200, {"offset": file["offset"]}
                    )

                if route.path == "/api/capture/file_end":
                    file_id = data.get("file_id")
                    expected_hash = data.get("hash")
                    file = files.get(file_id)

                    if file is None:
                        raise ValueError("file not declared")

                    if type(expected_hash) is not int or not 0 <= expected_hash <= 0xFFFFFFFF:
                        raise ValueError("invalid hash")

                    if file["complete"]:
                        if file["hash"] != expected_hash:
                            raise ValueError("hash conflict")
                        return self.reply(200, {"ok": True})

                    if file["offset"] != file["size"]:
                        raise ValueError("incomplete file")

                    temporary = folder / (file["name"] + ".part")

                    if capture_fnv(temporary) != expected_hash:
                        raise ValueError("file checksum mismatch")

                    destination = folder / file["name"]
                    os.replace(temporary, destination)

                    file["complete"] = True
                    file["hash"] = expected_hash

                    print(
                        f"CAPTURE saved: {destination.name} "
                        f"({file['size']} bytes)",
                        flush=True
                    )

                    return self.reply(200, {"ok": True})

                if route.path == "/api/capture/finish":
                    if item["finished"]:
                        return self.reply(
                            200, {"status": item["status"]}
                        )

                    status = data.get("status")

                    if status not in ("OK", "PARTIAL", "ERROR"):
                        raise ValueError("invalid status")

                    if not files or any(
                        not file["complete"] for file in files.values()
                    ):
                        status = "ERROR"

                    errors = []

                    for file in files.values():
                        if not file["complete"]:
                            continue

                        try:
                            png = capture_convert(folder / file["name"])
                            if png:
                                file["png"] = png
                        except (OSError, ValueError) as error:
                            errors.append(
                                f"{file['name']}: {error}"
                            )

                    if errors and status == "OK":
                        status = "PARTIAL"

                    manifest = {
                        "device_id": device,
                        "session": session,
                        "created": item["created"],
                        "finished": now_iso(),
                        "status": status,
                        "files": list(files.values()),
                        "conversion_errors": errors,
                    }

                    (folder / "capture_manifest.json").write_text(
                        json.dumps(manifest, ensure_ascii=False, indent=2),
                        encoding="utf-8"
                    )

                    item["finished"] = True
                    item["status"] = status

                    print(
                        f"CAPTURE {status}: {folder}",
                        flush=True
                    )

                    return self.reply(200, {"status": status})

                return self.reply(404, {"error": "unknown_capture_route"})

        except (ValueError, TypeError, KeyError) as error:
            return self.reply(400, {"error": str(error)})
        except OSError as error:
            print(f"CAPTURE I/O error: {error}", flush=True)
            return self.reply(500, {"error": "capture_io_error"})

    def do_GET(self):
        route = urlparse(self.path)

        if route.path not in ("/captures", "/capture_file"):
            return super().do_GET()

        if not self.is_admin():
            return self.reply(
                403,
                "Open the admin URL with token in the terminal first.",
                "text/plain; charset=utf-8"
            )

        try:
            with LOCK:
                if route.path == "/capture_file":
                    query = parse_qs(route.query)

                    device = identifier(query.get("device", [""])[0])
                    session = capture_session_name(
                        query.get("session", [""])[0]
                    )
                    name = capture_file_name(
                        query.get("name", [""])[0]
                    )

                    folder = ROOT / "captures" / device / session
                    manifest = json.loads(
                        (folder / "capture_manifest.json").read_text(
                            encoding="utf-8"
                        )
                    )

                    allowed = {"capture_manifest.json"}

                    for file in manifest["files"]:
                        if file["complete"]:
                            allowed.add(file["name"])
                            if file.get("png"):
                                allowed.add(file["png"])

                    if name not in allowed:
                        return self.reply(404, {"error": "file_missing"})

                    content_type = {
                        ".jpg": "image/jpeg",
                        ".jpeg": "image/jpeg",
                        ".png": "image/png",
                        ".json": "application/json",
                    }.get(Path(name).suffix, "application/octet-stream")

                    return self.reply(
                        200, (folder / name).read_bytes(), content_type
                    )

                root = ROOT / "captures"
                manifests = sorted(
                    root.glob("*/*/capture_manifest.json"),
                    key=lambda path: path.stat().st_mtime,
                    reverse=True
                )[:20]

                sections = []

                for path in manifests:
                    manifest = json.loads(
                        path.read_text(encoding="utf-8")
                    )

                    device = manifest["device_id"]
                    session = manifest["session"]
                    entries = []

                    for file in manifest["files"]:
                        if not file["complete"]:
                            continue

                        names = [file["name"]]
                        if file.get("png"):
                            names.append(file["png"])

                        for name in names:
                            url = "/capture_file?" + urlencode({
                                "device": device,
                                "session": session,
                                "name": name,
                            })

                            preview = ""
                            if Path(name).suffix in (".jpg", ".jpeg", ".png"):
                                preview = (
                                    f'<img src="{html.escape(url)}" '
                                    f'style="max-width:320px;display:block">'
                                )

                            entries.append(
                                '<div style="margin:12px">'
                                f'<a href="{html.escape(url)}">'
                                f'{html.escape(name)}</a>{preview}</div>'
                            )

                    sections.append(
                        f'<h2>{html.escape(device)} / {session}</h2>'
                        f'<p>{html.escape(manifest["status"])} — '
                        f'{html.escape(manifest["finished"])}</p>'
                        '<div style="display:flex;flex-wrap:wrap">'
                        + "".join(entries)
                        + "</div>"
                    )

                page = (
                    '<!doctype html><html lang="en">'
                    '<meta charset="utf-8">'
                    '<title>Wi-Fi Diagnostic Images</title>'
                    '<body style="font-family:system-ui;margin:24px">'
                    '<h1>Wi-Fi Diagnostic Images</h1>'
                    '<p><a href="/">Admin</a> · '
                    '<a href="/captures">Refresh</a></p>'
                    + ("".join(sections) or "<p>No completed capture sessions yet.</p>")
                    + "</body></html>"
                )

                return self.reply(
                    200, page, "text/html; charset=utf-8"
                )

        except (OSError, ValueError, KeyError, TypeError):
            return self.reply(404, {"error": "capture_not_found"})


if __name__ == "__main__":
    main()