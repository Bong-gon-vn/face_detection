# Capturing OV2640 Diagnostic Images to PC

This feature is added to both `s3_cam` and `esp_thuong`. Flash **both boards**.
No SPI wiring changes or SD card are needed. Images travel from the S3 over SPI to the regular ESP32,
then over the USB/serial connection of the **regular ESP32 driving the LCD** to the PC.

## BOOT Button on the Regular ESP32

| Action | Behavior |
|---|---|
| Press and release under 2 seconds | Enroll user via LAN server |
| Hold for 2 seconds | Capture and export diagnostic image set via LAN server |
| Scan NFC card | Query server, verify if approved |

The diagnostic image feature does not enroll faces and does not modify the database.
After exporting images, the system enters WAIT and continues streaming for 3 minutes before returning to SLEEP.
Wait for a session to finish before pressing again.

## Receiving Images on Windows

Open a terminal in the repository root directory. Install Python if not already installed, then run:

```powershell
py -m pip install -r tools/requirements-capture.txt
py tools/capture_faces.py --list
py tools/capture_faces.py --port COM5 --out captures
```

Replace `COM5` with the **regular/LCD ESP32** port, not the S3. Close PlatformIO Serial
Monitor, Arduino Serial Monitor, and any other applications holding the port.

Linux uses `python3` instead of `py`, for example:

```bash
python3 -m pip install -r tools/requirements-capture.txt
python3 tools/capture_faces.py --port /dev/ttyUSB0 --out captures
```

Grant serial port permissions to your Linux user account if you encounter `Permission denied`.

1. Flash firmware to both boards using your current PlatformIO workflow.
2. Power both boards while keeping the SPI wiring intact.
3. Run the receiver script and wait for `Listening ...`.
4. Position your face as in normal use. Hold BOOT for 2 seconds and release.
5. Keep your face relatively still for about **8 seconds**: the device waits 1.5 seconds and then captures two modes consecutively.
6. When the LCD displays `DANG GUI ANH PC`, capturing is complete and you can move.
7. Wait for the PC to report the output folder.

The script continues waiting for the next 2-second hold. Use `--once` if you only want to receive
a single session, or press Ctrl+C to stop. Do not press BOOT during reset or flashing.
Opening the port may still trigger a reset on some USB-UART adapters; if a board reboots,
wait until it is ready before holding for 2 seconds.