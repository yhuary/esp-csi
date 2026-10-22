#!/usr/bin/env python3
"""Capture and validate the ESP32-C5 1000 Hz CSI binary stream."""

import argparse
import json
import struct
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")

CSI_MAGIC = b"\xaa\x55"
RESULT_MAGIC = b"\xaa\x56"
VERSION = 1
CSI_TYPE = 1
RESULT_TYPE = 2
CSI_HEADER = struct.Struct("<2sBBIIHbBBB")
RESULT_FRAME = struct.Struct("<2sBB19I")
MAX_CSI_LEN = 512

RESULT_FIELDS = (
    "ping_expected",
    "ping_attempted",
    "ping_sent",
    "ping_send_failed",
    "ping_send_enomem",
    "ping_send_eagain",
    "ping_send_other",
    "ping_reply_unique",
    "ping_timeout",
    "ping_duration_us",
    "ping_interval_avg_us",
    "ping_interval_min_us",
    "ping_interval_max_us",
    "csi_received",
    "csi_processed",
    "csi_dropped",
    "csi_interval_avg_us",
    "csi_interval_min_us",
    "csi_interval_max_us",
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture one 10-second, 1000 Hz CSI measurement"
    )
    parser.add_argument("port", help="e.g. /dev/cu.usbmodem5C371828181")
    parser.add_argument("-o", "--output", type=Path, default=Path("capture.csi"))
    parser.add_argument(
        "--raw-output",
        type=Path,
        help="also save all received serial bytes for diagnostics",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=45.0,
        help="seconds to wait for Wi-Fi connection and measurement (default: 45)",
    )
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="do not reset the ESP32-C5 after opening the serial port",
    )
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + args.timeout
    buffer = bytearray()
    frame_count = 0
    raw_bytes_received = 0
    serial_preview = bytearray()
    sequence_gaps = 0
    previous_sequence = None

    raw_output = args.raw_output.open("wb") if args.raw_output else None
    with serial.Serial(args.port, 4_000_000, timeout=0.05) as port, args.output.open("wb") as output:
        port.reset_input_buffer()
        if not args.no_reset:
            # RTS is connected to EN on Espressif development boards. Keep DTR
            # deasserted so GPIO0 remains high, then pulse EN for a normal boot.
            port.dtr = False
            port.rts = True
            time.sleep(0.1)
            port.rts = False
            time.sleep(0.1)
        print(f"Capturing {args.port} at 4000000 baud.")

        while time.monotonic() < deadline:
            chunk = port.read(8192)
            if raw_output:
                raw_output.write(chunk)
            raw_bytes_received += len(chunk)
            if len(serial_preview) < 512:
                serial_preview.extend(chunk[: 512 - len(serial_preview)])
            buffer.extend(chunk)

            while len(buffer) >= 2:
                marker = buffer.find(b"\xaa")
                if marker < 0:
                    buffer.clear()
                    break
                if marker:
                    del buffer[:marker]
                if len(buffer) < 2:
                    break

                if buffer[:2] == CSI_MAGIC:
                    if len(buffer) < CSI_HEADER.size:
                        break
                    fields = CSI_HEADER.unpack_from(buffer)
                    _, version, frame_type, sequence, _, length, _, _, _, _ = fields
                    if (
                        version != VERSION
                        or frame_type != CSI_TYPE
                        or length == 0
                        or length > MAX_CSI_LEN
                    ):
                        del buffer[0]
                        continue

                    frame_size = CSI_HEADER.size + length
                    if len(buffer) < frame_size:
                        break

                    output.write(buffer[:frame_size])
                    del buffer[:frame_size]
                    frame_count += 1
                    if previous_sequence is not None and sequence > previous_sequence + 1:
                        sequence_gaps += sequence - previous_sequence - 1
                    previous_sequence = sequence
                    continue

                if buffer[:2] == RESULT_MAGIC:
                    if len(buffer) < RESULT_FRAME.size:
                        break
                    unpacked = RESULT_FRAME.unpack_from(buffer)
                    _, version, frame_type, *values = unpacked
                    if version != VERSION or frame_type != RESULT_TYPE:
                        del buffer[0]
                        continue

                    result = dict(zip(RESULT_FIELDS, values))
                    result["captured_frames_on_pc"] = frame_count
                    result["pc_sequence_gaps"] = sequence_gaps
                    result["serial_bytes_received"] = raw_bytes_received
                    result["ping_actual_hz"] = round(
                        result["ping_attempted"] * 1_000_000
                        / result["ping_duration_us"], 3
                    ) if result["ping_duration_us"] else 0
                    result["csi_actual_hz"] = round(
                        1_000_000 / result["csi_interval_avg_us"], 3
                    ) if result["csi_interval_avg_us"] else 0

                    result_path = args.output.with_suffix(args.output.suffix + ".json")
                    result_path.write_text(
                        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8",
                    )
                    print(json.dumps(result, ensure_ascii=False, indent=2))
                    print(f"Saved {frame_count} frames to {args.output}")
                    print(f"Saved measurement summary to {result_path}")
                    if raw_output:
                        raw_output.close()
                    return

                del buffer[0]

    preview = serial_preview.decode("utf-8", errors="backslashreplace")
    if raw_output:
        raw_output.close()
    sys.exit(
        f"Timed out after {args.timeout:g}s without a result frame; "
        f"received {raw_bytes_received} serial bytes and saved {frame_count} "
        f"CSI frames to {args.output}\nSerial preview: {preview!r}"
    )


if __name__ == "__main__":
    main()
