#!/usr/bin/env python3
"""Validate and convert the binary CSI files produced by capture_csi.py."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


CSI_MAGIC = b"\xaa\x55"
PROTOCOL_VERSION = 1
CSI_FRAME_TYPE = 1
CSI_HEADER = struct.Struct("<2sBBIIHbBBB")
MAX_CSI_LEN = 512
TIMESTAMP_MODULUS = 1 << 32


class CsiFormatError(ValueError):
    """Raised when a CSI file contains an invalid or truncated frame."""


@dataclass(frozen=True)
class CsiFrame:
    sequence: int
    timestamp_us: int
    timestamp_unwrapped_us: int
    rssi: int
    channel: int
    rx_format: int
    flags: int
    raw: tuple[int, ...]

    @property
    def first_word_invalid(self) -> bool:
        return bool(self.flags & 1)

    @property
    def complex_count(self) -> int:
        return len(self.raw) // 2


def read_frames(path: Path) -> tuple[list[CsiFrame], int]:
    """Read and validate every CSI frame. Return frames and parsed byte count."""
    frames: list[CsiFrame] = []
    offset = 0
    previous_timestamp: int | None = None
    timestamp_wrap_offset = 0

    with path.open("rb") as source:
        while True:
            header_bytes = source.read(CSI_HEADER.size)
            if not header_bytes:
                break
            if len(header_bytes) != CSI_HEADER.size:
                raise CsiFormatError(
                    f"truncated header at byte {offset}: "
                    f"expected {CSI_HEADER.size}, got {len(header_bytes)}"
                )

            (
                magic,
                version,
                frame_type,
                sequence,
                timestamp_us,
                length,
                rssi,
                channel,
                rx_format,
                flags,
            ) = CSI_HEADER.unpack(header_bytes)

            if magic != CSI_MAGIC:
                raise CsiFormatError(
                    f"invalid magic at byte {offset}: {magic.hex()} (expected aa55)"
                )
            if version != PROTOCOL_VERSION:
                raise CsiFormatError(
                    f"unsupported protocol version {version} at byte {offset}"
                )
            if frame_type != CSI_FRAME_TYPE:
                raise CsiFormatError(
                    f"unexpected frame type {frame_type} at byte {offset}"
                )
            if length == 0 or length > MAX_CSI_LEN:
                raise CsiFormatError(
                    f"invalid CSI length {length} at byte {offset}"
                )

            payload = source.read(length)
            if len(payload) != length:
                raise CsiFormatError(
                    f"truncated CSI payload at byte {offset}: "
                    f"expected {length}, got {len(payload)}"
                )

            if previous_timestamp is not None and timestamp_us < previous_timestamp:
                # A uint32 microsecond timestamp wraps about every 71.6 minutes.
                if previous_timestamp - timestamp_us > TIMESTAMP_MODULUS // 2:
                    timestamp_wrap_offset += TIMESTAMP_MODULUS

            raw = struct.unpack(f"<{length}b", payload)
            frames.append(
                CsiFrame(
                    sequence=sequence,
                    timestamp_us=timestamp_us,
                    timestamp_unwrapped_us=timestamp_us + timestamp_wrap_offset,
                    rssi=rssi,
                    channel=channel,
                    rx_format=rx_format,
                    flags=flags,
                    raw=raw,
                )
            )
            previous_timestamp = timestamp_us
            offset += CSI_HEADER.size + length

    if not frames:
        raise CsiFormatError("the file contains no CSI frames")
    return frames, offset


def percentile(values: list[int], fraction: float) -> int:
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def load_capture_summary(input_path: Path) -> dict | None:
    sidecar_path = input_path.with_suffix(input_path.suffix + ".json")
    if not sidecar_path.exists():
        return None
    try:
        return json.loads(sidecar_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return {"read_error": str(error)}


def build_summary(
    input_path: Path,
    frames: list[CsiFrame],
    parsed_bytes: int,
) -> dict:
    intervals = [
        current.timestamp_unwrapped_us - previous.timestamp_unwrapped_us
        for previous, current in zip(frames, frames[1:])
    ]
    sequence_gaps = sum(
        max(0, current.sequence - previous.sequence - 1)
        for previous, current in zip(frames, frames[1:])
    )
    sequence_non_increasing = sum(
        current.sequence <= previous.sequence
        for previous, current in zip(frames, frames[1:])
    )
    timestamp_non_increasing = sum(interval <= 0 for interval in intervals)
    span_us = frames[-1].timestamp_unwrapped_us - frames[0].timestamp_unwrapped_us
    capture_summary = load_capture_summary(input_path)

    summary = {
        "input_file": str(input_path),
        "file_size_bytes": input_path.stat().st_size,
        "parsed_bytes": parsed_bytes,
        "frame_count": len(frames),
        "first_sequence": frames[0].sequence,
        "last_sequence": frames[-1].sequence,
        "sequence_gaps": sequence_gaps,
        "sequence_non_increasing": sequence_non_increasing,
        "first_timestamp_us": frames[0].timestamp_us,
        "last_timestamp_us": frames[-1].timestamp_us,
        "timestamp_span_us": span_us,
        "timestamp_non_increasing": timestamp_non_increasing,
        "timestamp_wraps": (
            frames[-1].timestamp_unwrapped_us // TIMESTAMP_MODULUS
            - frames[0].timestamp_unwrapped_us // TIMESTAMP_MODULUS
        ),
        "actual_csi_hz": round((len(frames) - 1) * 1_000_000 / span_us, 6)
        if span_us > 0
        else 0,
        "csi_length_values": sorted({len(frame.raw) for frame in frames}),
        "complex_count_values": sorted({frame.complex_count for frame in frames}),
        "odd_length_frames": sum(len(frame.raw) % 2 != 0 for frame in frames),
        "first_word_invalid_frames": sum(frame.first_word_invalid for frame in frames),
        "rssi_min": min(frame.rssi for frame in frames),
        "rssi_avg": round(statistics.mean(frame.rssi for frame in frames), 6),
        "rssi_max": max(frame.rssi for frame in frames),
        "channels": sorted({frame.channel for frame in frames}),
        "rx_formats": sorted({frame.rx_format for frame in frames}),
        "flags": sorted({frame.flags for frame in frames}),
    }

    if intervals:
        summary["interval_us"] = {
            "min": min(intervals),
            "average": round(statistics.mean(intervals), 6),
            "median": round(statistics.median(intervals), 6),
            "p95": percentile(intervals, 0.95),
            "p99": percentile(intervals, 0.99),
            "max": max(intervals),
        }

    if capture_summary is not None:
        summary["capture_summary_file"] = str(
            input_path.with_suffix(input_path.suffix + ".json")
        )
        summary["capture_summary_consistency"] = {
            "captured_frame_count_matches": capture_summary.get(
                "captured_frames_on_pc"
            )
            == len(frames),
            "processed_frame_count_matches": capture_summary.get("csi_processed")
            == len(frames),
        }
    return summary


def write_frames_csv(path: Path, frames: Iterable[CsiFrame]) -> None:
    first_timestamp: int | None = None
    previous_timestamp: int | None = None
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "sequence",
                "timestamp_us",
                "timestamp_unwrapped_us",
                "elapsed_us",
                "elapsed_sec",
                "interval_us",
                "rssi",
                "channel",
                "rx_format",
                "first_word_invalid",
                "csi_length",
                "complex_count",
                "csi_raw",
            ]
        )
        for frame in frames:
            if first_timestamp is None:
                first_timestamp = frame.timestamp_unwrapped_us
            elapsed_us = frame.timestamp_unwrapped_us - first_timestamp
            interval_us = (
                ""
                if previous_timestamp is None
                else frame.timestamp_unwrapped_us - previous_timestamp
            )
            writer.writerow(
                [
                    frame.sequence,
                    frame.timestamp_us,
                    frame.timestamp_unwrapped_us,
                    elapsed_us,
                    f"{elapsed_us / 1_000_000:.6f}",
                    interval_us,
                    frame.rssi,
                    frame.channel,
                    frame.rx_format,
                    int(frame.first_word_invalid),
                    len(frame.raw),
                    frame.complex_count,
                    json.dumps(frame.raw, separators=(",", ":")),
                ]
            )
            previous_timestamp = frame.timestamp_unwrapped_us


def write_long_csv(path: Path, frames: Iterable[CsiFrame]) -> None:
    first_timestamp: int | None = None
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "sequence",
                "timestamp_us",
                "timestamp_unwrapped_us",
                "elapsed_us",
                "complex_index",
                "imag",
                "real",
                "amplitude",
                "phase_rad",
                "rssi",
                "channel",
                "rx_format",
            ]
        )
        for frame in frames:
            if first_timestamp is None:
                first_timestamp = frame.timestamp_unwrapped_us
            elapsed_us = frame.timestamp_unwrapped_us - first_timestamp
            for complex_index in range(frame.complex_count):
                imag = frame.raw[complex_index * 2]
                real = frame.raw[complex_index * 2 + 1]
                writer.writerow(
                    [
                        frame.sequence,
                        frame.timestamp_us,
                        frame.timestamp_unwrapped_us,
                        elapsed_us,
                        complex_index,
                        imag,
                        real,
                        f"{math.hypot(real, imag):.9f}",
                        f"{math.atan2(imag, real):.9f}",
                        frame.rssi,
                        frame.channel,
                        frame.rx_format,
                    ]
                )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a binary .csi file and export timestamped CSI raw data"
        )
    )
    parser.add_argument("input", type=Path, help="binary .csi file")
    parser.add_argument(
        "-o",
        "--output-prefix",
        type=Path,
        help="output path prefix (default: input path without .csi)",
    )
    parser.add_argument(
        "--long-csv",
        action="store_true",
        help="also export one row per complex CSI value (large file)",
    )
    parser.add_argument(
        "--preview",
        type=int,
        default=3,
        help="number of frames to preview on stdout (default: 3)",
    )
    args = parser.parse_args()

    if not args.input.is_file():
        parser.error(f"input file does not exist: {args.input}")
    if args.preview < 0:
        parser.error("--preview must be zero or greater")

    output_prefix = args.output_prefix or args.input.with_suffix("")
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    frames_csv = Path(f"{output_prefix}.frames.csv")
    summary_json = Path(f"{output_prefix}.summary.json")
    long_csv = Path(f"{output_prefix}.subcarriers.csv")

    try:
        frames, parsed_bytes = read_frames(args.input)
    except (OSError, CsiFormatError) as error:
        sys.exit(f"CSI analysis failed: {error}")

    summary = build_summary(args.input, frames, parsed_bytes)
    write_frames_csv(frames_csv, frames)
    summary_json.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if args.long_csv:
        write_long_csv(long_csv, frames)

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"Saved timestamped raw CSI to {frames_csv}")
    print(f"Saved analysis summary to {summary_json}")
    if args.long_csv:
        print(f"Saved complex CSI values to {long_csv}")

    preview_count = min(args.preview, len(frames))
    if preview_count:
        print(f"Previewing the first {preview_count} frame(s):")
        first_timestamp = frames[0].timestamp_unwrapped_us
        for frame in frames[:preview_count]:
            print(
                json.dumps(
                    {
                        "sequence": frame.sequence,
                        "timestamp_us": frame.timestamp_us,
                        "elapsed_us": frame.timestamp_unwrapped_us - first_timestamp,
                        "rssi": frame.rssi,
                        "channel": frame.channel,
                        "rx_format": frame.rx_format,
                        "csi_raw_first_32": list(frame.raw[:32]),
                    },
                    ensure_ascii=False,
                )
            )


if __name__ == "__main__":
    main()
