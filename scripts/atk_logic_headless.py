#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Protocol portions are adapted from the public ALIENTEK atk-logic sources:
# https://github.com/alientek-openedv/atk-logic
# Reimplemented in Python and substantially modified on 2026-08-14.
"""Headless capture client for the ALIENTEK ATK-Logic DL16 family.

The USB protocol implemented here is based on ALIENTEK's public atk-logic
sources and tested on compatible hardware. It deliberately uses only PyUSB
and the Python standard library.

Packed channel files are LSB-first: sample N is bit (N & 7) of byte N//8.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import statistics
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Sequence

try:
    import usb.core
    import usb.util
except ImportError as exc:  # pragma: no cover - depends on host setup
    raise SystemExit("PyUSB is required: python3 -m pip install pyusb") from exc


VID = 0x1A86
PID = 0xFFCC
INTERFACE = 0
EP_OUT = 0x02
EP_IN = 0x81
USB_BLOCK = 2048
DIRECT_PACKET_SIZE = 512  # Public source uses 512-byte MCU packets.
FLEXRAY_BITRATE = 10_000_000
SLOT10_BENCH_CYCLE_PERIOD_US = 5000  # signal-gen fixed 200 Hz schedule
FLEXRAY_NFI_INDICATOR_MASK = 0x04

RATE_INDEX = {
    1_000_000: 0,
    2_000_000: 1,
    4_000_000: 2,
    5_000_000: 3,
    10_000_000: 4,
    20_000_000: 5,
    25_000_000: 6,
    40_000_000: 7,
    50_000_000: 8,
    100_000_000: 9,
    200_000_000: 10,
    250_000_000: 11,
    500_000_000: 12,
    1_000_000_000: 13,
}

TRIGGER_BITS_EVEN = {
    "low": 0x00,
    "rising": 0x10,
    "falling": 0x20,
    "both": 0x30,
    "high": 0x40,
    "none": 0x70,
}
TRIGGER_BITS_ODD = {
    "low": 0x00,
    "rising": 0x01,
    "falling": 0x02,
    "both": 0x03,
    "high": 0x04,
    "none": 0x07,
}


class ProtocolError(RuntimeError):
    pass


def _crc32_atk(data: bytes) -> int:
    """ATK CRC32: reflected polynomial, initial 0, final XOR all ones."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def _device_to_host_block(block: bytes) -> bytes:
    if len(block) != USB_BLOCK:
        raise ValueError("ATK USB conversion requires one 2048-byte block")
    source = struct.unpack("<1024H", block)
    result = [0] * 1024
    result[0::4] = source[0:256]
    result[1::4] = source[256:512]
    result[2::4] = source[512:768]
    result[3::4] = source[768:1024]
    return struct.pack("<1024H", *result)


def _host_to_device_block(block: bytes) -> bytes:
    if len(block) != USB_BLOCK:
        raise ValueError("ATK USB conversion requires one 2048-byte block")
    source = struct.unpack("<1024H", block)
    result = source[0::4] + source[1::4] + source[2::4] + source[3::4]
    return struct.pack("<1024H", *result)


def _encode_command(code: int, payload: bytes = b"") -> bytes:
    if not 0 <= code <= 0xFF or len(payload) > 253:
        raise ValueError("invalid ATK command")

    inner = bytes((code, len(payload) + 1)) + payload
    wire_length = len(inner) + 15
    padded_length = math.ceil(wire_length / USB_BLOCK) * USB_BLOCK
    logical = bytearray(padded_length)
    logical[8] = 0x0A
    logical[9 : 9 + len(inner)] = inner
    logical[9 + len(inner)] = 0x0B
    struct.pack_into("<I", logical, 10 + len(inner), _crc32_atk(inner))

    return b"".join(
        _host_to_device_block(logical[offset : offset + USB_BLOCK])
        for offset in range(0, padded_length, USB_BLOCK)
    )


def _direct_packet(code: int, payload: bytes = b"") -> bytes:
    packet = bytearray(DIRECT_PACKET_SIZE)
    packet[0] = 0x0A
    packet[1] = code
    packet[2 : 2 + len(payload)] = payload
    packet[2 + len(payload)] = 0x0B
    return bytes(packet)


def _u40le(value: int) -> bytes:
    if not 0 <= value < (1 << 40):
        raise ValueError("value does not fit ATK's 40-bit field")
    return value.to_bytes(5, "little")


def _threshold_byte(volts: float) -> int:
    magnitude = round(abs(volts) * 10)
    if magnitude > 0x7F:
        raise ValueError("threshold must fit in signed tenths of a volt")
    return magnitude | (0x80 if volts < 0 else 0)


def _parameter_payload(
    *, rate: int, samples: int, trigger_samples: int, threshold: float, buffer: bool, rle: bool
) -> bytes:
    if rate not in RATE_INDEX:
        raise ValueError(f"unsupported sample rate: {rate}")
    flags = (0x80 if buffer else 0) | (0x40 if rle else 0)
    return bytes((flags, _threshold_byte(threshold), RATE_INDEX[rate] + 1)) + _u40le(
        samples
    ) + _u40le(trigger_samples)


def _trigger_payload(
    enabled_channels: Sequence[int], trigger_channel: int, trigger_kind: str, instant: bool
) -> bytes:
    enabled = set(enabled_channels)
    if any(channel < 0 or channel > 15 for channel in enabled):
        raise ValueError("channel IDs must be in 0..15")
    if not instant and trigger_channel not in enabled:
        raise ValueError("trigger channel must be enabled")

    result = bytearray(8)
    for pair in range(8):
        even = pair * 2
        odd = even + 1
        if even in enabled:
            kind = trigger_kind if even == trigger_channel else "none"
            result[pair] |= 0x80 | TRIGGER_BITS_EVEN[kind]
        if odd in enabled:
            kind = trigger_kind if odd == trigger_channel else "none"
            result[pair] |= 0x08 | TRIGGER_BITS_ODD[kind]
    result.append(1 if instant else 0)
    # Some device revisions require an explicit repeat=0 byte. This was
    # confirmed by an A/B hardware test against the public-source payload.
    result.append(0)
    return bytes(result)


@dataclass(frozen=True)
class Reply:
    order: int
    payload: bytes


class ReplyParser:
    """Incremental parser for the logical (post-conversion) reply stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[Reply]:
        self._buffer.extend(data)
        replies: list[Reply] = []
        while True:
            header = self._buffer.find(0x0A)
            if header < 0:
                self._buffer.clear()
                break
            if header:
                del self._buffer[:header]
            if len(self._buffer) < 4:
                break
            order = self._buffer[1]
            if not 1 <= order <= 6:
                del self._buffer[0]
                continue
            payload_length = int.from_bytes(self._buffer[2:4], "little")
            total = payload_length + 6
            if len(self._buffer) < total:
                break
            if self._buffer[payload_length + 4] != 0x00 or self._buffer[
                payload_length + 5
            ] != 0x0B:
                del self._buffer[0]
                continue
            replies.append(Reply(order, bytes(self._buffer[4 : 4 + payload_length])))
            del self._buffer[:total]
        return replies


@dataclass(frozen=True)
class FlexRayFrame:
    start_sample: int
    fss_start_sample: int
    end_sample: int
    samples_per_bit: int
    tss_bits: float
    fss_center_high: bool
    first_bss_ok: bool
    fbss_fall_sample: int | None
    fes_start_sample: int
    fes_fall_sample: int | None
    cid_start_sample: int
    cid_rise_sample: int | None
    fes_center_low: bool
    cid_bits_high: bool
    fes_ok: bool
    cid_ok: bool
    indicators: int
    nfi: int
    payload_valid: bool
    frame_id: int
    cycle: int
    payload: bytes
    header_crc: int
    header_crc_ok: bool
    frame_crc: int
    frame_crc_ok: bool


def _crc_msb(data: int, bit_count: int, polynomial: int, width: int, initial: int) -> int:
    register = initial
    for bit_index in range(bit_count - 1, -1, -1):
        feedback = ((register >> (width - 1)) & 1) ^ ((data >> bit_index) & 1)
        register <<= 1
        if feedback:
            register ^= polynomial
    return register & ((1 << width) - 1)


def _flexray_header_crc(header: bytes) -> int:
    if len(header) < 3:
        raise ValueError("FlexRay header requires at least three bytes")
    protected = (((header[0] & 0x1F) << 16) | (header[1] << 8) | header[2]) >> 1
    return _crc_msb(protected, 20, 0x385, 11, 0x01A)


def _flexray_frame_crc(frame_without_crc: bytes, channel_type: str) -> int:
    initial = 0xFEDCBA if channel_type == "A" else 0xABCDEF
    return _crc_msb(
        int.from_bytes(frame_without_crc, "big"),
        len(frame_without_crc) * 8,
        0x5D6DCB,
        24,
        initial,
    )


def _majority_sample(packed: bytes, center: int, radius: int) -> int:
    start = max(0, center - radius)
    end = min(len(packed) * 8, center + radius + 1)
    if start >= end:
        raise IndexError("sample lies outside capture")
    high = sum(_unpack_sample(packed, sample) for sample in range(start, end))
    return int(high * 2 >= end - start)


def _nearest_edge(
    packed: bytes,
    sample_count: int,
    expected: int,
    new_level: int,
    tolerance: int,
) -> int | None:
    """Return the closest half-open run boundary with the requested polarity."""
    candidates: list[int] = []
    start = max(1, expected - tolerance)
    end = min(sample_count - 1, expected + tolerance)
    for sample in range(start, end + 1):
        if (_unpack_sample(packed, sample) == new_level and
                _unpack_sample(packed, sample - 1) != new_level):
            candidates.append(sample)
    return min(candidates, key=lambda sample: (abs(sample - expected), sample)) \
        if candidates else None


def _decode_flexray_frames(
    packed: bytes,
    sample_count: int,
    sample_rate: int,
    bitrate: int,
    channel_type: str,
) -> list[FlexRayFrame]:
    if sample_rate % bitrate:
        raise ValueError("FlexRay decoder currently requires an integral samples/bit ratio")
    samples_per_bit = sample_rate // bitrate
    if samples_per_bit < 5:
        raise ValueError("FlexRay decoding requires at least five samples per bit")
    radius = max(1, samples_per_bit // 4)
    edge_tolerance = max(1, samples_per_bit // 3)
    idle_min = samples_per_bit * 10.5
    frames: list[FlexRayFrame] = []

    level = _unpack_sample(packed, 0)
    run_start = 0
    previous_high_length = 0
    candidates: list[tuple[int, int]] = []
    for edge_sample, new_level in _edge_iterator(packed, sample_count):
        run_length = edge_sample - run_start
        if level:
            previous_high_length = run_length
        elif new_level and previous_high_length >= idle_min:
            # The low-to-high edge is the TSS/FSS boundary. FlexRay permits a
            # longer CAS, so only reject implausibly short/long dominant runs.
            if samples_per_bit <= run_length <= samples_per_bit * 32:
                candidates.append((run_start, edge_sample))
        run_start = edge_sample
        level = new_level

    def sample_at_half_bits(half_bit_offset: int) -> int:
        # Explicit half-up rounding avoids Python's banker-rounding when the
        # analyzer runs at an odd samples/bit ratio such as 5 Sa/bit.
        numerator = 2 * fss_start + half_bit_offset * samples_per_bit
        center = (numerator + 1) // 2
        return _majority_sample(packed, center, radius)

    for tss_start, fss_start in candidates:
        try:
            fss_center_high = sample_at_half_bits(1) == 1
            if not fss_center_high:
                continue

            decoded = bytearray()
            valid_bss = True
            first_bss_ok = False
            for byte_index in range(5):
                group = 1 + byte_index * 10
                bss_ok = (
                    sample_at_half_bits(2 * group + 1) == 1 and
                    sample_at_half_bits(2 * group + 3) == 0
                )
                if byte_index == 0:
                    first_bss_ok = bss_ok
                if not bss_ok:
                    valid_bss = False
                    break
                value = 0
                for bit in range(8):
                    value = (value << 1) | sample_at_half_bits(
                        2 * group + 5 + 2 * bit
                    )
                decoded.append(value)
            if not valid_bss:
                continue

            payload_words = decoded[2] >> 1
            total_bytes = 5 + payload_words * 2 + 3
            if total_bytes > 262:
                continue
            for byte_index in range(5, total_bytes):
                group = 1 + byte_index * 10
                if (sample_at_half_bits(2 * group + 1) != 1 or
                        sample_at_half_bits(2 * group + 3) != 0):
                    valid_bss = False
                    break
                value = 0
                for bit in range(8):
                    value = (value << 1) | sample_at_half_bits(
                        2 * group + 5 + 2 * bit
                    )
                decoded.append(value)
            if not valid_bss:
                continue

            after_bytes = 1 + total_bytes * 10
            fes_center_ok = sample_at_half_bits(2 * after_bytes + 1) == 0
            # Check every one of the 11 CID bits.  The final center is
            # after_bytes+11.5; using +10.5 would only check ten bits.
            cid_centers_ok = all(
                sample_at_half_bits(2 * (after_bytes + 1 + bit) + 1) == 1
                for bit in range(11)
            )
        except IndexError:
            continue

        expected_header_crc = _flexray_header_crc(decoded[:5])
        header_crc = ((decoded[2] & 1) << 10) | (decoded[3] << 2) | (decoded[4] >> 6)
        frame_crc = int.from_bytes(decoded[-3:], "big")
        expected_frame_crc = _flexray_frame_crc(decoded[:-3], channel_type)
        indicators = decoded[0] >> 3
        nfi = int(bool(indicators & FLEXRAY_NFI_INDICATOR_MASK))
        fbss_expected = fss_start + 2 * samples_per_bit
        fes_start = fss_start + after_bytes * samples_per_bit
        cid_start = fes_start + samples_per_bit
        fbss_fall = _nearest_edge(
            packed, sample_count, fbss_expected, 0, edge_tolerance
        )
        # The last CRC bit may already be dominant, so a distinct falling
        # edge at the nominal FES boundary is optional.  The FES-to-CID rise
        # is mandatory and proves the dominant delimiter is not extended.
        fes_fall = _nearest_edge(
            packed, sample_count, fes_start, 0, edge_tolerance
        )
        cid_rise = _nearest_edge(
            packed, sample_count, cid_start, 1, edge_tolerance
        )
        frames.append(
            FlexRayFrame(
                start_sample=tss_start,
                fss_start_sample=fss_start,
                end_sample=fss_start + (after_bytes + 12) * samples_per_bit,
                samples_per_bit=samples_per_bit,
                tss_bits=(fss_start - tss_start) / samples_per_bit,
                fss_center_high=fss_center_high,
                first_bss_ok=first_bss_ok,
                fbss_fall_sample=fbss_fall,
                fes_start_sample=fes_start,
                fes_fall_sample=fes_fall,
                cid_start_sample=cid_start,
                cid_rise_sample=cid_rise,
                fes_center_low=fes_center_ok,
                cid_bits_high=cid_centers_ok,
                fes_ok=fes_center_ok,
                cid_ok=cid_centers_ok and cid_rise is not None,
                indicators=indicators,
                nfi=nfi,
                payload_valid=bool(nfi),
                frame_id=((decoded[0] & 0x07) << 8) | decoded[1],
                cycle=decoded[4] & 0x3F,
                payload=bytes(decoded[5:-3]),
                header_crc=header_crc,
                header_crc_ok=header_crc == expected_header_crc,
                frame_crc=frame_crc,
                frame_crc_ok=frame_crc == expected_frame_crc,
            )
        )
    return frames


def _slot10_wire_metrics(
    frame: FlexRayFrame,
    tolerance_samples: int,
) -> dict[str, object]:
    """Return explicit prefix/delimiter timing checks for one decoded FID10."""
    samples_per_bit = frame.samples_per_bit
    expected_tss_samples = 2 * samples_per_bit
    tss_samples = frame.fss_start_sample - frame.start_sample
    tss_residual = tss_samples - expected_tss_samples

    fss_expected = frame.start_sample + expected_tss_samples
    fss_residual = frame.fss_start_sample - fss_expected
    fbss_expected = frame.fss_start_sample + 2 * samples_per_bit
    fbss_residual = (
        None if frame.fbss_fall_sample is None
        else frame.fbss_fall_sample - fbss_expected
    )

    fes_fall_residual = (
        None if frame.fes_fall_sample is None
        else frame.fes_fall_sample - frame.fes_start_sample
    )
    center_offset = (samples_per_bit + 1) // 2
    fes_center_sample = frame.fes_start_sample + center_offset
    # A physical FES falling edge exists only when the final frame-CRC data
    # bit was recessive.  If it was already dominant, the FES boundary is
    # deliberately represented by its nominal sample and center-level check.
    fes_edge_expected = bool(frame.frame_crc & 1)
    cid_rise_residual = (
        None if frame.cid_rise_sample is None
        else frame.cid_rise_sample - frame.cid_start_sample
    )

    tss_pass = abs(tss_residual) <= tolerance_samples
    prefix_pass = bool(
        frame.fss_center_high and frame.first_bss_ok and
        abs(fss_residual) <= tolerance_samples and
        fbss_residual is not None and
        abs(fbss_residual) <= tolerance_samples
    )
    fes_pass = bool(
        frame.fes_center_low and
        (not fes_edge_expected or (
            fes_fall_residual is not None and
            abs(fes_fall_residual) <= tolerance_samples
        ))
    )
    cid_pass = bool(
        frame.cid_bits_high and cid_rise_residual is not None and
        abs(cid_rise_residual) <= tolerance_samples
    )

    return {
        "samples_per_bit": samples_per_bit,
        "tolerance_samples": tolerance_samples,
        "tss_start_sample": frame.start_sample,
        "tss_samples": tss_samples,
        "tss_expected_samples": expected_tss_samples,
        "tss_residual_samples": tss_residual,
        "tss_pass": tss_pass,
        "fss_expected_sample": fss_expected,
        "fss_start_sample": frame.fss_start_sample,
        "fss_residual_samples": fss_residual,
        "fss_center_high": frame.fss_center_high,
        "fbss_expected_sample": fbss_expected,
        "fbss_fall_sample": frame.fbss_fall_sample,
        "fbss_residual_samples": fbss_residual,
        "first_bss_ok": frame.first_bss_ok,
        "prefix_pass": prefix_pass,
        "fes_start_sample": frame.fes_start_sample,
        "fes_center_sample": fes_center_sample,
        "fes_fall_sample": frame.fes_fall_sample,
        "fes_fall_residual_samples": fes_fall_residual,
        "fes_edge_expected": fes_edge_expected,
        "fes_center_low": frame.fes_center_low,
        "fes_pass": fes_pass,
        "cid_start_sample": frame.cid_start_sample,
        "cid_first_center_sample": frame.cid_start_sample + center_offset,
        "cid_last_center_sample": (
            frame.cid_start_sample + 10 * samples_per_bit + center_offset
        ),
        "cid_rise_sample": frame.cid_rise_sample,
        "cid_rise_residual_samples": cid_rise_residual,
        "cid_bits_high": frame.cid_bits_high,
        "cid_end_sample": frame.end_sample,
        "cid_pass": cid_pass,
        "pass": tss_pass and prefix_pass and fes_pass and cid_pass,
    }


def _active_low_runs(packed: bytes, sample_count: int) -> list[dict[str, object]]:
    """Return half-open active-low runs, including capture-boundary runs."""
    if sample_count <= 0 or len(packed) * 8 < sample_count:
        raise ValueError("TXEN packed data is shorter than the capture")

    runs: list[dict[str, object]] = []
    run_start = 0 if _unpack_sample(packed, 0) == 0 else None
    for sample, new_level in _edge_iterator(packed, sample_count):
        if new_level == 0:
            run_start = sample
        elif run_start is not None:
            runs.append({
                "index": len(runs),
                "start_sample": run_start,
                "end_sample": sample,
                "width_samples": sample - run_start,
                "truncated_at_start": run_start == 0,
                "truncated_at_end": False,
            })
            run_start = None
    if run_start is not None:
        runs.append({
            "index": len(runs),
            "start_sample": run_start,
            "end_sample": sample_count,
            "width_samples": sample_count - run_start,
            "truncated_at_start": run_start == 0,
            "truncated_at_end": True,
        })
    return runs


def _verify_slot10_timing(
    frames: Sequence[FlexRayFrame],
    scenario: str,
    tolerance_samples: int,
    expected_payload: bytes | None,
    sample_count: int | None = None,
    txen_packed: bytes | None = None,
    opposite_txen_packed: bytes | None = None,
    opposite_txen_channel: int | None = None,
    expected_nfi: int | None = None,
) -> dict[str, object]:
    """Verify every fully captured slot-10 opportunity, not only successes."""
    if tolerance_samples < 0:
        raise ValueError("slot timing tolerance cannot be negative")
    if scenario not in ("with11", "without11"):
        raise ValueError(f"unknown slot10 scenario: {scenario}")
    if expected_nfi not in (None, 0, 1):
        raise ValueError("expected FID10 NFI must be 0, 1, or unspecified")

    decoded = sorted(frames, key=lambda frame: frame.start_sample)
    valid = sorted(
        (frame for frame in decoded if frame.header_crc_ok and frame.frame_crc_ok),
        key=lambda frame: frame.start_sample,
    )

    normal_pairs: list[dict[str, object]] = []
    for first, second in zip(valid, valid[1:]):
        if (first.cycle != second.cycle or
                second.frame_id != first.frame_id + 1 or
                10 in (first.frame_id, second.frame_id)):
            continue
        normal_pairs.append({
            "cycle": first.cycle,
            "from_fid": first.frame_id,
            "to_fid": second.frame_id,
            "tss_samples": second.start_sample - first.start_sample,
            "fss_samples": second.fss_start_sample - first.fss_start_sample,
        })

    tss_baseline = statistics.median(
        int(pair["tss_samples"]) for pair in normal_pairs
    ) if normal_pairs else None
    fss_baseline = statistics.median(
        int(pair["fss_samples"]) for pair in normal_pairs
    ) if normal_pairs else None
    for pair in normal_pairs:
        pair["tss_error_samples"] = (
            None if tss_baseline is None else pair["tss_samples"] - tss_baseline
        )
        pair["fss_error_samples"] = (
            None if fss_baseline is None else pair["fss_samples"] - fss_baseline
        )
        pair["pass"] = bool(
            tss_baseline is not None and fss_baseline is not None and
            abs(float(pair["tss_error_samples"])) <= tolerance_samples and
            abs(float(pair["fss_error_samples"])) <= tolerance_samples
        )

    inferred_capture_end = max((frame.end_sample for frame in decoded), default=0)
    explicit_sample_count = sample_count is not None
    if sample_count is None:
        sample_count = (
            len(txen_packed) * 8 if txen_packed is not None
            else len(opposite_txen_packed) * 8
            if opposite_txen_packed is not None
            else inferred_capture_end + tolerance_samples
        )
    if sample_count <= 0:
        raise ValueError("slot10 verification requires a non-empty capture")
    if any(
        frame.start_sample < 0 or frame.end_sample > sample_count
        for frame in decoded
    ):
        raise ValueError("decoded FlexRay frame lies outside the capture")

    ordinary_extents = [
        frame.end_sample - frame.start_sample
        for frame in valid
        if frame.frame_id != 10
    ]
    ordinary_samples_per_bit = [
        frame.samples_per_bit for frame in valid if frame.frame_id != 10
    ]
    frame_extent = (
        statistics.median(ordinary_extents) if ordinary_extents else None
    )
    samples_per_bit = (
        statistics.median(ordinary_samples_per_bit)
        if ordinary_samples_per_bit else None
    )
    capture_head_margin = (
        tolerance_samples if samples_per_bit is None
        else int(math.ceil(10.5 * float(samples_per_bit))) + tolerance_samples
    )

    def crc_valid(frame: FlexRayFrame) -> bool:
        return frame.header_crc_ok and frame.frame_crc_ok

    def payload_semantics_ok(frame: FlexRayFrame) -> bool:
        # FlexRay NFI is asserted for a data frame.  A null frame retains its
        # configured payload length, but every transmitted payload byte is 0.
        return frame.payload_valid or not any(frame.payload)

    def frame_diagnostic(frame: FlexRayFrame) -> dict[str, object]:
        return {
            "frame_id": frame.frame_id,
            "cycle": frame.cycle,
            "start_sample": frame.start_sample,
            "end_sample": frame.end_sample,
            "header_crc_ok": frame.header_crc_ok,
            "frame_crc_ok": frame.frame_crc_ok,
            "indicators": frame.indicators,
            "nfi": frame.nfi,
            "payload_valid": frame.payload_valid,
            "payload_semantics_ok": payload_semantics_ok(frame),
            "payload_hex": frame.payload.hex(),
        }

    # Cycle numbers wrap at 64. Split repeated values by a gap far larger than
    # the static portion of one cycle, then derive the missing-slot phase from
    # every decoded ordinary frame in the group. This also keeps a corrupt or
    # absent F9 from silently removing that cycle from the acceptance divisor.
    cycle_groups: list[list[FlexRayFrame]] = []
    if tss_baseline is not None:
        split_gap = max(1, int(math.ceil(float(tss_baseline) * 32)))
        by_cycle_number: dict[int, list[FlexRayFrame]] = {}
        for frame in decoded:
            by_cycle_number.setdefault(frame.cycle, []).append(frame)
        for same_number in by_cycle_number.values():
            same_number.sort(key=lambda frame: frame.start_sample)
            group: list[FlexRayFrame] = []
            for frame in same_number:
                if group and frame.start_sample - group[-1].start_sample > split_gap:
                    cycle_groups.append(group)
                    group = []
                group.append(frame)
            if group:
                cycle_groups.append(group)

    phased_anchors: list[tuple[float, int]] = []
    for group in cycle_groups:
        phase_samples = [
            frame.start_sample + (9 - frame.frame_id) * float(tss_baseline)
            for frame in group
            if crc_valid(frame) and
            0 <= frame.frame_id <= 15 and frame.frame_id != 10
        ]
        if phase_samples:
            phased_anchors.append((statistics.median(phase_samples), group[0].cycle))
    phased_anchors.sort(key=lambda item: item[0])

    cycle_period_candidates: list[float] = []
    for (first_phase, first_cycle), (second_phase, second_cycle) in zip(
        phased_anchors, phased_anchors[1:]
    ):
        cycle_delta = (second_cycle - first_cycle) & 0x3F
        if 1 <= cycle_delta <= 32:
            cycle_period_candidates.append(
                (second_phase - first_phase) / cycle_delta
            )
    cycle_period = (
        statistics.median(cycle_period_candidates)
        if cycle_period_candidates else None
    )
    expected_cycle_period = (
        None if samples_per_bit is None else
        float(samples_per_bit) * FLEXRAY_BITRATE *
        SLOT10_BENCH_CYCLE_PERIOD_US / 1_000_000
    )
    expected_cycle_tolerance = (
        None if expected_cycle_period is None else
        max(4 * float(tss_baseline), 0.05 * expected_cycle_period)
    )
    cycle_period_evidence_sufficient = bool(
        cycle_period is not None and expected_cycle_period is not None and
        expected_cycle_tolerance is not None and
        abs(cycle_period - expected_cycle_period) <= expected_cycle_tolerance
    )
    cycle_period_tolerance = (
        None if cycle_period is None else
        max(4 * float(tss_baseline), 0.05 * float(cycle_period))
    )
    cycle_sequence_issues: list[dict[str, object]] = []
    phased_groups: list[tuple[float, int]] = []
    for anchor_index, anchor in enumerate(phased_anchors):
        if anchor_index == 0:
            phased_groups.append(anchor)
            continue
        first_phase, first_cycle = phased_anchors[anchor_index - 1]
        second_phase, second_cycle = anchor
        cycle_delta = (second_cycle - first_cycle) & 0x3F
        unit_period = (
            None if cycle_delta == 0
            else (second_phase - first_phase) / cycle_delta
        )
        period_agrees = bool(
            unit_period is not None and cycle_period is not None and
            cycle_period_tolerance is not None and
            abs(unit_period - cycle_period) <= cycle_period_tolerance
        )
        if not 1 <= cycle_delta <= 32 or not period_agrees:
            cycle_sequence_issues.append({
                "from_cycle": first_cycle,
                "to_cycle": second_cycle,
                "from_phase_sample": first_phase,
                "to_phase_sample": second_phase,
                "cycle_delta": cycle_delta,
                "unit_period_samples": unit_period,
                "reason": "ambiguous_or_inconsistent_cycle_gap",
            })
        elif cycle_delta > 1:
            for step in range(1, cycle_delta):
                phased_groups.append((
                    first_phase +
                    (second_phase - first_phase) * step / cycle_delta,
                    (first_cycle + step) & 0x3F,
                ))
        phased_groups.append(anchor)

    # If an entire cycle at either capture boundary has no decodable frames,
    # extrapolate one-period anchors until the target-slot region is outside
    # the sample window. The normal boundary test below then excludes only the
    # genuinely partial head/tail cycle.
    if phased_groups and cycle_period is not None and cycle_period > 0:
        first_phase, first_cycle = phased_groups[0]
        previous_phase = first_phase - cycle_period
        preceding: list[tuple[float, int]] = []
        previous_cycle = (first_cycle - 1) & 0x3F
        while previous_phase + 4 * float(tss_baseline) >= 0:
            preceding.append((previous_phase, previous_cycle))
            previous_phase -= cycle_period
            previous_cycle = (previous_cycle - 1) & 0x3F
        phased_groups[0:0] = reversed(preceding)

        last_phase, last_cycle = phased_groups[-1]
        next_phase = last_phase + cycle_period
        while next_phase < sample_count:
            phased_groups.append((next_phase, (last_cycle + 1) & 0x3F))
            last_phase, last_cycle = phased_groups[-1]
            next_phase = last_phase + cycle_period

    occurrences: list[dict[str, object]] = []
    complete_cycles: list[dict[str, object]] = []
    excluded_cycles: list[dict[str, object]] = []
    cycle_contexts: list[dict[str, object]] = []
    next_id = 11 if scenario == "with11" else 12
    target_extents = [
        frame.end_sample - frame.start_sample
        for frame in valid if frame.frame_id == next_id
    ]
    target_extent_baseline = (
        statistics.median(target_extents) if target_extents else frame_extent
    )

    for cycle_instance, (predicted_f9, cycle_number) in enumerate(phased_groups):
        next_start = predicted_f9 + (next_id - 9) * float(tss_baseline)
        half_slot = float(tss_baseline) / 2

        def candidates(frame_id: int) -> list[FlexRayFrame]:
            center = predicted_f9 + (frame_id - 9) * float(tss_baseline)
            return [
                frame for frame in decoded
                if frame.frame_id == frame_id and
                center - half_slot <= frame.start_sample < center + half_slot
            ]

        target_candidates = candidates(next_id)
        if target_candidates:
            required_end = max(frame.end_sample for frame in target_candidates)
            required_extent_source = f"decoded_fid{next_id}"
        elif target_extent_baseline is not None:
            required_end = next_start + float(target_extent_baseline)
            required_extent_source = f"fid{next_id}_extent_baseline"
        else:
            required_end = None
            required_extent_source = "unavailable"
        excluded_reason: str | None = None
        if predicted_f9 < capture_head_margin:
            excluded_reason = "capture_head_before_f9"
        elif required_end is None:
            excluded_reason = "no_frame_extent_baseline"
        elif (
            required_end + (tolerance_samples if explicit_sample_count else 0) >
            sample_count
        ):
            excluded_reason = "capture_tail_before_required_frame_end"
        if excluded_reason is not None:
            excluded_cycles.append({
                "cycle_instance": cycle_instance,
                "cycle": cycle_number,
                "predicted_fid9_start_sample": predicted_f9,
                "required_end_sample": required_end,
                "required_extent_source": required_extent_source,
                "reason": excluded_reason,
            })
            continue

        f9_all = candidates(9)
        f10_all = candidates(10)
        f11_all = candidates(11)
        f12_all = candidates(12)
        f9_valid = [
            frame for frame in f9_all
            if crc_valid(frame) and frame.cycle == cycle_number
        ]
        f10_valid = [
            frame for frame in f10_all
            if crc_valid(frame) and frame.cycle == cycle_number
        ]
        f11_valid = [
            frame for frame in f11_all
            if crc_valid(frame) and frame.cycle == cycle_number
        ]
        f12_valid = [
            frame for frame in f12_all
            if crc_valid(frame) and frame.cycle == cycle_number
        ]

        fid9_pass = len(f9_all) == 1 and len(f9_valid) == 1
        fid10_crc_pass = len(f10_all) == 1 and len(f10_valid) == 1
        fid10_nfi_pass = bool(
            fid10_crc_pass and
            (expected_nfi is None or f10_valid[0].nfi == expected_nfi)
        )
        fid10_payload_semantics_pass = bool(
            fid10_crc_pass and payload_semantics_ok(f10_valid[0])
        )
        payload_pass = bool(
            fid10_crc_pass and
            (expected_payload is None or f10_valid[0].payload == expected_payload)
        )
        if scenario == "with11":
            scenario_frame_pass = len(f11_all) == 1 and len(f11_valid) == 1
        else:
            scenario_frame_pass = (
                len(f11_all) == 0 and
                len(f12_all) == 1 and len(f12_valid) == 1
            )
        coverage_pass = (
            fid9_pass and fid10_crc_pass and fid10_nfi_pass and
            fid10_payload_semantics_pass and payload_pass and scenario_frame_pass
        )
        cycle_report: dict[str, object] = {
            "cycle_instance": cycle_instance,
            "cycle": cycle_number,
            "predicted_fid9_start_sample": predicted_f9,
            "candidate_half_slot_samples": half_slot,
            "required_end_sample": required_end,
            "required_extent_source": required_extent_source,
            "fid9_frames": [frame_diagnostic(frame) for frame in f9_all],
            "fid9_decoded_count": len(f9_all),
            "fid9_crc_valid_count": len(f9_valid),
            "fid9_pass": fid9_pass,
            "fid10_frames": [frame_diagnostic(frame) for frame in f10_all],
            "fid10_decoded_count": len(f10_all),
            "fid10_crc_valid_count": len(f10_valid),
            "fid10_crc_pass": fid10_crc_pass,
            "fid10_nfi": f10_valid[0].nfi if len(f10_valid) == 1 else None,
            "fid10_payload_valid": (
                f10_valid[0].payload_valid if len(f10_valid) == 1 else None
            ),
            "fid10_nfi_pass": fid10_nfi_pass,
            "fid10_payload_semantics_pass": fid10_payload_semantics_pass,
            "fid10_payload_pass": payload_pass,
            "fid11_frames": [frame_diagnostic(frame) for frame in f11_all],
            "fid11_decoded_count": len(f11_all),
            "fid11_crc_valid_count": len(f11_valid),
            "fid12_frames": [frame_diagnostic(frame) for frame in f12_all],
            "fid12_decoded_count": len(f12_all),
            "fid12_crc_valid_count": len(f12_valid),
            "scenario_frame_pass": scenario_frame_pass,
            "coverage_pass": coverage_pass,
            "timing_pass": False,
            "txen_pass": None,
            "opposite_txen_pass": None,
            "pass": False,
        }
        complete_cycles.append(cycle_report)
        context: dict[str, object] = {
            "report": cycle_report,
            "predicted_f9": predicted_f9,
            "f9": f9_valid[0] if len(f9_valid) == 1 else (
                f9_all[0] if len(f9_all) == 1 else None
            ),
            "f10": f10_valid[0] if len(f10_valid) == 1 else (
                f10_all[0] if len(f10_all) == 1 else None
            ),
            "next": (
                f11_valid[0] if scenario == "with11" and len(f11_valid) == 1
                else f12_valid[0] if scenario == "without11" and len(f12_valid) == 1
                else None
            ),
        }
        cycle_contexts.append(context)

        if not (len(f9_valid) == 1 and len(f10_valid) == 1):
            continue
        if scenario == "with11":
            if len(f11_valid) != 1:
                continue
            selected = {9: f9_valid[0], 10: f10_valid[0], 11: f11_valid[0]}
            pair_specs = ((9, 10), (10, 11))
        else:
            if len(f12_valid) != 1:
                continue
            selected = {9: f9_valid[0], 10: f10_valid[0], 12: f12_valid[0]}
            pair_specs = ((9, 10), (10, 12))

        pair_results: list[dict[str, object]] = []
        pair_pass = True
        for first_id, second_id in pair_specs:
            first = selected[first_id]
            second = selected[second_id]
            slot_span = second_id - first_id
            tss_delta = second.start_sample - first.start_sample
            fss_delta = second.fss_start_sample - first.fss_start_sample
            expected_tss = None if tss_baseline is None else tss_baseline * slot_span
            expected_fss = None if fss_baseline is None else fss_baseline * slot_span
            tss_error = None if expected_tss is None else tss_delta - expected_tss
            fss_error = None if expected_fss is None else fss_delta - expected_fss
            passed = bool(
                tss_error is not None and fss_error is not None and
                abs(tss_error) <= tolerance_samples and
                abs(fss_error) <= tolerance_samples
            )
            pair_pass &= passed
            pair_results.append({
                "from_fid": first_id,
                "to_fid": second_id,
                "slot_span": slot_span,
                "tss_samples": tss_delta,
                "fss_samples": fss_delta,
                "expected_tss_samples": expected_tss,
                "expected_fss_samples": expected_fss,
                "tss_error_samples": tss_error,
                "fss_error_samples": fss_error,
                "pass": passed,
            })

        midpoint: dict[str, object] | None = None
        if scenario == "with11":
            f9, f10, f11 = selected[9], selected[10], selected[11]
            tss_skew = 2 * f10.start_sample - f9.start_sample - f11.start_sample
            fss_skew = (
                2 * f10.fss_start_sample -
                f9.fss_start_sample - f11.fss_start_sample
            )
            midpoint = {
                "tss_skew_samples": tss_skew,
                "tss_midpoint_error_samples": tss_skew / 2,
                "fss_skew_samples": fss_skew,
                "fss_midpoint_error_samples": fss_skew / 2,
                "pass": (
                    abs(tss_skew) <= 2 * tolerance_samples and
                    abs(fss_skew) <= 2 * tolerance_samples
                ),
            }
            pair_pass &= bool(midpoint["pass"])

        fid10_wire = _slot10_wire_metrics(selected[10], tolerance_samples)
        selected_wire = {
            str(frame_id): _slot10_wire_metrics(frame, tolerance_samples)
            for frame_id, frame in selected.items()
        }
        delimiter_pass = all(
            bool(metrics["fes_pass"]) and bool(metrics["cid_pass"])
            for metrics in selected_wire.values()
        )
        timing_wire_pass = bool(
            pair_pass and bool(fid10_wire["pass"]) and delimiter_pass and
            fid10_nfi_pass and fid10_payload_semantics_pass and payload_pass
        )
        occurrence = {
            "cycle_instance": cycle_instance,
            "cycle": cycle_number,
            # Retain the historical start_sample field for JSON consumers;
            # make both frame roles explicit alongside it.
            "start_sample": selected[9].start_sample,
            "fid9_start_sample": selected[9].start_sample,
            "fid10_start_sample": selected[10].start_sample,
            "pairs": pair_results,
            "midpoint": midpoint,
            "fid10_wire": fid10_wire,
            "selected_frame_wire": selected_wire,
            "delimiter_pass": delimiter_pass,
            "fid10_indicators": selected[10].indicators,
            "fid10_nfi": selected[10].nfi,
            "fid10_payload_valid": selected[10].payload_valid,
            "fid10_nfi_pass": fid10_nfi_pass,
            "fid10_payload_semantics_pass": fid10_payload_semantics_pass,
            "fid10_payload_hex": selected[10].payload.hex(),
            "payload_pass": payload_pass,
            "coverage_pass": coverage_pass,
            "timing_wire_pass": timing_wire_pass,
            "txen_pass": None,
            "pass": coverage_pass and timing_wire_pass,
        }
        cycle_report["occurrence_index"] = len(occurrences)
        cycle_report["timing_pass"] = bool(occurrence["pass"])
        occurrences.append(occurrence)

    coverage_all_pass = bool(complete_cycles) and all(
        bool(cycle["coverage_pass"]) for cycle in complete_cycles
    )

    txen_verification: dict[str, object]
    if txen_packed is None:
        txen_verification = {
            "evaluated": False,
            "channel": None,
            "active_level": "low",
            "reason": "USB CH0 was not captured",
            "pass": None,
            "cycles": [],
        }
        txen_all_pass = None
    else:
        low_runs = _active_low_runs(txen_packed, sample_count)
        txen_cycles: list[dict[str, object]] = []
        for context in cycle_contexts:
            cycle_report = context["report"]
            predicted_f9 = float(context["predicted_f9"])
            f9 = context["f9"]
            f10 = context["f10"]
            next_frame = context["next"]
            assert isinstance(cycle_report, dict)
            assert f9 is None or isinstance(f9, FlexRayFrame)
            assert f10 is None or isinstance(f10, FlexRayFrame)
            assert next_frame is None or isinstance(next_frame, FlexRayFrame)

            guard_start = int(math.floor(
                f9.end_sample if f9 is not None
                else predicted_f9 + float(frame_extent)
            ))
            earliest_start = guard_start - tolerance_samples
            predicted_slot_end = int(round(
                predicted_f9 + 2 * float(tss_baseline)
            ))
            if scenario == "with11" and next_frame is not None:
                release_deadline = next_frame.start_sample
                # The physical forwarder is expected to assert this same TXEN
                # again a few samples into F11.  Do not classify that normal
                # F11 run as a second slot10 envelope; an earlier generated
                # run that actually overlaps F11 is still selected by its
                # start and fails the release-deadline check below.
                target_window_end = release_deadline
            else:
                release_deadline = predicted_slot_end
                target_window_end = release_deadline + tolerance_samples
            latest_allowed_release = release_deadline + tolerance_samples

            target_runs = [
                run for run in low_runs
                if int(run["end_sample"]) > guard_start and
                int(run["start_sample"]) < target_window_end
            ]
            covering_runs = [] if f10 is None else [
                run for run in target_runs
                if int(run["start_sample"]) <= f10.start_sample and
                int(run["end_sample"]) >= f10.end_sample
            ]
            unique_run = target_runs[0] if len(target_runs) == 1 else None
            start_pass = bool(
                unique_run is not None and
                int(unique_run["start_sample"]) >= earliest_start
            )
            release_pass = bool(
                unique_run is not None and
                int(unique_run["end_sample"]) <= latest_allowed_release
            )
            if f10 is None:
                txen_pass = len(target_runs) == 0
            else:
                txen_pass = bool(
                    len(target_runs) == 1 and len(covering_runs) == 1 and
                    start_pass and release_pass
                )
            txen_cycle = {
                "cycle_instance": cycle_report["cycle_instance"],
                "cycle": cycle_report["cycle"],
                "guard_start_sample": guard_start,
                "earliest_allowed_start_sample": earliest_start,
                "predicted_slot11_start_sample": predicted_slot_end,
                "next_frame_tss_sample": (
                    None if next_frame is None else next_frame.start_sample
                ),
                "release_deadline_sample": release_deadline,
                "latest_allowed_release_sample": latest_allowed_release,
                "fid10_start_sample": None if f10 is None else f10.start_sample,
                "fid10_end_sample": None if f10 is None else f10.end_sample,
                "overlapping_low_run_count": len(target_runs),
                "covering_low_run_count": len(covering_runs),
                "low_runs": target_runs,
                "start_pass": start_pass if f10 is not None else None,
                "release_pass": release_pass if f10 is not None else None,
                "release_margin_samples": (
                    None if unique_run is None
                    else release_deadline - int(unique_run["end_sample"])
                ),
                "no_fid10_has_no_target_low": (
                    len(target_runs) == 0 if f10 is None else None
                ),
                "pass": txen_pass,
            }
            cycle_report["txen_pass"] = txen_pass
            if "occurrence_index" in cycle_report:
                occurrence = occurrences[int(cycle_report["occurrence_index"])]
                occurrence["txen_pass"] = txen_pass
                occurrence["pass"] = (
                    bool(occurrence["coverage_pass"]) and
                    bool(occurrence["timing_wire_pass"]) and txen_pass
                )
            txen_cycles.append(txen_cycle)

        txen_all_pass = bool(complete_cycles) and all(
            bool(cycle["pass"]) for cycle in txen_cycles
        )
        txen_verification = {
            "evaluated": True,
            "channel": 0,
            "active_level": "low",
            "low_run_count": len(low_runs),
            "complete_cycle_count": len(complete_cycles),
            "cycles": txen_cycles,
            "pass": txen_all_pass,
        }

    opposite_txen_verification: dict[str, object]
    if opposite_txen_packed is None:
        opposite_txen_verification = {
            "evaluated": False,
            "channel": opposite_txen_channel,
            "active_level": "low",
            "reason": "opposite-side TXEN channel was not captured",
            "pass": None,
            "cycles": [],
        }
        opposite_txen_all_pass = None
    else:
        opposite_low_runs = _active_low_runs(
            opposite_txen_packed, sample_count
        )
        opposite_cycles: list[dict[str, object]] = []
        for context in cycle_contexts:
            cycle_report = context["report"]
            predicted_f9 = float(context["predicted_f9"])
            f9 = context["f9"]
            next_frame = context["next"]
            assert isinstance(cycle_report, dict)
            assert f9 is None or isinstance(f9, FlexRayFrame)
            assert next_frame is None or isinstance(next_frame, FlexRayFrame)

            isolation_start = int(math.floor(
                f9.end_sample if f9 is not None
                else predicted_f9 + float(frame_extent)
            ))
            predicted_slot_end = int(round(
                predicted_f9 + 2 * float(tss_baseline)
            ))
            if scenario == "with11" and next_frame is not None:
                isolation_end = next_frame.start_sample
            else:
                isolation_end = predicted_slot_end + tolerance_samples
            target_runs = [
                run for run in opposite_low_runs
                if int(run["end_sample"]) > isolation_start and
                int(run["start_sample"]) < isolation_end
            ]
            isolated = len(target_runs) == 0
            cycle_report["opposite_txen_pass"] = isolated
            if "occurrence_index" in cycle_report:
                occurrence = occurrences[int(cycle_report["occurrence_index"])]
                occurrence["opposite_txen_pass"] = isolated
            opposite_cycles.append({
                "cycle_instance": cycle_report["cycle_instance"],
                "cycle": cycle_report["cycle"],
                "isolation_start_sample": isolation_start,
                "isolation_end_sample": isolation_end,
                "overlapping_low_run_count": len(target_runs),
                "low_runs": target_runs,
                "pass": isolated,
            })
        opposite_txen_all_pass = bool(complete_cycles) and all(
            bool(cycle["pass"]) for cycle in opposite_cycles
        )
        opposite_txen_verification = {
            "evaluated": True,
            "channel": opposite_txen_channel,
            "active_level": "low",
            "requirement": "remain recessive throughout the FID10 opportunity",
            "low_run_count": len(opposite_low_runs),
            "complete_cycle_count": len(complete_cycles),
            "cycles": opposite_cycles,
            "pass": opposite_txen_all_pass,
        }

    occurrence_by_instance = {
        int(occurrence["cycle_instance"]): occurrence for occurrence in occurrences
    }
    for cycle in complete_cycles:
        occurrence = occurrence_by_instance.get(int(cycle["cycle_instance"]))
        cycle["timing_pass"] = bool(
            occurrence and occurrence["timing_wire_pass"]
        )
        txen_pass = True if cycle["txen_pass"] is None else bool(cycle["txen_pass"])
        opposite_txen_pass = (
            True if cycle["opposite_txen_pass"] is None
            else bool(cycle["opposite_txen_pass"])
        )
        cycle["pass"] = bool(
            cycle["coverage_pass"] and cycle["timing_pass"] and txen_pass and
            opposite_txen_pass
        )
        if occurrence is not None:
            occurrence["pass"] = bool(
                occurrence["coverage_pass"] and
                occurrence["timing_wire_pass"] and txen_pass and
                opposite_txen_pass
            )

    normal_consistent = bool(normal_pairs) and all(
        bool(pair["pass"]) for pair in normal_pairs
    )
    frame11_count = sum(frame.frame_id == 11 for frame in valid)
    frame11_decoded_count = sum(frame.frame_id == 11 for frame in decoded)
    absence_pass = scenario != "without11" or frame11_decoded_count == 0
    fid10_wire_all_pass = (
        bool(complete_cycles) and len(occurrences) == len(complete_cycles) and all(
            bool(occurrence["fid10_wire"]["pass"])
            for occurrence in occurrences
        )
    )
    cycle_sequence_contiguous = (
        cycle_period_evidence_sufficient and not cycle_sequence_issues
    )
    overall_pass = (
        bool(complete_cycles) and normal_consistent and
        cycle_sequence_contiguous and absence_pass and all(
            bool(cycle["pass"]) for cycle in complete_cycles
        )
    )
    return {
        "scenario": scenario,
        "tolerance_samples": tolerance_samples,
        "valid_frame_count": len(valid),
        "normal_pair_count": len(normal_pairs),
        "normal_tss_baseline_samples": tss_baseline,
        "normal_fss_baseline_samples": fss_baseline,
        "normal_pairs_consistent": normal_consistent,
        "normal_pairs": normal_pairs,
        "capture_sample_count": sample_count,
        "capture_head_margin_samples": capture_head_margin,
        "cycle_period_samples": cycle_period,
        "expected_cycle_period_samples": expected_cycle_period,
        "expected_cycle_tolerance_samples": expected_cycle_tolerance,
        "cycle_period_evidence_sufficient": cycle_period_evidence_sufficient,
        "cycle_sequence_contiguous": cycle_sequence_contiguous,
        "cycle_sequence_issues": cycle_sequence_issues,
        "complete_cycle_count": len(complete_cycles),
        "complete_cycles": complete_cycles,
        "excluded_cycle_count": len(excluded_cycles),
        "excluded_cycles": excluded_cycles,
        "coverage_all_pass": coverage_all_pass,
        "occurrence_count": len(occurrences),
        "occurrences": occurrences,
        "fid10_wire_all_pass": fid10_wire_all_pass,
        "frame11_count": frame11_count,
        "frame11_decoded_count": frame11_decoded_count,
        "frame11_absence_pass": absence_pass,
        "txen_verification": txen_verification,
        "txen_all_pass": txen_all_pass,
        "opposite_txen_verification": opposite_txen_verification,
        "opposite_txen_all_pass": opposite_txen_all_pass,
        "expected_fid10_payload_hex": (
            None if expected_payload is None else expected_payload.hex()
        ),
        "expected_fid10_nfi": expected_nfi,
        "pass": overall_pass,
    }


def _print_slot10_verification(report: dict[str, object], sample_rate: int) -> None:
    tss = report["normal_tss_baseline_samples"]
    fss = report["normal_fss_baseline_samples"]
    to_ns = 1e9 / sample_rate
    print(
        f"Slot10 {report['scenario']} verification: "
        f"{'PASS' if report['pass'] else 'FAIL'}; "
        f"ordinary pairs={report['normal_pair_count']} "
        f"consistent={report['normal_pairs_consistent']}"
    )
    expected_nfi = report["expected_fid10_nfi"]
    print(
        "  FID10 NFI expectation="
        f"{'any' if expected_nfi is None else expected_nfi}; "
        "NFI=0 requires an all-zero payload"
    )
    print(
        f"  cycle period={report['cycle_period_samples']} "
        f"expected={report['expected_cycle_period_samples']}"
        f"±{report['expected_cycle_tolerance_samples']} samples "
        f"evidence={'YES' if report['cycle_period_evidence_sufficient'] else 'NO'} "
        f"sequence={'PASS' if report['cycle_sequence_contiguous'] else 'FAIL'} "
        f"F11-absence={'PASS' if report['frame11_absence_pass'] else 'FAIL'}"
    )
    for issue in report["cycle_sequence_issues"][:4]:
        print(
            f"    cycle gap {issue['from_cycle']}->{issue['to_cycle']} "
            f"delta={issue['cycle_delta']} "
            f"period={issue['unit_period_samples']} "
            f"reason={issue['reason']}"
        )
    if tss is not None and fss is not None:
        print(
            f"  ordinary median: TSS-start {tss:g} samples/{tss * to_ns:g} ns, "
            f"FSS-start {fss:g} samples/{fss * to_ns:g} ns"
        )
    print(
        f"  complete cycles={report['complete_cycle_count']} "
        f"excluded boundary cycles={report['excluded_cycle_count']} "
        f"coverage={'PASS' if report['coverage_all_pass'] else 'FAIL'}; "
        f"matched cycles={report['occurrence_count']} "
        f"FID11 valid/decoded={report['frame11_count']}/"
        f"{report['frame11_decoded_count']} "
        f"F10-wire={'PASS' if report['fid10_wire_all_pass'] else 'FAIL'}"
    )
    failed_cycles = [
        cycle for cycle in report["complete_cycles"] if not cycle["pass"]
    ]
    for cycle in failed_cycles[:8]:
        print(
            f"  cycle={cycle['cycle']}#{cycle['cycle_instance']} coverage "
            f"F9={cycle['fid9_crc_valid_count']}/{cycle['fid9_decoded_count']} "
            f"F10={cycle['fid10_crc_valid_count']}/{cycle['fid10_decoded_count']} "
            f"NFI={cycle['fid10_nfi']} "
            f"nfi={'OK' if cycle['fid10_nfi_pass'] else 'BAD'} "
            f"semantics={'OK' if cycle['fid10_payload_semantics_pass'] else 'BAD'} "
            f"payload={'OK' if cycle['fid10_payload_pass'] else 'BAD'} "
            f"F11={cycle['fid11_crc_valid_count']}/{cycle['fid11_decoded_count']} "
            f"F12={cycle['fid12_crc_valid_count']}/{cycle['fid12_decoded_count']} "
            f"timing={'PASS' if cycle['timing_pass'] else 'FAIL'} "
            f"TXEN={('n/a' if cycle['txen_pass'] is None else 'PASS' if cycle['txen_pass'] else 'FAIL')} "
            "FAIL"
        )
    for cycle in report["excluded_cycles"][:4]:
        print(
            f"  excluded cycle={cycle['cycle']}#{cycle['cycle_instance']} "
            f"predicted-F9={cycle['predicted_fid9_start_sample']:g} "
            f"required-end={cycle['required_end_sample']} "
            f"reason={cycle['reason']}"
        )

    txen = report["txen_verification"]
    if txen["evaluated"]:
        print(
            f"  CH0 TXEN active-low: {'PASS' if txen['pass'] else 'FAIL'}; "
            f"low runs={txen['low_run_count']}"
        )
        failed_txen = [cycle for cycle in txen["cycles"] if not cycle["pass"]]
        for cycle in failed_txen[:8]:
            print(
                f"    cycle={cycle['cycle']}#{cycle['cycle_instance']} "
                f"target-runs={cycle['overlapping_low_run_count']} "
                f"covering={cycle['covering_low_run_count']} "
                f"F10={cycle['fid10_start_sample']}..{cycle['fid10_end_sample']} "
                f"deadline={cycle['release_deadline_sample']} "
                f"margin={cycle['release_margin_samples']} FAIL"
            )
    else:
        print(f"  CH0 TXEN: not evaluated ({txen['reason']})")

    opposite_txen = report["opposite_txen_verification"]
    if opposite_txen["evaluated"]:
        print(
            f"  CH{opposite_txen['channel']} opposite TXEN active-low isolation: "
            f"{'PASS' if opposite_txen['pass'] else 'FAIL'}; "
            f"all-capture low runs={opposite_txen['low_run_count']}"
        )
        failed_opposite = [
            cycle for cycle in opposite_txen["cycles"] if not cycle["pass"]
        ]
        for cycle in failed_opposite[:8]:
            print(
                f"    cycle={cycle['cycle']}#{cycle['cycle_instance']} "
                f"target-runs={cycle['overlapping_low_run_count']} "
                f"window={cycle['isolation_start_sample']}.."
                f"{cycle['isolation_end_sample']} FAIL"
            )
    else:
        print(
            "  Opposite TXEN: not evaluated "
            f"({opposite_txen['reason']})"
        )

    for occurrence in report["occurrences"][:8]:
        def error_text(value: object) -> str:
            return "n/a" if value is None else f"{float(value):+g}"

        pair_text = ", ".join(
            f"{pair['from_fid']}->{pair['to_fid']} "
            f"TSS={pair['tss_samples']}({error_text(pair['tss_error_samples'])}) "
            f"FSS={pair['fss_samples']}({error_text(pair['fss_error_samples'])})"
            for pair in occurrence["pairs"]
        )
        midpoint = occurrence["midpoint"]
        midpoint_text = "" if midpoint is None else (
            f", midpoint skew TSS={midpoint['tss_skew_samples']:+d} "
            f"FSS={midpoint['fss_skew_samples']:+d} samples"
        )
        wire = occurrence["fid10_wire"]
        print(
            f"  cycle={occurrence['cycle']} {pair_text}{midpoint_text}; "
            f"FES/CID={'OK' if occurrence['delimiter_pass'] else 'BAD'} "
            f"F10={occurrence['fid10_payload_hex']} "
            f"ind=0x{occurrence['fid10_indicators']:02x} "
            f"NFI={occurrence['fid10_nfi']} "
            f"semantics={'OK' if occurrence['fid10_payload_semantics_pass'] else 'BAD'} "
            f"{'PASS' if occurrence['pass'] else 'FAIL'}"
        )
        print(
            f"    F10 wire: TSS@{wire['tss_start_sample']} "
            f"len={wire['tss_samples']}({error_text(wire['tss_residual_samples'])}); "
            f"FSS@{wire['fss_start_sample']}({error_text(wire['fss_residual_samples'])}) "
            f"FBSS@{wire['fbss_fall_sample']}({error_text(wire['fbss_residual_samples'])}); "
            f"FES@{wire['fes_start_sample']} "
            f"fall={wire['fes_fall_sample']}({error_text(wire['fes_fall_residual_samples'])}) "
            f"center@{wire['fes_center_sample']}="
            f"{'LOW' if wire['fes_center_low'] else 'BAD'}; "
            f"CID@{wire['cid_rise_sample']}({error_text(wire['cid_rise_residual_samples'])}) "
            f"end={wire['cid_end_sample']} "
            f"11-high={'YES' if wire['cid_bits_high'] else 'NO'} "
            f"{'PASS' if wire['pass'] else 'FAIL'}"
        )


class ATKLogic:
    def __init__(self, serial: str | None = None) -> None:
        self.device = usb.core.find(
            idVendor=VID,
            idProduct=PID,
            custom_match=(
                None
                if serial is None
                else lambda dev: usb.util.get_string(dev, dev.iSerialNumber) == serial
            ),
        )
        if self.device is None:
            suffix = "" if serial is None else f" with serial {serial}"
            raise RuntimeError(f"ATK-Logic {VID:04x}:{PID:04x}{suffix} not found")
        self._claimed = False

    @property
    def serial(self) -> str:
        return usb.util.get_string(self.device, self.device.iSerialNumber)

    def open(self) -> None:
        try:
            self.device.set_configuration()
        except usb.core.USBError as exc:
            # macOS/libusb may report BUSY when configuration 1 is already active.
            if getattr(exc, "errno", None) not in (16,):
                raise
        usb.util.claim_interface(self.device, INTERFACE)
        self._claimed = True

    def close(self) -> None:
        try:
            if self._claimed:
                usb.util.release_interface(self.device, INTERFACE)
                self._claimed = False
        finally:
            usb.util.dispose_resources(self.device)

    def __enter__(self) -> "ATKLogic":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def direct_write(self, code: int, payload: bytes = b"") -> int:
        return int(self.device.write(EP_OUT, _direct_packet(code, payload), timeout=1000))

    def write_command(self, code: int, payload: bytes = b"") -> int:
        wire = _encode_command(code, payload)
        return int(self.device.write(EP_OUT, wire, timeout=1000))

    def read(self, size: int, timeout_ms: int) -> bytes:
        return bytes(self.device.read(EP_IN, size, timeout=timeout_ms))

    def drain(
        self,
        *,
        read_size: int = 32_768,
        timeout_ms: int = 20,
        max_reads: int = 64,
        max_bytes: int = 8 * 1024 * 1024,
    ) -> int:
        count = 0
        for _ in range(max_reads):
            try:
                count += len(self.read(read_size, timeout_ms))
            except usb.core.USBTimeoutError:
                return count
            if count >= max_bytes:
                return count
        return count


def _printable(data: bytes) -> str:
    return "".join(chr(value) if 32 <= value < 127 else "." for value in data)


def _parse_channels(value: str) -> list[int]:
    try:
        result = sorted({int(item, 0) for item in value.split(",")})
    except ValueError as exc:
        raise argparse.ArgumentTypeError("channels must be comma-separated integers") from exc
    if not result or any(channel < 0 or channel > 15 for channel in result):
        raise argparse.ArgumentTypeError("channels must be a non-empty subset of 0..15")
    return result


def _parse_rate(value: str) -> int:
    suffixes = {"k": 1_000, "m": 1_000_000, "g": 1_000_000_000}
    normalized = value.strip().lower().replace("hz", "")
    multiplier = 1
    if normalized and normalized[-1] in suffixes:
        multiplier = suffixes[normalized[-1]]
        normalized = normalized[:-1]
    try:
        rate = int(float(normalized) * multiplier)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid sample rate: {value}") from exc
    if rate not in RATE_INDEX:
        choices = ", ".join(f"{item // 1_000_000}M" for item in RATE_INDEX)
        raise argparse.ArgumentTypeError(f"unsupported rate; choose one of {choices}")
    return rate


def _parse_hex_bytes(value: str) -> bytes:
    normalized = value.replace(" ", "").replace(":", "").replace("_", "")
    try:
        result = bytes.fromhex(normalized)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("payload must be hexadecimal bytes") from exc
    if not result:
        raise argparse.ArgumentTypeError("payload cannot be empty")
    return result


def _unpack_sample(packed: bytes, sample: int) -> int:
    return (packed[sample >> 3] >> (sample & 7)) & 1


def _edge_iterator(packed: bytes, sample_count: int) -> Iterator[tuple[int, int]]:
    if not packed or sample_count <= 0:
        return
    previous = packed[0] & 1
    for sample in range(1, sample_count):
        current = _unpack_sample(packed, sample)
        if current != previous:
            yield sample, current
            previous = current


def _write_vcd(path: Path, channels: dict[int, bytes], rate: int, sample_count: int) -> None:
    if 1_000_000_000 % rate:
        raise ValueError("VCD writer requires an integral nanosecond sample period")
    period_ns = 1_000_000_000 // rate
    identifiers = [chr(33 + index) for index in range(len(channels))]
    items = list(zip(sorted(channels), identifiers))
    iterators = {channel: iter(_edge_iterator(channels[channel], sample_count)) for channel, _ in items}
    next_edges: dict[int, tuple[int, int] | None] = {
        channel: next(iterator, None) for channel, iterator in iterators.items()
    }

    with path.open("w", encoding="ascii", newline="\n") as output:
        output.write("$date\n")
        output.write(f"  {dt.datetime.now().astimezone().isoformat()}\n")
        output.write("$end\n$version\n  atk_logic_headless.py\n$end\n")
        output.write("$timescale 1ns $end\n$scope module logic $end\n")
        for channel, identifier in items:
            output.write(f"$var wire 1 {identifier} CH{channel} $end\n")
        output.write("$upscope $end\n$enddefinitions $end\n#0\n$dumpvars\n")
        for channel, identifier in items:
            output.write(f"{_unpack_sample(channels[channel], 0)}{identifier}\n")
        output.write("$end\n")

        while True:
            active = [edge for edge in next_edges.values() if edge is not None]
            if not active:
                break
            sample = min(edge[0] for edge in active)
            output.write(f"#{sample * period_ns}\n")
            for channel, identifier in items:
                edge = next_edges[channel]
                if edge is not None and edge[0] == sample:
                    output.write(f"{edge[1]}{identifier}\n")
                    next_edges[channel] = next(iterators[channel], None)


def _edge_summary(packed: bytes, sample_count: int, limit: int = 8) -> tuple[int, list[int]]:
    count = 0
    first: list[int] = []
    for sample, _ in _edge_iterator(packed, sample_count):
        count += 1
        if len(first) < limit:
            first.append(sample)
    return count, first


def _validate_stream_rate(rate: int, channel_count: int) -> None:
    maximum = 100_000_000 if channel_count <= 3 else 50_000_000 if channel_count <= 6 else 20_000_000
    if rate > maximum:
        raise ValueError(
            f"DL16 Plus Stream limit for {channel_count} channels is {maximum // 1_000_000} MHz"
        )


def command_probe(args: argparse.Namespace) -> int:
    with ATKLogic(args.serial) as analyzer:
        device = analyzer.device
        print(
            f"ATK-Logic found: serial={analyzer.serial} bus={device.bus} address={device.address} "
            f"VID:PID={VID:04x}:{PID:04x}"
        )
        analyzer.drain(read_size=512)
        analyzer.direct_write(0x81)  # MCU version query
        try:
            reply = analyzer.read(512, 1000)
        except usb.core.USBTimeoutError:
            print("MCU version query timed out", file=sys.stderr)
            return 2
        print(f"MCU reply ({len(reply)} bytes): {reply[:64].hex(' ')}")
        print(f"MCU reply text: {_printable(reply[:64])}")
    return 0


def command_capture(args: argparse.Namespace) -> int:
    channels: list[int] = args.channels
    for option, channel in (
        ("--opposite-txen-channel", args.opposite_txen_channel),
        ("--opposite-flexray-channel", args.opposite_flexray_channel),
    ):
        if channel is not None and channel not in channels:
            raise ValueError(f"{option} must be included in --channels")
    if (args.opposite_flexray_channel is not None and
            args.opposite_flexray_channel == args.flexray_channel):
        raise ValueError("primary and opposite FlexRay channels must differ")
    if args.buffer:
        raise ValueError(
            "Buffer mode is not yet exported safely; use Stream mode (omit --buffer)"
        )
    if not args.buffer:
        _validate_stream_rate(args.rate, len(channels))
    sample_count = round(args.rate * args.duration_ms / 1000)
    if sample_count < 8:
        raise ValueError("capture duration is too short")
    bytes_per_channel = (sample_count + 7) // 8
    trigger_samples = round(sample_count * args.trigger_position / 100)
    parameter = _parameter_payload(
        rate=args.rate,
        samples=sample_count,
        trigger_samples=trigger_samples,
        threshold=args.threshold,
        buffer=args.buffer,
        rle=False,
    )
    trigger = _trigger_payload(channels, args.trigger_channel, args.trigger, args.instant)

    prefix = args.output
    if prefix is None:
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        prefix = Path("captures") / f"atk-{stamp}"
    prefix = prefix.expanduser().resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)

    channel_data = {channel: bytearray() for channel in channels}
    parser = ReplyParser()
    raw_usb = bytearray()
    pending_usb = bytearray()
    acknowledgements: list[int] = []
    offsets: list[str] = []
    finished = False
    stream_overflow = False
    started = False

    actual_serial = ""
    with ATKLogic(args.serial) as analyzer:
        actual_serial = analyzer.serial
        try:
            # Recover cleanly if a previous process died during a Stream
            # capture. Both operations are intentionally bounded.
            analyzer.write_command(0x15)
            time.sleep(0.050)
            analyzer.drain(read_size=32_768, max_reads=16)
            analyzer.direct_write(0x87, b"\x01")  # wake FPGA
            try:
                wake_reply = analyzer.read(512, 1000)
                print(f"FPGA wake reply: {wake_reply[:16].hex(' ')}")
            except usb.core.USBTimeoutError:
                print("Warning: no explicit FPGA wake reply", file=sys.stderr)
            analyzer.drain()

            analyzer.write_command(0x11, parameter)
            time.sleep(0.030)  # required by the vendor application
            analyzer.write_command(0x12, trigger)
            started = True
            deadline = time.monotonic() + args.timeout
            print(
                f"Capturing CH{','.join(map(str, channels))} at {args.rate / 1e6:g} MHz, "
                f"{args.duration_ms:g} ms, "
                + ("instant" if args.instant else f"CH{args.trigger_channel} {args.trigger} trigger")
            )

            stop_sent = False
            while time.monotonic() < deadline:
                try:
                    usb_chunk = analyzer.read(32_768, 250)
                except usb.core.USBTimeoutError:
                    continue
                raw_usb.extend(usb_chunk)
                pending_usb.extend(usb_chunk)
                logical = bytearray()
                while len(pending_usb) >= USB_BLOCK:
                    logical.extend(_device_to_host_block(bytes(pending_usb[:USB_BLOCK])))
                    del pending_usb[:USB_BLOCK]

                for reply in parser.feed(logical):
                    if len(reply.payload) < 2:
                        continue
                    if reply.order == 1:
                        channel = reply.payload[0]
                        if channel in channel_data:
                            channel_data[channel].extend(reply.payload[2:])
                    elif reply.order == 3:
                        offsets.append(reply.payload.hex())
                    elif reply.order == 4 and len(reply.payload) >= 3:
                        command = reply.payload[2]
                        acknowledgements.append(command)
                        if command == 0x12:
                            if len(reply.payload) < 4 or reply.payload[3] != 3:
                                status = None if len(reply.payload) < 4 else reply.payload[3]
                                raise ProtocolError(f"capture start rejected (status={status})")
                    elif reply.order == 6:
                        finished = True
                        if len(reply.payload) >= 3 and reply.payload[2] == 1:
                            stream_overflow = True
                            raise ProtocolError("analyzer reported Stream capacity/bandwidth overflow")

                have_target = all(len(channel_data[channel]) >= bytes_per_channel for channel in channels)
                if have_target and not stop_sent:
                    analyzer.write_command(0x15)
                    stop_sent = True
                    deadline = min(deadline, time.monotonic() + 1.0)
                if finished or (stop_sent and 0x15 in acknowledgements):
                    break

            if not all(len(channel_data[channel]) >= bytes_per_channel for channel in channels):
                received = ", ".join(
                    f"CH{channel}={len(channel_data[channel]) * 8} samples" for channel in channels
                )
                mode = "trigger did not fire or " if not args.instant else ""
                raise TimeoutError(f"{mode}capture incomplete ({received})")
        finally:
            if started:
                try:
                    analyzer.write_command(0x15)
                    time.sleep(0.050)
                    analyzer.drain()
                except usb.core.USBError:
                    pass
            try:
                analyzer.direct_write(0x87, b"\x00")  # sleep FPGA
            except usb.core.USBError:
                pass

    packed_channels: dict[int, bytes] = {}
    for channel in channels:
        packed = bytes(channel_data[channel][:bytes_per_channel])
        if sample_count & 7:
            final = packed[-1] & ((1 << (sample_count & 7)) - 1)
            packed = packed[:-1] + bytes((final,))
        packed_channels[channel] = packed
        packed_path = Path(f"{prefix}.ch{channel}.bin")
        packed_path.write_bytes(packed)

    raw_path = Path(f"{prefix}.usb.bin")
    raw_path.write_bytes(raw_usb)
    vcd_path = Path(f"{prefix}.vcd")
    if not args.no_vcd:
        _write_vcd(vcd_path, packed_channels, args.rate, sample_count)

    summaries = {}
    for channel in channels:
        edge_count, first_edges = _edge_summary(packed_channels[channel], sample_count)
        high_samples = sum(value.bit_count() for value in packed_channels[channel])
        summaries[str(channel)] = {
            "initial_level": _unpack_sample(packed_channels[channel], 0),
            "high_samples": high_samples,
            "edges": edge_count,
            "first_edge_samples": first_edges,
        }
        times_us = [round(sample / args.rate * 1e6, 6) for sample in first_edges]
        print(
            f"USB CH{channel} (panel CH{channel + 1}): "
            f"initial={_unpack_sample(packed_channels[channel], 0)}, "
            f"high={high_samples / sample_count:.1%}, {edge_count} edges; "
            f"first edge times (us): {times_us}"
        )

    decoded_frames: list[FlexRayFrame] = []
    opposite_decoded_frames: list[FlexRayFrame] = []
    slot10_verification: dict[str, object] | None = None
    if args.flexray_channel is not None:
        if args.flexray_channel not in packed_channels:
            raise ValueError("--flexray-channel must be included in --channels")
        decoded_frames = _decode_flexray_frames(
            packed_channels[args.flexray_channel],
            sample_count,
            args.rate,
            args.flexray_bitrate,
            args.flexray_channel_type,
        )
        if decoded_frames:
            print(f"FlexRay CH{args.flexray_channel}: {len(decoded_frames)} frame(s)")
            for frame in decoded_frames:
                print(
                    f"  t={frame.start_sample / args.rate * 1e6:.3f} us "
                    f"FID={frame.frame_id} cycle={frame.cycle} "
                    f"ind=0x{frame.indicators:02x} NFI={frame.nfi} "
                    f"payload-valid={'YES' if frame.payload_valid else 'NO'} "
                    f"payload={frame.payload.hex(' ')} "
                    f"semantics={'OK' if frame.payload_valid or not any(frame.payload) else 'BAD'} "
                    f"TSS={frame.tss_bits:g} bit "
                    f"FES/CID={'OK' if frame.fes_ok and frame.cid_ok else 'BAD'} "
                    f"HCRC={'OK' if frame.header_crc_ok else 'BAD'} "
                    f"FCRC={'OK' if frame.frame_crc_ok else 'BAD'}"
                )
        else:
            print(f"FlexRay CH{args.flexray_channel}: no complete frame decoded")

        if args.verify_slot10 is not None:
            if args.flexray_channel == 0 and 0 in packed_channels:
                raise ValueError(
                    "slot10 verification reserves USB CH0 for active-low TXEN; "
                    "decode FlexRay from a different captured channel"
                )
            slot10_verification = _verify_slot10_timing(
                decoded_frames,
                args.verify_slot10,
                args.slot_tolerance_samples,
                args.expect_fid10_payload,
                sample_count,
                packed_channels.get(0),
                None if args.opposite_txen_channel is None else
                packed_channels.get(args.opposite_txen_channel),
                args.opposite_txen_channel,
                expected_nfi=args.expect_fid10_nfi,
            )
            _print_slot10_verification(slot10_verification, args.rate)
    elif args.verify_slot10 is not None:
        raise ValueError("--verify-slot10 requires --flexray-channel")

    if args.opposite_flexray_channel is not None:
        opposite_decoded_frames = _decode_flexray_frames(
            packed_channels[args.opposite_flexray_channel],
            sample_count,
            args.rate,
            args.flexray_bitrate,
            args.flexray_channel_type,
        )
        print(
            f"Opposite FlexRay CH{args.opposite_flexray_channel}: "
            f"{len(opposite_decoded_frames)} frame(s)"
        )
        for frame in opposite_decoded_frames:
            print(
                f"  t={frame.start_sample / args.rate * 1e6:.3f} us "
                f"FID={frame.frame_id} cycle={frame.cycle} "
                f"ind=0x{frame.indicators:02x} NFI={frame.nfi} "
                f"payload-valid={'YES' if frame.payload_valid else 'NO'} "
                f"payload={frame.payload.hex(' ')} "
                f"semantics={'OK' if frame.payload_valid or not any(frame.payload) else 'BAD'} "
                f"HCRC={'OK' if frame.header_crc_ok else 'BAD'} "
                f"FCRC={'OK' if frame.frame_crc_ok else 'BAD'}"
            )

    metadata = {
        "serial": actual_serial,
        "rate_hz": args.rate,
        "duration_ms": args.duration_ms,
        "sample_count": sample_count,
        "channels": channels,
        "threshold_volts": args.threshold,
        "mode": "buffer" if args.buffer else "stream",
        "instant": args.instant,
        "trigger": None
        if args.instant
        else {"channel": args.trigger_channel, "kind": args.trigger, "position_percent": args.trigger_position},
        "acknowledgements": acknowledgements,
        "offset_packets": offsets,
        "finished_packet": finished,
        "stream_overflow": stream_overflow,
        "summaries": summaries,
        "flexray_frames": [
            {
                "start_sample": frame.start_sample,
                "start_us": frame.start_sample / args.rate * 1e6,
                "fss_start_sample": frame.fss_start_sample,
                "fss_start_us": frame.fss_start_sample / args.rate * 1e6,
                "end_sample": frame.end_sample,
                "samples_per_bit": frame.samples_per_bit,
                "tss_bits": frame.tss_bits,
                "fss_center_high": frame.fss_center_high,
                "first_bss_ok": frame.first_bss_ok,
                "fbss_fall_sample": frame.fbss_fall_sample,
                "fes_start_sample": frame.fes_start_sample,
                "fes_fall_sample": frame.fes_fall_sample,
                "cid_start_sample": frame.cid_start_sample,
                "cid_rise_sample": frame.cid_rise_sample,
                "fes_center_low": frame.fes_center_low,
                "cid_bits_high": frame.cid_bits_high,
                "fes_ok": frame.fes_ok,
                "cid_ok": frame.cid_ok,
                "indicators": frame.indicators,
                "nfi": frame.nfi,
                "payload_valid": frame.payload_valid,
                "payload_semantics_ok": (
                    frame.payload_valid or not any(frame.payload)
                ),
                "frame_id": frame.frame_id,
                "cycle": frame.cycle,
                "payload_hex": frame.payload.hex(),
                "header_crc": frame.header_crc,
                "header_crc_ok": frame.header_crc_ok,
                "frame_crc": frame.frame_crc,
                "frame_crc_ok": frame.frame_crc_ok,
            }
            for frame in decoded_frames
        ],
        "opposite_flexray_channel": args.opposite_flexray_channel,
        "opposite_flexray_frames": [
            {
                "start_sample": frame.start_sample,
                "start_us": frame.start_sample / args.rate * 1e6,
                "end_sample": frame.end_sample,
                "frame_id": frame.frame_id,
                "cycle": frame.cycle,
                "indicators": frame.indicators,
                "nfi": frame.nfi,
                "payload_valid": frame.payload_valid,
                "payload_semantics_ok": (
                    frame.payload_valid or not any(frame.payload)
                ),
                "payload_hex": frame.payload.hex(),
                "header_crc_ok": frame.header_crc_ok,
                "frame_crc_ok": frame.frame_crc_ok,
                "fes_ok": frame.fes_ok,
                "cid_ok": frame.cid_ok,
            }
            for frame in opposite_decoded_frames
        ],
        "slot10_verification": slot10_verification,
        "packed_format": "LSB-first, one bit per sample, one file per channel",
    }
    metadata_path = Path(f"{prefix}.json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    outputs = [metadata_path, raw_path]
    outputs.extend(Path(f"{prefix}.ch{channel}.bin") for channel in channels)
    if not args.no_vcd:
        outputs.append(vcd_path)
    print("Saved:")
    for output in outputs:
        print(f"  {output}")
    if slot10_verification is not None and not slot10_verification["pass"]:
        return 2
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="select analyzer by USB serial number")
    subparsers = parser.add_subparsers(dest="command", required=True)

    probe = subparsers.add_parser("probe", help="find the analyzer and query its MCU version")
    probe.set_defaults(handler=command_probe)

    capture = subparsers.add_parser("capture", help="capture packed samples without the GUI")
    capture.add_argument("--output", type=Path, help="output prefix (default: captures/atk-TIMESTAMP)")
    capture.add_argument(
        "--channels",
        type=_parse_channels,
        default=[0, 1],
        help="zero-based USB channels; panel CHn is USB n-1 (default: 0,1)",
    )
    capture.add_argument("--rate", type=_parse_rate, default=100_000_000, help="sample rate (default: 100M)")
    capture.add_argument("--duration-ms", type=float, default=100.0, help="capture duration per channel")
    capture.add_argument("--threshold", type=float, default=1.6, help="logic threshold in volts")
    capture.add_argument("--trigger-channel", type=int, default=0, help="simple-trigger channel")
    capture.add_argument(
        "--trigger",
        choices=tuple(TRIGGER_BITS_EVEN),
        default="falling",
        help="simple-trigger condition (default: falling)",
    )
    capture.add_argument("--trigger-position", type=float, default=10.0, help="trigger position percent")
    capture.add_argument("--instant", action="store_true", help="start immediately and ignore trigger")
    capture.add_argument(
        "--buffer",
        action="store_true",
        help="reserved; currently rejected because ring-buffer trigger offsets are not exported",
    )
    capture.add_argument("--timeout", type=float, default=10.0, help="overall arm/capture timeout")
    capture.add_argument("--no-vcd", action="store_true", help="skip VCD generation")
    capture.add_argument("--flexray-channel", type=int, help="decode FlexRay from this captured channel")
    capture.add_argument(
        "--flexray-bitrate", type=_parse_rate, default=10_000_000, help="FlexRay bitrate (default: 10M)"
    )
    capture.add_argument(
        "--flexray-channel-type", choices=("A", "B"), default="A", help="FlexRay CRC channel (default: A)"
    )
    capture.add_argument(
        "--opposite-txen-channel",
        type=int,
        help=(
            "active-low opposite-side TXEN channel; slot10 verification "
            "requires it to remain high throughout the FID10 opportunity"
        ),
    )
    capture.add_argument(
        "--opposite-flexray-channel",
        type=int,
        help="optional opposite-side TXD channel to decode and report",
    )
    capture.add_argument(
        "--verify-slot10",
        choices=("with11", "without11"),
        help=(
            "verify every complete slot10 cycle and FES/CID; when USB CH0 is "
            "captured it is also checked as active-low TXEN"
        ),
    )
    capture.add_argument(
        "--slot-tolerance-samples",
        type=int,
        default=2,
        help="maximum edge-interval residual at the analyzer sample grid (default: 2)",
    )
    capture.add_argument(
        "--expect-fid10-payload",
        type=_parse_hex_bytes,
        help="optional exact FID10 payload bytes, for example 0a0a",
    )
    capture.add_argument(
        "--expect-fid10-nfi",
        type=int,
        choices=(0, 1),
        help=(
            "optional exact FID10 null-frame indicator: 1 means payload data "
            "is valid; 0 means a null frame with an all-zero payload"
        ),
    )
    capture.set_defaults(handler=command_capture)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.handler(args))
    except (ProtocolError, RuntimeError, TimeoutError, ValueError, usb.core.USBError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
