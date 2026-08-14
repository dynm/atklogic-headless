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
    if (args.opposite_flexray_channel is not None and
            args.opposite_flexray_channel not in channels):
        raise ValueError("--opposite-flexray-channel must be included in --channels")
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
        "--opposite-flexray-channel",
        type=int,
        help="optional opposite-side TXD channel to decode and report",
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
