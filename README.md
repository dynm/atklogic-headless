# atklogic-headless

[English](README.md) | [Chinese](README_CN.md)

An unofficial headless capture client and installable Codex Skill for the ALIENTEK ATK-Logic DL16 family of logic analyzers.

## Features

- Discovers and operates USB devices with VID:PID `1a86:ffcc` through PyUSB
- Supports immediate capture and low-level, high-level, rising-edge, falling-edge, and both-edge triggers
- Supports 16 digital channels and the commonly available sample rates
- Exports per-channel packed binary samples, JSON metadata, VCD waveforms, and raw USB data
- Includes 10 Mbps FlexRay frame decoding, CRC validation, and waveform timing reports
- Keeps protocol decoding extensible so additional decoders can be implemented for user-requested protocols

Only Stream mode is exported safely. Buffer mode is deliberately rejected because ring-buffer trigger offsets are not yet handled completely.

## Validated Hardware

The following tests were completed on an ATK-Logic DL16-family device on 2026-08-14:

- MCU discovery and version query
- 100 MHz, two-channel, 10 ms immediate capture
- 100 MHz, one-channel, 2 ms low-level triggered capture
- Consistency checks for JSON, VCD, packed-channel, and raw USB outputs

Other hardware and firmware revisions may behave differently. When reporting an issue, include the command, complete error output, and device version.

## Installation

Python 3, PyUSB, and a system libusb installation are required.

```bash
git clone https://github.com/dynm/atklogic-headless.git
cd atklogic-headless
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

Install libusb with Homebrew on macOS:

```bash
brew install libusb
```

Install the runtime library on Debian or Ubuntu:

```bash
sudo apt install libusb-1.0-0
```

To install the repository as a personal Codex Skill:

```bash
git clone https://github.com/dynm/atklogic-headless.git ~/.codex/skills/atklogic-headless
```

## Usage

Close the ATK-Logic GUI or any other program that may hold the USB interface, then probe the analyzer:

```bash
python3 scripts/atk_logic_headless.py probe
```

Capture USB CH0 and CH1 immediately:

```bash
python3 scripts/atk_logic_headless.py capture \
  --output captures/smoke \
  --channels 0,1 \
  --rate 100M \
  --duration-ms 10 \
  --threshold 1.6 \
  --instant
```

Capture on a falling-edge trigger:

```bash
python3 scripts/atk_logic_headless.py capture \
  --output captures/triggered \
  --channels 0,1 \
  --rate 100M \
  --duration-ms 100 \
  --trigger-channel 0 \
  --trigger falling \
  --trigger-position 10
```

Channel IDs are zero-based: USB `CH0` is front-panel `CH1`. Stream mode has the following limits:

- 1–3 channels: up to 100 MHz
- 4–6 channels: up to 50 MHz
- 7–16 channels: up to 20 MHz

List all capture options with:

```bash
python3 scripts/atk_logic_headless.py capture --help
```

## Output Formats

For an output prefix named `PREFIX`, the client writes:

- `PREFIX.json`: capture configuration, edge summaries, and decode results
- `PREFIX.vcd`: a waveform that can be opened with GTKWave or another VCD viewer
- `PREFIX.chN.bin`: LSB-first packed samples for channel N
- `PREFIX.usb.bin`: raw USB data for troubleshooting

In a packed channel file, sample `N` is bit `N & 7` of byte `N // 8`.

## FlexRay Decoding

```bash
python3 scripts/atk_logic_headless.py capture \
  --output captures/flexray \
  --channels 0,4,5 \
  --rate 100M \
  --duration-ms 100 \
  --instant \
  --flexray-channel 4 \
  --flexray-channel-type A
```

## Adding Protocol Decoders

FlexRay is currently built in. Additional protocol decoders can be added when users provide a public specification or enough protocol details and representative samples to validate the implementation.

New decoders should remain isolated from the USB capture path, use opt-in CLI arguments, emit structured records into the JSON metadata, and include tests for valid, invalid, truncated, and idle-only input. When this repository is installed as a Codex Skill, the bundled `SKILL.md` directs Codex to implement and validate a requested decoder instead of treating unsupported protocols as already available.

## Source and License

The USB protocol implementation is adapted from the official public ALIENTEK repository, [`alientek-openedv/atk-logic`](https://github.com/alientek-openedv/atk-logic), at baseline commit:

```text
0dff562d24436def2bec3791684f1911997b9e35
```

The primary public reference files are:

- `pv/usb/usb_base.cpp`
- `pv/usb/usb_control.cpp`
- `pv/static/util.cpp`
- `pv/controller/session_controller.cpp`
- `pv/data/session.cpp`

Compatibility differences between device revisions are determined through A/B tests on hardware owned by the maintainers. This repository does not include vendor applications, firmware, capture samples, or other non-public resources.

The upstream project is licensed under GPL-3.0-or-later. This Python port and its extensions are likewise distributed as a whole under [GPL-3.0-or-later](LICENSE). The relevant modification date is 2026-08-14.

This is an unofficial community project and is not affiliated with or endorsed by ALIENTEK. ATK-Logic and any other names or trademarks belong to their respective owners.

## Safety

- Ensure the analyzer and target share ground before connecting signal probes.
- Select a threshold appropriate for the target logic level.
- Raw USB data and waveforms may contain information from the system under test; inspect them before attaching them to a public issue.
- Firmware updates, PWM control, and other device-management write operations are outside this tool's scope.
