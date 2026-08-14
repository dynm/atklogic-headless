---
name: atklogic-headless
description: Operate an ALIENTEK ATK-Logic DL16-family logic analyzer without its GUI by probing USB devices, capturing digital channels with immediate or edge triggers, exporting packed samples/JSON/VCD, decoding supported protocols such as FlexRay, and extending the bundled client with new protocol decoders when requested. Use for ATK-Logic or DL16 headless capture, scripted logic-analyzer acquisition, ATK USB troubleshooting, VCD generation, protocol decoding, or implementing and validating a decoder for a user-specified digital protocol.
---

# ATKLogic Headless

Use the bundled `scripts/atk_logic_headless.py` client. It talks directly to USB VID:PID `1a86:ffcc` and requires Python 3, PyUSB, and a working libusb backend.

## Prepare

1. Locate the directory containing this `SKILL.md` and refer to it as `ATKLOGIC_SKILL_DIR` in commands. Do not assume the current working directory is the skill directory.
2. Close the vendor ATK-Logic application so it does not hold the USB interface.
3. Ensure the analyzer and target share ground before capturing hardware signals.
4. Inspect both help surfaces before composing unfamiliar options:

```bash
python3 "$ATKLOGIC_SKILL_DIR/scripts/atk_logic_headless.py" --help
python3 "$ATKLOGIC_SKILL_DIR/scripts/atk_logic_headless.py" capture --help
```

If import fails, install PyUSB in the user's preferred Python environment. On macOS or Linux, also ensure libusb is installed. Ask before installing packages or changing USB permissions.

## Probe First

Run a bounded probe before every new hardware session:

```bash
python3 "$ATKLOGIC_SKILL_DIR/scripts/atk_logic_headless.py" probe
```

With multiple analyzers, put the global selector before the subcommand:

```bash
python3 "$ATKLOGIC_SKILL_DIR/scripts/atk_logic_headless.py" --serial SERIAL probe
```

Do not capture if the probe cannot claim the interface. Close competing applications, reconnect the analyzer, and probe again.

## Capture

Choose an output prefix in the user's workspace. Channel IDs are zero-based: USB `CH0` is front-panel `CH1`.

Use immediate capture when no event trigger is required:

```bash
python3 "$ATKLOGIC_SKILL_DIR/scripts/atk_logic_headless.py" capture \
  --output captures/atk-smoke \
  --channels 0,1 \
  --rate 100M \
  --duration-ms 10 \
  --threshold 1.6 \
  --instant
```

Use an edge or level trigger when alignment matters:

```bash
python3 "$ATKLOGIC_SKILL_DIR/scripts/atk_logic_headless.py" capture \
  --output captures/atk-triggered \
  --channels 0,1 \
  --rate 100M \
  --duration-ms 100 \
  --trigger-channel 0 \
  --trigger falling \
  --trigger-position 10 \
  --timeout 15
```

Respect Stream limits: at most 100 MHz for 1–3 channels, 50 MHz for 4–6 channels, and 20 MHz for 7–16 channels. Do not use `--buffer`; the client intentionally rejects it because ring-buffer trigger offsets are not exported safely.

## Decode FlexRay

Include each source channel in `--channels`, then select the primary channel with `--flexray-channel`. Use the physical bus channel (`A` or `B`) that matches the capture so header CRC evaluation is meaningful.

```bash
python3 "$ATKLOGIC_SKILL_DIR/scripts/atk_logic_headless.py" capture \
  --output captures/flexray \
  --channels 4,5 \
  --rate 100M \
  --duration-ms 100 \
  --instant \
  --flexray-channel 4 \
  --opposite-flexray-channel 5 \
  --flexray-channel-type A
```

## Add a Requested Protocol Decoder

When the user requests an unsupported protocol, extend the bundled client instead of claiming it is already supported:

1. Establish the protocol from a public specification, public datasheet, or user-provided description. Record bitrate, polarity, framing, bit order, checksums, and timing tolerances.
2. Ask for a representative capture only when the protocol or signal mapping cannot be determined safely from available context. Preserve raw channel files as fixtures when the user authorizes their inclusion.
3. Implement an isolated `_decode_<protocol>` function that accepts packed samples, sample count, sample rate, and explicit protocol settings. Return structured records rather than printing inside the decoder.
4. Add opt-in CLI arguments such as `--<protocol>-channel`; require decoder channels to appear in `--channels`. Keep ordinary capture behavior unchanged when decoder options are absent.
5. Print a concise decode summary and store complete decoded records under a protocol-specific key in `PREFIX.json`.
6. Test known-valid frames, checksum failures, truncated frames, idle-only input, polarity, and sample-grid tolerance. Run syntax and CLI help checks plus a representative hardware or fixture test.
7. Update both `README.md` and `README_CN.md`, CLI help, and this Skill when support is complete. Cite only public sources and documented A/B hardware tests; do not include vendor applications, firmware, captures without permission, or other non-public resources.

Keep protocol-specific policy out of the generic USB capture path. Prefer a small decoder plus focused tests over special-case checks tied to one application or frame ID.

## Inspect Results

For output prefix `PREFIX`, expect:

- `PREFIX.json`: capture settings, edge summaries, and decoded protocol records.
- `PREFIX.vcd`: waveform for GTKWave or another VCD viewer; omit only with `--no-vcd`.
- `PREFIX.chN.bin`: packed samples, LSB-first; sample `N` is bit `N & 7` of byte `N // 8`.
- `PREFIX.usb.bin`: raw USB traffic for protocol debugging.

Treat exit code `0` as success and `1` as a setup, capture, protocol, or argument failure. Report exact output paths, capture configuration, decoded-record counts, checksum status, and any incomplete frames.

## Troubleshoot

- `ATK-Logic ... not found`: check USB attachment, VID:PID, cable, and `--serial`.
- Interface claim or busy error: close ATK-Logic GUI and other clients, reconnect, then probe.
- Incomplete capture: confirm the trigger fired, then reduce rate, channel count, or duration and verify Stream limits.
- Stream overflow: reduce sample rate or enabled channels.
- No useful edges: confirm zero-based channel mapping, shared ground, probe location, and threshold voltage.
- Decoder failure: confirm bitrate, polarity, channel mapping, sampling ratio, frame boundaries, and checksum parameters against the public specification and raw waveform.
