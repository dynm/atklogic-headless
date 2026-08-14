---
name: atklogic-headless
description: Operate an ALIENTEK ATK-Logic DL16-family logic analyzer without its GUI by probing USB devices, capturing digital channels with immediate or edge triggers, exporting packed samples/JSON/VCD, decoding FlexRay frames, and verifying the pico-flexray slot10 bench timing. Use for ATK-Logic or DL16 headless capture, scripted logic-analyzer acquisition, ATK USB troubleshooting, VCD generation, FlexRay waveform decoding, or slot10 with11/without11 verification.
---

# ATKLogic Headless

Use the bundled `scripts/atk_logic_headless.py` client. It talks directly to USB VID:PID `1a86:ffcc` and requires only Python 3, PyUSB, and a working libusb backend.

## Prepare

1. Locate the directory containing this `SKILL.md` and refer to it as `ATKLOGIC_SKILL_DIR` in commands. Do not assume the current working directory is the skill directory.
2. Close the vendor ATK-Logic application so it does not hold the USB interface.
3. Ensure the analyzer and target share ground before capturing hardware signals.
4. Run the client help before composing unfamiliar options:

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

Do not start a capture if the probe cannot claim the interface. Close competing applications, reconnect the analyzer, and probe again. Do not repeatedly retry protocol errors without diagnosing the cause.

## Capture

Choose an output prefix in the user's workspace. Channel IDs are zero-based: USB `CH0` is the front-panel `CH1`, USB `CH1` is panel `CH2`, and so on.

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

Use a simple edge or level trigger when alignment matters:

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

Select only supported sample rates shown by `capture --help`. Respect DL16 Plus Stream limits: at most 100 MHz for 1–3 channels, 50 MHz for 4–6 channels, and 20 MHz for 7–16 channels. Do not use `--buffer`; the client intentionally rejects it because ring-buffer trigger offsets are not exported safely.

## Decode and Verify FlexRay

Include the source channel in `--channels`, then select it with `--flexray-channel`. Use the physical bus channel (`A` or `B`) that matches the capture so header CRC evaluation is meaningful.

```bash
python3 "$ATKLOGIC_SKILL_DIR/scripts/atk_logic_headless.py" capture \
  --output captures/flexray-slot10 \
  --channels 0,4,5 \
  --rate 100M \
  --duration-ms 100 \
  --instant \
  --flexray-channel 4 \
  --flexray-channel-type A \
  --verify-slot10 with11 \
  --opposite-txen-channel 5 \
  --expect-fid10-payload 0a0a \
  --expect-fid10-nfi 1
```

For slot10 verification, USB `CH0` is reserved for active-low target TXEN and must not also be the decoded FlexRay channel. Use `--verify-slot10 with11` or `without11` to match the bench schedule. Add `--opposite-flexray-channel` only when that channel is also captured.

## Inspect Results

For output prefix `PREFIX`, expect:

- `PREFIX.json`: capture settings, edge summaries, decoded frames, and verification results.
- `PREFIX.vcd`: waveform for GTKWave or another VCD viewer; omit only with `--no-vcd`.
- `PREFIX.chN.bin`: packed channel samples, LSB-first; sample `N` is bit `N & 7` of byte `N // 8`.
- `PREFIX.usb.bin`: raw USB traffic for protocol debugging.

Treat exit code `0` as success, `1` as setup/capture/protocol failure, and `2` as a completed capture whose slot10 verification failed. Report the exact output paths, capture configuration, decoded-frame summary, and any failed timing checks. Preserve failed captures because their JSON, VCD, and raw USB files are diagnostic evidence.

## Troubleshoot

- `ATK-Logic ... not found`: check USB attachment, VID:PID, cable, and `--serial`.
- Interface claim or busy error: close ATK-Logic GUI and other clients, reconnect, then probe.
- Incomplete capture: for triggered mode, confirm the trigger fired; otherwise reduce rate, channel count, or duration and verify Stream limits.
- Stream overflow: reduce sample rate or enabled channels.
- No useful edges: confirm zero-based channel mapping, shared ground, probe location, and threshold voltage.
- FlexRay CRC/timing failure: confirm channel A/B selection, 10 Mbps bitrate, signal polarity, sample rate, and correct slot10 schedule before changing tolerances.
