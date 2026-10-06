# Encoder Wiring Wiggle Test

The optional encoder wiggle-test firmware helps identify intermittent MT6835
power and SPI connections. It continuously checks all three encoders while you
move one wire or connector at a time.

In this mode the firmware holds the motor-driver enable pin low, does not start
the robot, motion planner, or servo loop, and does not write calibration data.
It uses a 1 MHz SPI clock to reduce signal-integrity effects while diagnosing
physical connections.

To check communication at the normal robot's **8 MHz** SPI clock instead, use
the optional `encoder_spi8m_test` environment. It has the same disabled motors,
channel/status/CRC/ID checks and output; only the SPI clock differs. Passing the
1 MHz test alone does not establish reliable 8 MHz communication. Conversely,
this standalone test does not reproduce the normal servo loop's polling rate,
concurrent timing or powered-motor interference.

## Enable and Flash

Build and flash the dedicated diagnostic environment from the firmware
directory. No source-file edit is required:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --environment encoder_wiggle_test --target upload
```

For the operating-clock comparison, replace `encoder_wiggle_test` with
`encoder_spi8m_test` in that command. The banner prints the actual selected
clock. Neither environment changes saved calibrations or runs motor motion.

Open the live serial monitor:

```bash
~/.platformio/penv/bin/pio device monitor \
  --port /dev/ttyACM0 \
  --baud 921600 \
  --filter direct
```

The serial port may have a different name after reconnecting the controller.
Press `Ctrl+C` to close the monitor.

Keep the rotor stationary and wiggle only one wire or connector at a time. This
makes position jumps distinguishable from real rotor movement.

## Output

The firmware prints one line every 100 ms:

```text
t=12345 | E1 raw=100240 d=-3 max=18 st=0[----] bad=0 crc+=0 id=OK
```

| Field | Meaning |
|---|---|
| `t` | Milliseconds since boot. |
| `E1`–`E3` | Encoder channel. |
| `raw` | Accumulated encoder position in raw counts. One encoder-field revolution is 2,097,152 counts. |
| `d` | Change from the immediately preceding sample. |
| `max` | Largest absolute change during the latest 100 ms reporting window. |
| `st` | Latest hexadecimal status and readable flags. `0[----]` is healthy. |
| `bad` | Samples with at least one status/error flag during the reporting window. |
| `crc+` | New CRC failures during the reporting window. |
| `id` | Register write/read communication check, updated once per second. |

The status flags are:

- `O`: overspeed
- `W`: weak magnetic field
- `U`: encoder undervoltage
- `C`: CRC/data-integrity failure

A healthy stationary channel normally has small `d` and `max` values,
`st=0[----]`, `bad=0`, `crc+=0`, and `id=OK`. An unplugged or intermittent
channel may show large position jumps, status flags, increasing error counts,
or `id=FAIL`.

Some MT6835 modules do not provide a reliable CRC byte. Treat `crc+` as one
signal among several and also inspect `raw`, `max`, `st`, and `id`.

## Restore Normal Firmware

Build and flash the normal `pico` environment:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --environment pico --target upload
```

Calibration and normal motion commands are not available while the diagnostic
image is running.
