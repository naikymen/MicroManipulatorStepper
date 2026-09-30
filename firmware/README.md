# Motion Controller Firmware

This directory contains the PlatformIO project for the three-axis motion
controller. The normal firmware targets a Raspberry Pi Pico 2 (RP2350) using
the Arduino-Pico framework and runs the system clock at 250 MHz.

## Hardware handled by the firmware

The current three-axis configuration controls:

- three MT6835 magnetic encoders on a shared SPI bus with separate chip-select
  lines;
- three two-phase stepper motors through TB6612 drivers, using sinusoidal field
  commutation and a shared hardware standby pin; and
- two timing-aware PWM tool outputs.

Pin assignments, motor pole-pair counts, encoder geometry, current limits,
homing behavior, calibration settings, and servo gains are defined in
[`MotionControllerRP/src/hw_config.h`](MotionControllerRP/src/hw_config.h).
Review these values before flashing a controller with different motors,
mechanics, supply voltage, or wiring.

## Runtime architecture

Normal firmware divides time-sensitive work across the RP2350:

- Core 0 receives USB serial commands and feeds the path planner.
- A 500 microsecond repeating timer advances the motion controller and publishes
  joint targets.
- Core 1 updates the encoder-based joint servo controllers.
- Hardware spinlocks protect targets and joint state shared between those
  execution contexts.

At startup the firmware mounts LittleFS, initializes the encoders and motor
drivers, loads any saved joint calibration tables, configures the tool outputs,
and then starts the second-core servo loop. Calibration tables are stored as
`jointN_enc_to_pos_lut.dat` and `jointN_pos_to_field_lut.dat`; an `M56` command
only persists new tables when its save option is supplied.

## Source layout

| Path | Responsibility |
| --- | --- |
| `src/main.cpp` | Clock, serial, filesystem, core startup, and optional diagnostic entry points. |
| `src/robot.cpp` | Robot initialization, command dispatch, planning coordination, and shared state. |
| `src/command_parser/` | Incremental G-code parsing. |
| `src/motion_control/` | Path buffering, interpolation, and joint-target generation. |
| `src/kinematic_models/` | Cartesian-pose to joint-space conversion and machine geometry. |
| `src/robot_joint/` | Per-axis ownership, calibration persistence, and homing/calibration orchestration. |
| `src/servo_control/` | Position/velocity control, homing, and actuator calibration. |
| `src/hardware/` | MT6835/MT6701 encoder and TB6612 motor-driver interfaces. |
| `src/robot_tool/` | Synchronized PWM tool outputs. |
| `src/utilities/` | Logging, lookup tables, math, timing, and buffering helpers. |

Machine dimensions live in
[`kinematic_model_delta3d.cpp`](MotionControllerRP/src/kinematic_models/kinematic_model_delta3d.cpp),
the reported firmware version lives in
[`version.h`](MotionControllerRP/src/version.h), and the target, dependencies,
serial port, and build flags live in
[`platformio.ini`](MotionControllerRP/platformio.ini).

## Command-line build workflow

From the repository root, build the normal firmware with:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run
```

Upload it to the configured controller with:

```bash
~/.platformio/penv/bin/pio run --target upload
```

PlatformIO build products are generated under `MotionControllerRP/.pio/` and
are not source-controlled. The USB serial interface runs at 921600 baud. For
host-side control and calibration examples, see the
[Python API documentation](../software/PythonAPI/README.md).

## Optional firmware debug modes

The RP2350 firmware includes two optional standalone diagnostic modes:

- `ENCODER_WIGGLE_TEST` continuously reports all three encoders so intermittent
  power, chip-select, and SPI connections can be found by moving one wire at a
  time. Motor outputs remain disabled.
- `MOTOR_STEP_TEST` drives one motor at a time without using the encoders,
  kinematic model, homing, or closed-loop controller. It helps isolate motor,
  winding, driver, socket, and output-wiring faults.

Both modes are disabled by default. Normal motion, homing, and calibration are
unavailable while either diagnostic is enabled. Enable only one mode at a
time.

### Prerequisites

Install PlatformIO Core or the PlatformIO VS Code extension. The commands below
assume PlatformIO's default Linux installation path. If `pio` is already on
your `PATH`, it can be used instead of `~/.platformio/penv/bin/pio`.

The configured upload and monitor port is the controller's stable device path:

```text
/dev/serial/by-id/usb-2e8a_Micro-Manipulator_81A5365DC37CD552-if00
```

If using another controller, update `upload_port` and `monitor_port` in
`MotionControllerRP/platformio.ini`.

### Enable a mode

Open `MotionControllerRP/src/hw_config.h`. To diagnose encoder connections,
change:

```cpp
// #define ENCODER_WIGGLE_TEST
```

to:

```cpp
#define ENCODER_WIGGLE_TEST
```

Alternatively, to test motor outputs, uncomment:

```cpp
#define MOTOR_STEP_TEST
```

Do not uncomment both definitions simultaneously.

### Build and flash

Connect the controller over USB, then run from the repository root:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --target upload
```

A successful upload ends with `SUCCESS`. Open the live serial output afterward:

```bash
~/.platformio/penv/bin/pio device monitor
```

The monitor settings in `platformio.ini` select the persistent device path,
921600 baud, and the direct output filter. Press `Ctrl+C` to close it.

### Encoder wiring mode

The encoder diagnostic begins reporting automatically. Keep the mechanism
stationary and move only one wire or connector at a time. A healthy channel
normally reports small `d` and `max` changes, `st=0[----]`, `bad=0`, `crc+=0`,
and `id=OK`.

The motors remain in standby in this mode. See the
[complete output-field reference](../documentation/setup_guide/encoder_wiggle_test.md)
for interpretation and troubleshooting guidance.

### Motor step mode

> **Warning:** The TB6612 does not measure winding current. This mode applies a
> low PWM-duty cap, not closed-loop current regulation. Use a current-limited
> supply, keep clear of the mechanism, and be ready to remove motor power.

The motor test starts with all outputs disabled. Enter one command in the serial
monitor and press Enter:

| Command | Action |
| --- | --- |
| `1`, `2`, `3` | Move the selected motor eight full steps forward and backward using 1/16 microsteps. |
| `a` | Test all three motors sequentially. |
| `p` | Test motor 1 holding torque on phase A and then phase B. |
| `b`, `c`, `d`, `e` | Hold motor 1 at B+, A+, A-, or B- for an eight-second voltage measurement. |
| `x` | Set all amplitudes to zero and disable the shared driver enable. |

Each move ramps the output up and down and finishes with the drivers disabled.
See the [motor-test reference](../documentation/setup_guide/motor_step_test.md)
for the configured duty limits and additional safety notes.

### Restore normal firmware

Comment out the enabled definition in `MotionControllerRP/src/hw_config.h` so
the beginning of the file contains:

```cpp
// #define ENCODER_WIGGLE_TEST
// #define MOTOR_STEP_TEST
```

Then build and upload again:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --target upload
```

Normal robot initialization, motion, homing, and calibration will be restored.
