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

The main firmware divides time-sensitive work across the RP2350:

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

Each diagnostic has its own PlatformIO environment, so no source file needs to
be edited. The default `pico` environment always builds normal firmware. Normal
motion, homing, and calibration are unavailable in either diagnostic image.

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

### Select, build, and flash a mode

Connect the controller over USB, then run from the repository root:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --environment encoder_wiggle_test --target upload
```

That command builds and flashes the `ENCODER_WIGGLE_TEST` image. To build and
flash the standalone `MOTOR_STEP_TEST` image instead, run:

```bash
~/.platformio/penv/bin/pio run --environment motor_step_test --target upload
```

In the PlatformIO VS Code extension, the equivalent workflow is to select
`encoder_wiggle_test` or `motor_step_test` in the environment selector and use
the Upload action. Because the modes are separate environments, they cannot be
accidentally enabled together.

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

### Normal homing and feedback handover

The default `pico` environment enables `HOMING_BUMPLESS_SERVO_RESTART` and
`HOMING_RESTART_GUARD`, but no diagnostic pauses, pulse cutoffs, phase traces,
or calibration overrides. Feedback continues normally after Home.
On restart, velocity history uses a fresh encoder reading and the velocity
PID is seeded with the held field's offset from the calibrated reference.
This avoids replacing the held field with a different one in a single update;
the existing +/-81 electrical-degree correction limit is unchanged.

`G28` now defaults to `HOMING_BACKOFF_ANGLE_DEG=3.6` in `src/hw_config.h`.
This is a commanded field rotation expressed as mechanical motor degrees,
not a guarantee of actual rotor movement. The previous 1.8-degree command
could leave these motors outside the measured calibration range. Explicit
values such as `G28 B1.8` still override the default. Calibration keeps its
original separate backoff and measurement origin, so this change does not
rewrite or shift saved tables.

The restart preflight rejects out-of-calibration encoder readings, non-finite
phase comparisons, or field mismatches exceeding 75 electrical degrees.
A refusal pauses that axis's feedback, clears its homed status, and makes
Home/enable/calibration completion report an error rather than silently
claiming a successful handover. The held field remains powered; use `M18`
to disable outputs. This check is not a physical travel-limit guarantee,
and existing travel restrictions and PWM/current limits are unchanged.

### Homing transition test

The optional `homing_transition_test` environment runs normal robot firmware
with `HOMING_TRANSITION_TEST` enabled. It does not move automatically. Build and
flash it from `firmware/MotionControllerRP` with:

```bash
~/.platformio/penv/bin/pio run --environment homing_transition_test --target upload
```

When ready to observe all three axes, send `G28 A B C`. Each axis backs off,
waits one second, restores its normal PWM amplitude, and waits another second.
After all three finish, feedback field updates stay paused for two more seconds
before position control resumes. Motors remain powered during these pauses;
the test does not increase the configured amplitudes or backoff distance.

`HOME TEST` messages identify each transition and report encoder raw-count
changes during amplitude restoration. Before servo restart, `enc_in_lut=0`
indicates a reading outside the calibration table's input range.
`calibrated_delta_deg` is the shortest electrical-angle difference between the
held field and the calibrated reference field, **not** rotor displacement or
the first applied servo field (which also includes the PID correction).
`error_deg` compares the joint target with the measured motor angle before
restart. Added pauses can change settling and scheduling, so this mode helps
locate the snap but cannot by itself rule out timing-dependent faults.

These pauses and logs are absent from the default `pico` environment.

`homing_phase_trace_test` adds phase-reference snapshots to the open-loop test
described below and never restarts feedback after Home. Build/upload it with
`pio run -e homing_phase_trace_test -t upload`. Each axis's `HOME TEST` backoff
marker identifies the following `HOME PHASE` rows: `endstop`, `before_backoff`,
`after_backoff`, `before_restore`, and `after_restore`. The end-stop snapshot is
captured without logging while other axes are still homing, then printed during
finalization. `raw` is the absolute encoder count, `in_lut` indicates whether it
is within saved calibration, and `pos_deg` is the lookup-table motor position.
`field_deg` and `reference_deg` are the held and calibrated electrical angles;
`mismatch_deg` is their shortest difference, not mechanical displacement.
Outside calibration, the lookup table returns its endpoint; phase comparisons
there are not reliable rotor-phase measurements. This diagnostic preserves the
existing homing speed, backoff distance, PWM limits, pauses, and saved tables.
Motors stay powered after Home until `M18`; do not jog or calibrate in this mode.

`calibration_reference_test` extends that diagnostic by also skipping servo
restart after `M56`. This permits an isolated open-loop calibration measurement
without handing control back to potentially incompatible field references on
any axis. There is no automatic movement on boot. `M56 J1 P` homes and sweeps
only motor 2 through the configured `CALIBRATION_RANGE` (currently 83 motor
degrees), returns to its starting field, prints samples, and replaces that
axis's lookup tables **in RAM only**. Omit `S`: adding it saves to flash.
Subsequent phase-trace Home commands can check the fresh reference, still
without restarting feedback. Rebooting reloads the original saved tables.
PWM limits, sweep range/speed, and calibration fitting remain unchanged.
Motors remain powered after measurement until `M18`. Do not use this mode for
normal jogging; restore normal firmware after investigation.

For a comparison that isolates backoff and amplitude restoration from servo
restart, flash `homing_open_loop_test` instead. It uses the same sequence but
skips servo restart after `G28`. Motors remain powered at the final homing field
until an explicit `M18`; do not jog, calibrate, or use this image for normal
operation. Observations before `M18` exclude motor release as a source of the
snap. Both diagnostic environments preserve the configured PWM limits.

`homing_servo_pulse_test` uses the same staged homing sequence, then allows
feedback field updates for up to 50 ms per axis, or until raw encoder counts
indicate at least 2 degrees of rotor excursion from that axis's first sample.
Core 1 enforces this cutoff without serial logging. Afterward the last field
stays powered; there is no automatic `M18`. The cutoff freezes field commands,
not physical motion: it cannot prevent the initial snap, guarantee a mechanical
travel limit, or protect against an encoder that fails to report movement.
Do not jog or calibrate in this diagnostic; restore `pico` for normal operation.

`HOME PULSE` reports the first actual sample's target/error and velocity history.
`reference_delta_deg` is the calibrated field change from the held homing field;
`pid_deg` is the controller's correction; `applied_delta_deg` is the shortest
electrical-angle change actually applied on the first update. Field angles are
electrical degrees, not rotor degrees. `max_rotor_excursion_deg` uses the nominal
encoder geometry, not the clamped lookup table. A missing capture or incomplete
cutoff is reported explicitly, with field updates also paused by core 0
50 ms after the configured pulse duration (100 ms total for the default pulse).
These logs and cutoffs are absent from the default firmware.

`homing_bumpless_test` adds `HOMING_BUMPLESS_SERVO_RESTART` to that same bounded
pulse test. On servo restart, it seeds the velocity PID integral with the
shortest electrical-angle offset from the calibrated reference to the held
homing field, instead of zero. Existing integral/output limits still apply;
offsets beyond those limits cannot be retained exactly. Compare
`applied_delta_deg` and the visible snap against `homing_servo_pulse_test`.
This handover flag itself does not change PWM limits, backoff, or calibration
tables, and does not correct out-of-range encoder lookup-table inputs.
It is enabled in normal `pico` firmware and these comparison environments.
Builds without `HOMING_BUMPLESS_SERVO_RESTART` retain zero-initialized PID resets.

`homing_bumpless_hold_test` extends only that fixed handover's pulse duration
to one second with `HOMING_SERVO_PULSE_US=1000000`. The same raw-encoder-based
2-degree excursion cutoff remains active. This checks for delayed drift or
snapping without leaving position control running indefinitely. Core 0's
fallback freeze occurs after 1.05 seconds. Motors remain powered afterward,
and the same diagnostic precautions and encoder limitations apply.

`calibration_bumpless_test` combines this one-second pulse with phase tracing
and open-loop calibration completion. `M56 J1 P` without `S` refreshes motor 2's
calibration in RAM without restarting feedback. After separately preparing the
observer, `G28 B` homes only motor 2 and attempts the bounded feedback handover.
The `HOMING_RESTART_GUARD` flag refuses feedback restart if the encoder input
is outside calibration, the phase comparison is non-finite, or the mismatch
exceeds 75 electrical degrees (margin below the existing 81-degree correction
limit). A refused restart leaves the held field powered and reports `HOME
GUARD`; Home reports an error and that axis's homed status is cleared. It is
not a successful feedback test. Only homed/calibrated joints are
eligible, so after boot and calibration of motor 2 alone, the other axes do not
resume feedback. The preflight and excursion cutoff are not physical safety
limits and cannot guarantee against snapping or an invalid encoder. Do not jog
in this diagnostic. Reflashing/rebooting loses RAM-only calibration; saved
tables remain unchanged unless a calibration command explicitly includes `S`.

`calibration_bumpless_long_hold_test` uses the same guarded handover with a
ten-second observation window (`HOMING_SERVO_PULSE_US=10000000`). The first
snapshot, roughly one-second snapshots, and the cutoff snapshot fit in the
same fixed-size buffer. The 2-degree excursion cutoff remains active, and
core 0 freezes updates after 10.05 seconds if necessary. This environment
does not move automatically and is not intended for normal jogging.

`homing_bumpless_immediate_test` retains that ten-second guarded observation
and open-loop calibration completion but omits the transition-test pauses.
It therefore checks handover immediately after the normal backoff and
amplitude ramp. No motion occurs automatically; use explicit commands such
as `G28 A3.6` to compare the already-tested backoff at normal timing. The
diagnostic still pauses feedback at the end of its bounded window, so this
is not normal operating firmware.

Unpowered axes are excluded from feedback restart in every environment.
At zero driver amplitude, encoder tracking continues but both PID controllers
and velocity history are reset, and the field command is not advanced. This
prevents a disabled motor from accumulating an unseen correction that could
be applied at its next power-up. `M18` pauses feedback before ramping all
channels to zero and putting their shared enable line in standby. `M17`
reasserts that enable line, ramps the channels, and resumes feedback only
for homed, calibrated, powered axes. These changes do not raise current limits.
Each diagnostic Home clears previous pulse records, so an unpowered or
guard-refused axis reports `captured=0` rather than an old test's results.

Host-only regression checks (no device access) can be run with
`bash firmware/MotionControllerRP/test/run_host_servo_restart.sh` from the
repository root. They compile the actual servo update/restart methods and
PID implementation against mock encoder/driver/time objects, checking
zero-amplitude field retention, fresh velocity history, bounded PID preload,
phase wrapping, duration/excursion cutoffs, diagnostic snapshot timing,
preflight refusal, and readiness invalidation.
These tests require `g++`; they do not verify real encoder reliability or
physical travel limits.

Pulse diagnostics also capture `HOME TRACE` snapshots at roughly 100 ms
intervals (one second in the ten-second environment) and at cutoff.
`a` is the one-based motor position, `ms` is elapsed
time since its first servo update, `rotor_d` is signed raw-encoder-derived rotor
movement, and `err` is target minus lookup-table position. `pid` is the computed
controller output, and `field_d` is the shortest applied field change relative
to the held homing field. All angular values are degrees; `pid` and `field_d`
are electrical degrees, not rotor degrees. At cutoff, the controller output is
computed but no further field write is allowed; `field_d` still reports the
last applied field. Snapshots stay in memory during the pulse and are printed
afterward so serial output does not interrupt active servo control.

### Restore normal firmware

Build and upload the default `pico` environment again:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --environment pico --target upload
```

Normal robot initialization, motion, homing, and calibration will be restored.
