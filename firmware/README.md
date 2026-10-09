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

## Cartesian motion limits

Normal `G0`/`G1` moves and realtime `G24` targets require all joints to be homed
and calibrated, even when the legacy `JOINT_READY_OVERRIDE` define is present.
Each joint's usable motor-angle interval is the intersection of its encoder-LUT
output range, field-LUT input range, and `0..CALIBRATION_RANGE`. A successful
normal Home records the final joint position, the encoder-measured clearance
from the physical stop, and the direction away from that stop. Normal motion
may use `HOMING_USABLE_CLEARANCE_FRACTION` (default 75%) of that specific
clearance; the remaining 25% is kept as a physical reserve. The opposite,
unmeasured end retains `JOINT_OPPOSITE_TRAVEL_MARGIN_DEG` (default 0.5 degrees).
Calibration invalidates this Home reference, so a normal measured Home is
required before Cartesian motion resumes.

The firmware rejects the whole command if inverse kinematics fails, produces
non-finite angles, or demands an out-of-range joint. Rejection does not change
the accepted Cartesian target, feedrate, or any joint target. `G0`/`G1` paths
are preflighted at no more than 0.01 mm translation / 0.001 rad rotation spacing,
including both endpoints; paths requiring more than 4096 intervals are rejected.
Sampling is not a mathematical proof of continuous Cartesian reachability:
every generated joint segment is checked again before execution. Since these
segments use linear joint interpolation, their checked endpoints bound the
entire commanded segment. The interrupt also checks targets before publication.
An unexpected execution-time violation holds all axes at the last published
targets and latches a fault; Home clears pending paths and permits recovery.
`JOINT_LIMIT_NUMERIC_TOLERANCE_DEG` (default 0.0001 degrees) applies only to
boundary comparisons. It does not clamp a target, shorten a path, or change the
requested Cartesian endpoint.

`G24` returns `busy` while a planned path is queued or still executing, preventing
the two command sources from overwriting each other's targets. Homing,
calibration and measured-pose synchronization retain their internal procedures
and are not restricted by the normal-motion margin.

`G4` is a planner-ordered dwell, not a motion command. It validates its duration
and retains the joint endpoint already accepted by the planner without invoking
inverse kinematics or joint-limit checks during the wait. It therefore remains
available before Home while `G0`, `G1`, and `G24` retain their fail-closed
readiness and travel checks. Tool outputs keep their existing queued timing in
normal operation. If a motion fault has stopped the planner, only an explicit
zero-valued tool command is applied immediately; a nonzero output is rejected
and not retained for later application after recovery.

These are calibrated **command limits**, not independent physical limit switches
or a tracking/encoder-fault watchdog. Bad calibration, incorrect geometry or a
mechanical stop inside the calibrated interval can still cause a collision.
`M50` reports the last accepted target, not a measured Cartesian position.
`M53` reports completion only after the last active joint segment finishes,
not merely when the planner queues become empty.

Offline regressions (no hardware access):

```bash
bash firmware/MotionControllerRP/test/run_host_motion_limits.sh
bash firmware/MotionControllerRP/test/run_host_servo_restart.sh
```

Current code-only validation covers measured Home, dynamic limit derivation,
the actual motion-command handlers, servo restart, calibration-reference
behavior, and mocked GUI/API rejection handling. The captured post-Home geometry
accepts exact 1 mm inward X, Y, and Z commands. Homed range enforcement still
requires live validation; the current fix has not been uploaded while hardware
is unavailable.

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

The default `pico` firmware finds the mechanical stop from encoder stall,
backs away by 2.5 degrees of encoder-confirmed movement, validates the saved
calibration and 75-electrical-degree phase limit, then starts feedback from one
final measured position. Its first command preserves the field already holding
the motor and removes that temporary difference at a bounded rate while the
encoder position is monitored. It then moves each selected joint smoothly to
the calibrated 7-degree post-Home position so full 1 mm Cartesian jogs have
room in either direction. The complete sequence, failure behavior, and
resulting travel limits are explained in
[How a normal homing move works](../documentation/firmware/homing_flow.md).

Calibration remains a separate operation. Before its measurement sweep, it
establishes its origin only after raw encoder travel confirms 1.5 degrees of
clearance from the stop. Normal Home uses the saved tables but does not modify
them.

Offline regressions (no device access):

```sh
bash firmware/MotionControllerRP/test/run_host_measured_homing.sh
bash firmware/MotionControllerRP/test/run_host_servo_restart.sh
bash firmware/MotionControllerRP/test/run_host_motion_limits.sh
```

### Guarded homing phase trace (normal timing)

`homing_guard_trace_test` inherits the normal `pico` handover and restart guard,
adding only `HOMING_PHASE_TRACE`. It labels each axis and records its encoder
position, held electrical field and calibrated reference at the detected stop,
before/after backoff, and after amplitude restoration. It adds no deliberate
pauses, bounded servo pulses, calibration override or automatic movement. Build
and flash with `pio run -e homing_guard_trace_test -t upload`; send `G28` only
when ready to observe homing. Restore normal firmware with `pio run -e pico -t upload`.

`HOME GUARD ... mismatch_deg` is an **electrical** phase disagreement, not a
mechanical travel angle. An in-range encoder can still have an invalid phase
reference. A refused restart leaves the affected joint unhomed, so Home returns
an error and normal Cartesian moves are blocked. Do not raise the guard threshold
to mask the error: the existing velocity-PID correction is limited to 81
electrical degrees. This trace distinguishes backoff, current restoration and
calibration-reference disagreement before choosing a correction.

`M57` now also reports one-based axis diagnostics: measured motor/target angles,
PWM amplitude, encoder-LUT membership, held/reference electrical field angles
with their wrapped mismatch, measured Home clearance, estimated stop, and the
active dynamic travel interval. These are read-only snapshots; they do not
re-enable an axis or change a calibration. Motion rejections identify the
offending joint, angle, allowed interval and path fraction (0 means the starting
pose failed).

`calibration_guard_reference_test` extends this normal-timing trace with
`CALIBRATION_REFERENCE_TEST`: calibration finishes with feedback still paused,
but subsequent Home retains the normal bounded transition and 75-degree guard.
Flash with `pio run -e calibration_guard_reference_test -t upload`. For motor 3,
`M56 J2 P` homes, sweeps through the configured 83-degree calibration range and
back, and replaces only its RAM tables. **Do not add `S`**: the saved tables
must remain unchanged for this comparison. After inspecting the calibration
result, a separate `G28 C` tests the fresh reference with the same default
backoff and current settings. No commands run automatically on boot. Rebooting
restores the saved tables; do not save the experimental fit or use it for normal
jogging until its homing/restart behavior has been verified.

`homing_backoff_power_test` changes only the amplitude-restoration order: search
still uses `HOMING_CURRENT`, but the previous amplitude is restored at the stop,
with the field held fixed, before the slow backoff. The PWM cap is unchanged.
Only `G28` selects the earlier restoration; `M56` keeps the existing calibration
procedure so its changed measurement cannot confound this timing comparison.
It inherits the RAM-calibration comparison and normal Home guard; it neither
runs commands automatically nor saves tables without `S`. Phase rows therefore
show `before_restore`/`after_restore` before `before_backoff`/`after_backoff`.
Flash with `pio run -e homing_backoff_power_test -t upload`. The flag is absent
from normal firmware; it is an investigation of low-amplitude backoff, not an
established fix. Do not combine it with the staged transition-test flag.

In the motor-2 investigation, early restoration did **not** resolve the restart
failure: two Homes were refused at about 84 and 151 electrical degrees. Two
fresh calibrations also differed by about 102 electrical degrees at matching
encoder counts, despite low individual fitting errors. Do not promote this
timing change or save a fit merely because its RMS error is small. Check the
rotor-to-motor-shaft connection described in the
[setup checklist](../documentation/setup_guide/setup_guide.md), and investigate
encoder/driver faults if that connection is sound. These observations suggest
a shifting mechanical/electrical reference; they do not prove which component
is responsible.

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
This diagnostic handover flag itself does not change PWM limits, backoff, or
calibration tables, and does not correct out-of-range encoder lookup-table
inputs. It is not enabled in normal `pico` firmware; normal firmware uses the
bounded temporary transition described above. Builds without
`HOMING_BUMPLESS_SERVO_RESTART` retain zero-initialized PID resets.

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
preflight refusal, readiness invalidation, and selected-axis calibration with
explicit-only saving and diagnostic feedback-restart suppression.
They also check normal versus optional early amplitude-restoration order using
the actual homing finalization method, including unchanged backoff and phase
snapshot order.
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

### Idle feedback fault recorder

`servo_idle_diagnostic` uses normal homing and saved calibration, with an
optional recorder and automatic motor-output shutdown. It does not start
motion automatically or change PID gains, current limits, or saved tables.
Build/upload it with `pio run -e servo_idle_diagnostic -t upload` in
`firmware/MotionControllerRP`.

The diagnostic enables encoder CRC checking. A CRC-rejected reading does not
change the accumulated encoder position or the raw-angle history used to
count revolutions. CRC is not proof of a correct position: an entirely zero
reply also has a valid CRC. Some modules may not support CRC correctly.

While feedback controls a powered motor, it stops on an encoder status error,
a reading outside calibration, or non-finite control values. It also stops
if target-minus-measured position exceeds 1 motor-shaft degree immediately,
or 0.15 motor-shaft degree for 50 ms; or if the PID field correction remains
at least 78 electrical degrees for 100 ms after the homing handover. The
existing handover-failure check also cuts output in this build. These are
conservative diagnostic thresholds, not validated limits for normal motion.
Their duration timers reset when motor power or feedback is paused/restarted;
time spent disabled cannot count toward a continuous-error cutoff.

A fault switches all three motor PWM amplitudes to zero on the servo core,
invalidates homing, freezes all records, and prevents commands from enabling
motion again. The fault remains latched until reboot. `M18`, `M57`, `M58`, and
`M60` remain available; do not reboot/reflash before collecting the records.
This is software protection, not an independent hardware emergency stop.

`M60 J0`, `M60 J1`, and `M60 J2` dump each joint's chronological record.
Each holds 512 samples, collected at most once per millisecond, plus a forced
final fault sample replacing the oldest entry when full. This normally covers
roughly the preceding half-second. Recording does not write serial output or
flash during control; dumping copies the record under the joint lock and
prints after releasing it. A disabled motor does not fill the trace.

The header contains the zero-based joint, fault number, frozen flag, and
sample count. Fault numbers are: 0 none, 1 encoder status, 2 calibration range,
3 invalid number, 4 tracking error, 5 sustained correction, 6 failed homing
handover. Other joints can be frozen with fault 0 when one joint fails.

CSV columns:

| Column | Meaning |
| --- | --- |
| `time_us` | Microseconds since firmware startup. |
| `raw` | Accepted accumulated encoder count; on CRC rejection, the previous accepted count. |
| `status` | Encoder flags: 1 overspeed, 2 weak field, 4 undervoltage, 8 CRC failure; flags can combine. |
| `target_deg` | Requested calibrated motor-shaft position, degrees. |
| `measured_deg` | Calibrated motor-shaft position derived from feedback, degrees. |
| `velocity_deg_s` | Filtered measured motor-shaft speed, degrees/second. |
| `correction_deg` | Computed PID correction to the drive field, electrical degrees. |
| `field_deg` | Last field angle actually written to the driver, electrical degrees. |
| `dt_us` | Time interval passed to the controller, microseconds; not necessarily the wall-clock sample interval. |

An early encoder-status/range fault is recorded before converting or applying
that reading: the measured position and controller values then describe the
previous update. At any fault, `field_deg` retains the last applied field,
not a rejected new command. An encoder error demonstrates a detected feedback
problem, but does not by itself identify a wire, module, or the original
runaway's cause. A trace with no CRC errors also cannot rule out bad feedback.

For a motor-disabled SPI-speed comparison, `encoder_spi8m_test` runs the
existing wiring diagnostic at normal firmware's 8 MHz, whereas
`encoder_wiggle_test` uses 1 MHz. Both disable the drivers and report CRC,
status, raw-angle changes, and periodic register communication checks.

Host checks (no device access):

```bash
bash firmware/MotionControllerRP/test/run_host_feedback_diagnostic.sh
bash firmware/MotionControllerRP/test/run_host_encoder_rejection.sh
```

These checks compile the actual update, shutdown, enable/restart, and encoder
reader methods against mocked hardware. They cover chronological records,
fault latching and all-joint shutdown, rejected-packet revolution history,
invalid numeric inputs, refusal to re-enable after a fault, and duration-timer
reset across a feedback pause. They do not establish physical safety or
detect every possible corrupted encoder reply.

### Restore normal firmware

Build and upload the default `pico` environment again:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --environment pico --target upload
```

Normal robot initialization, motion, homing, and calibration will be restored.
