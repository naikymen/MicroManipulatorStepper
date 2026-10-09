# Changelog

## Update the GUI for selective calibration, homing and motor disable

- Advance the GUI submodule from `112c096` to `a8fc49e`, incorporating three
  focused commits: the 30-second homing reply timeout (`063e909`), shared axis
  homing/calibration controls (`96fdb0e`) and Disable Motors (`a8fc49e`).
- Include one full-width All/X/Y/Z selector with axis, joint and G28-letter
  labels above Home Axis, Calibrate Axis and Save. Preserve the all-axis
  default and checked persistence default while allowing individual joints
  and unsaved in-memory calibration.
- Add Disable Motors beside Set Origin with GUI playback shutdown and visible
  failure handling. It removes holding torque and is not an emergency stop.
- Include documentation, commit-specific GUI changelog entries and 28 passing
  offline Qt/API tests. Also verify the standalone API's 30-second Home wait
  with mocked serial access; no device or camera was accessed for these checks.

## Give the Python API time to finish normal Home

- Increase the standalone Python API's G28 reply timeout from 10 to 30 seconds
  so the physical-stop search, measured backoff, guarded feedback handover
  and final move into the usable range can finish before a timeout is reported.
- Preserve the existing homing command and return status. This changes the
  host-side wait only; it does not modify firmware, motion limits or calibration.

## Make calibration and Home finish at verified, usable positions

- Establish calibration's measurement origin only after raw encoder movement
  confirms 1.5 motor-shaft degrees of clearance from the physical stop. Keep
  the reduced homing drive strength during that move and restore normal drive
  strength afterward. Calibration no longer assumes that a fixed field
  rotation after a stall produced the same shaft rotation.
- Back normal Home 2.5 motor-shaft degrees from the stop so its first feedback
  position is inside, rather than directly on the edge of, the saved
  calibration table.
- Keep the existing 75-electrical-degree validation limit. For an accepted
  Home, preserve the already-applied field on the first feedback update, then
  remove only that temporary difference at 60 electrical degrees per second
  while holding the fresh encoder measurement as the target. The temporary
  difference reaches zero and is neither saved nor retained as another
  calibration offset.
- After the handover, move every selected joint smoothly to exactly 7 calibrated
  motor-shaft degrees. Monitor encoder health and tracking, and fail Home on a
  one-degree tracking excursion, invalid target range, or timeout. This places
  the stage where the current delta geometry can perform a full 1 mm jog in all
  six Cartesian directions without lookup-table extrapolation or shortening
  the requested move.
- Correct the earlier regression, which exercised only positive Cartesian
  directions from an unrealistic mocked Home position. The updated tests cover
  both directions of X, Y, and Z from the real post-Home target, plus invalid
  calibration range, encoder failure, tracking failure, transition completion,
  and calibration without a pre-existing lookup table.
- Propagate calibration-file write failures instead of reporting successful
  saved calibration when persistence failed. Keep the optional RAM-versus-saved
  calibration comparison isolated from normal firmware.
- The bounded field transition passed 20 consecutive device Homes before the
  post-Home position move was added: every temporary difference reached zero,
  the largest measured shaft-position error was 0.013138 degrees, and the final
  observed Home had no snap. The new 2.5-degree backoff and 7-degree finish move
  have passed host regressions and firmware builds but still require a later
  device test.

## Explain the normal homing flow

- Replace temporary implementation and working-tree records with one durable,
  plain-language description of a normal `G28` move.
- Explain stop detection, encoder-measured backoff, settling, the shared final
  snapshot, calibrated electrical-phase checks, feedback handover, failure
  behavior, and the software limits established by a successful Home.

## Update the GUI submodule for rejected commands

- Advance `software/OpenMicroManipulatorGUI` from `56a4dce` to `112c096` so
  jog, realtime, and Home failures remain visible and rejected targets do not
  advance the GUI's cached position.
- Preserve exact jog semantics: each click sends the selected displacement once
  and never retries, subdivides, or silently shortens a rejected request.
- Include seven mocked Qt/API regressions and the matching GUI documentation;
  these checks do not open a camera, serial port, or hardware connection.

## Derive motion limits from measured Home clearance

- Increase normal encoder-verified G28 clearance from 0.5 to 1.5 mechanical
  degrees. Retain 25 percent of each joint's actual measured clearance from the
  detected physical stop and permit exact Cartesian paths to use the remaining
  75 percent. Keep the opposite, unmeasured calibration boundary inset by 0.5
  degrees and never extrapolate outside either calibration table.
- Store the stop reference from the same final encoder snapshot used for target,
  FK and servo-history handover. Invalidate it on calibration or failed Home;
  normal Cartesian motion requires a subsequent successful measured Home.
- Add a 0.0001-degree range-comparison tolerance for floating-point kinematics.
  Accepted joint and Cartesian targets are not clamped or shortened. The GUI
  still sends the selected displacement once and visibly reports whole-command
  rejection.
- Extend M57 with measured clearance, estimated stop and active joint limits.
  Add host coverage for exact 1 mm inward X/Y/Z moves from the captured Home
  geometry, retained clearance, boundary tolerance, invalid Home references and
  a single exact GUI request with no retry. Calibration measurement, fitting,
  persistence and file format remain unchanged.
- Keep `G4` as a planner-ordered, zero-motion dwell. It captures the joint
  endpoint already accepted by the planner, bypasses Cartesian IK and travel
  enforcement during the wait, and remains available before Home without
  weakening the `G0`/`G1`/`G24` readiness checks.
- Preserve queued timing for normal tool changes, while allowing an explicit
  zero output to reach the tool immediately after a latched planner fault.
  Nonzero outputs are rejected and not retained while the queue is stopped. Add host regressions for
  pre-Home dwell, fixed targets, limit bypass, and faulted tool behavior.

## Verify G28 backoff and restart from one measured position snapshot

- Add normal-firmware encoder-verified backoff: request 1.5 degrees of measured
  shaft-equivalent clearance, remain inside both calibration domains, and cap
  physical-equivalent travel at 3.6 degrees. Bound
  field advance/time and reject wrong-direction movement, encoder errors and
  unsettled positions, including after amplitude restoration. This addresses
  the fact that a fixed field rotation after stall does not certify rotor travel.
- Start targets and controller history from one final encoder snapshot, retain
  the held field through bounded PID preload, and derive Cartesian pose with
  FK only. Take the joint lock before the servo core copies targets: the old
  order could apply a pre-Home target after a freshly initialized handover.
- Keep calibration measurement/backoff/origin, saved table format, PWM limits
  and the 75-electrical-degree phase guard unchanged. Persistent
  disagreement remains an explicit failure; no table offset or forced field
  alignment conceals it. Search timeout is terminal and finalization is once-only.
- Exercise actual production methods with host clock/driver/encoder stand-ins
  for stall-phase delays, wrap, missing/reversed/faulty/noisy readings, current
  restoration displacement, timeout, calibration-path preservation, exact
  snapshot reuse, phase refusal and joint/target lock ordering. These are offline
  checks, not evidence that the reported device failure is resolved.

## Check encoders at the normal SPI clock

- Add optional `encoder_spi8m_test`, reusing the motor-disabled encoder
  diagnostic with the robot's 8 MHz SPI clock. Keep the original wiring test
  at 1 MHz and leave normal firmware unchanged.
- Document that a low-speed wiring test cannot validate operating-speed
  communication, and that the standalone test does not reproduce servo-loop
  polling/concurrency or interference from energized motors.

## Isolate reduced-power homing backoff

- Add optional `homing_backoff_power_test`, restoring the previous amplitude
  while the field is held at the stop, before backoff rather than afterward.
  Search current, amplitude cap, guards and backoff speed/distance are unchanged;
  normal firmware and the calibration procedure retain their existing order.
- A fresh motor-2 RAM calibration passed its first guarded Home but a repeat
  failed at roughly 84 electrical degrees. This test distinguishes a
  reduced-amplitude backoff problem from simply outdated saved tables; no
  experimental calibration is saved or guard threshold raised.
- Test the actual finalization method against host driver mocks to verify
  one restoration ramp, correct trace order and unchanged default backoff.
- Live motor-2 Homes still failed at about 84 and 151 electrical degrees;
  leave this change diagnostic-only. Successive fresh fits differed by about
  102 electrical degrees at matching encoder counts, so investigate a shifting
  reference (including the rotor-to-shaft fastening) rather than saving fits
  or widening the guard. The exact physical cause remains unconfirmed.

## Compare a temporary calibration against the saved phase reference

- Add optional `calibration_guard_reference_test`, combining the existing
  normal-timing phase trace and restart guard with feedback paused at the end
  of calibration. This isolates a fresh RAM calibration without applying a
  suspect saved reference or changing production settings.
- Document the separate motor-3 calibration and Home commands, omission of `S`
  to preserve flash, and reboot recovery. The previous doubled-backoff test
  remained smooth but still reported about 91 electrical degrees of mismatch,
  versus about 93 at the default backoff; simply increasing backoff did not
  resolve the reference disagreement.
- Extend host regressions using the actual calibration-completion method to
  check selected-axis isolation, explicit-only saving, diagnostic restart
  suppression, and normal restart-refusal propagation.
- Motor 3's temporary fit enabled two guarded Homes at about 3-degree mismatch.
  Motor 2's temporary fit passed once then failed a repeat at about 84 degrees;
  recalibration alone is not established as a reliable correction.

## Expose failed homing and specific motion rejections

- Check Home's reply in the GUI, report failures and retain the prior cache on
  failure. Physical stop detection/backoff is not sufficient when the firmware
  subsequently refuses feedback restart; silently ignoring that error hid why
  all Cartesian jogs were blocked.
- Report the offending joint, motor-angle interval and path fraction for motion
  rejection, and add read-only phase/current/target snapshots to `M57`.
- Add `homing_guard_trace_test`, which inherits normal guards/current/timing and
  records phases around backoff and amplitude restoration without diagnostic
  pauses or automatic motion. Keep the 75-degree restart guard, PID limits,
  current settings and saved calibrations unchanged while investigating the
  repeated roughly 93-degree electrical mismatch on axis 3.
- Initialize shared target arrays before the first servo update/diagnostic and
  extend offline regressions for failure messages and GUI Home failure handling.

## Enforce calibrated joint travel for Cartesian commands

- Validate `G0`/`G1` paths and `G24` targets against the intersection of each
  joint's two calibration domains and the configured sweep, with 0.5-degree
  endpoint clearance. Reject failed/non-finite IK and unready joints rather than
  allowing a physically stopped motor to leave the other motors moving.
- Preflight path interiors and check every generated linear joint segment and
  published target. Latch unexpected execution faults without sending partial
  joint updates; Home clears the queued trajectory for recovery.
- Leave rejected commands' pose/feedrate unchanged and prevent realtime targets
  from racing an executing planned path. Preserve homing/calibration procedures,
  motor current and normal jog speeds.
- In the GUI submodule, refresh jog targets, preserve the last accepted target
  on rejection, display controller errors, and stop rejected realtime control.
  These changes avoid stale/unreachable GUI targets; they do not diagnose the
  reported button-versus-mouse Y behavior or add an encoder watchdog.
- Add offline actual-kinematics, command-handler and Qt regressions, documenting
  that calibrated command limits are not a physical-limit guarantee.

## Configure a persistent PlatformIO serial device

- Configure upload and serial monitoring to use the controller's stable Linux
  `/dev/serial/by-id` path at the firmware's 921600 baud rate.
- This avoids uploads and monitoring targeting the wrong device when transient
  `/dev/ttyACM*` numbers change after reconnecting or rebooting the controller.

## Add an optional standalone motor-driver test mode

- Add a compile-time `MOTOR_STEP_TEST` mode with low-duty 1/16-microstep moves,
  individual phase holds, and automatic output disable after each test.
- Document its serial commands, current-limiting assumptions, and restoration
  procedure in the setup guide.
- The mode is disabled by default so production motion and calibration behavior
  is unchanged; it exists to distinguish wiring, motor, socket, and driver
  faults without involving encoder feedback or closed-loop control.

## Fix RP2350 multicore synchronization

- Replace hard-coded hardware spinlock IDs 0 and 1 with locks claimed from the
  Pico SDK's unused pool during robot initialization.
- The fixed IDs overlap SDK-reserved locks and caused the second-core servo loop
  to interfere with motor control; runtime allocation prevents that collision
  while preserving the existing locking model.

## Correct shared motor-driver standby handling

- Make motor enable operations explicitly reassert the TB6612 standby pin, and
  make `M18` enter standby only after all three channel amplitudes reach zero.
- The board shares this enable pin across every driver; managing it as a shared
  resource prevents calibration from silently producing no motion after `M18`
  or a diagnostic while still leaving the hardware disabled when requested.

## Reject invalid calibration fits

- Correct lookup-table error reporting to calculate root-mean-square error
  instead of the sample-count-dependent root-sum-square value.
- Treat non-finite encoder fits and fits above 0.5 degrees RMS as calibration
  failures rather than warnings, preventing disconnected or flat encoder data
  from being accepted and saved as a valid calibration.

## Tune motor current and calibration speed for this machine

- Set the motor PWM amplitude cap to 30% for the 12 V supply and approximately
  6-ohm windings, and reduce homing and calibration field velocities.
- These conservative values limit average winding current to roughly 0.6 A,
  stay within the available 2-3 A power-supply budget, and produced reliable
  motion and sub-0.15-degree calibration fits on all three installed axes.

## Add opt-in Python calibration controls

- Add optional calibration flags for saving, quiet output, fail-fast handling,
  headless execution, and disabling the motors after a run, plus a documented
  safe command that combines them.
- Extend the API calibration timeout from 30 to 120 seconds so conservative
  current and speed settings can complete without a false host-side timeout.
- All flags default off, preserving the script's original verbose, unsaved,
  continue-on-error, plotting behavior for existing users.

## Add the Open Micro-Manipulator GUI as a submodule

- Register `software/OpenMicroManipulatorGUI` as a Git submodule pointing to
  its upstream `0x23/OpenMicroManipulatorGUI` repository.
- Keeping the GUI as a pinned submodule makes its exact compatible revision
  reproducible without copying or mixing its independent history into the
  firmware repository.

## Document firmware and debug-mode workflows

- Add a firmware README covering the hardware interfaces, multicore runtime,
  persistent calibration data, source layout, configuration locations, and
  command-line build workflow not detailed by the main project README.
- Explain how to enable, build, flash, monitor, use, and disable both firmware
  diagnostics, emphasizing mutual exclusion, motor-current safety, and
  restoration of normal firmware after testing.

## Add dedicated PlatformIO diagnostic environments

- Add `encoder_wiggle_test` and `motor_step_test` build environments while
  keeping `pico` as the default normal-firmware environment.
- Move diagnostic selection out of `hw_config.h`, preventing stale source
  toggles and accidental simultaneous activation while allowing each image to
  be built and flashed with one explicit PlatformIO command.
- Update the firmware and setup documentation with the new environment-based
  build, upload, monitoring, and normal-firmware restoration workflow.

## Update the GUI for clean terminal interruption

- Advance the GUI submodule to include Ctrl+C handling: a Qt timer dispatches
  SIGINT requests through window cleanup, and serial disconnection is ensured
  when the event loop exits.
- This allows the GUI to stop cleanly from its launching terminal even while
  idle; verified in an offscreen Qt run with mocked hardware.
- The updated reference also includes the previously committed incremental
  jog-limit fix (`57bbae6`), which avoids jumps back to the allowed boundary.

## Preserve servo state across motor disable and feedback restart

- Keep the shared driver standby line inactive at boot until a motor is
  explicitly enabled. At zero amplitude, continue encoder tracking without
  advancing the field command or accumulating PID correction.
- Pause feedback before `M18`, exclude unpowered channels from feedback
  restart, and resume eligible powered channels after `M17`.
- Refresh encoder-based velocity history when feedback resumes and support
  a bounded initial PID integral for retaining the held field. These changes
  address stale-history impulses and corrections accumulated while motors
  were off; current and PID output limits are unchanged.

## Use the tested continuous-field homing handover in normal firmware

- Enable held-field PID preload and calibration/phase preflight in the normal
  `pico` environment, without diagnostic pauses or feedback cutoffs. Refused
  restarts clear homed status and report failure rather than silently
  accepting an invalid handover.
- Slow the electrical backoff to 10 rad/s and default `G28` to a 3.6-degree
  mechanical-equivalent field command. The former 1.8-degree command could
  leave the rotor outside its calibration table, where its position was
  reported as a fixed endpoint and restarting feedback caused a snap.
- Keep calibration's separate measurement origin, saved data, PWM caps,
  existing travel restrictions, and PID correction limits unchanged. All
  three motors completed individual observed Home/hold checks without a
  reported post-home snap on the normal firmware.

## Add optional staged and bounded homing diagnostics

- Add separate PlatformIO environments to isolate backoff, current restoration,
  field-reference disagreement, and feedback restart, including an open-loop
  calibration comparison that only saves when explicitly requested with `S`.
- Capture the first feedback command and bounded snapshots in memory on core 1,
  freezing field updates after 50 ms, one second, or ten seconds, or a nominal
  two-degree raw-encoder excursion. A core-0 fallback also pauses updates;
  previous pulse records are cleared before each Home to avoid stale results.
- Keep diagnostic pauses, traces, cutoffs, and calibration overrides disabled
  in the normal environment. Document electrical versus mechanical units,
  motor-disable commands, and why these diagnostics are not physical safety
  limits. These modes made the observed snap reproducible and separable from
  motor enable/current changes without logging in the active servo loop.

## Add host regression checks for servo restart and record live validation

- Add a device-free runner that compiles the actual servo update/restart,
  PID, and restart-preflight methods against encoder, driver, and clock mocks.
  Check disabled-field retention, fresh history, bounded preload, phase
  wrapping, pulse expiration/excursion cutoff, snapshot timing, and explicit
  refusal/readiness invalidation, including baseline comparison builds.
- Document the runner and its hardware limitations so future changes can
  repeat these checks without moving the mechanism. Both production-fix
  staged snapshots and all eleven final normal/homing build environments
  compiled successfully.
- Live normal-firmware validation used fresh Home before each first X/Y/Z
  move: 0.1 mm out and back at 0.1 mm/s, with motors continuously powered.
  Encoder-derived motor increments matched the kinematic predictions; return
  errors were below 0.001 motor degree. All three individual Home/20-second
  hold observations reported no post-backoff snap or twitch. These are sampled
  encoder checks and user observations, not independent stage metrology or
  proof against intermittent faults. Saved calibration entries were also
  verified after normal firmware was flashed.
