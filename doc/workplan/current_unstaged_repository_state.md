# Current unstaged repository state

## Snapshot

This document records the working tree on 2026-10-06 after the code-only fix for
unusable 1 mm Cartesian jogs and the follow-up G4/tool-output separation.

```text
Branch: naiky
HEAD:   documentation commit following 05d537e (three commits ahead of origin/naiky)
Index:  no staged changes
Tree:   workplan files pending, plus unrelated modified files
GUI:    clean submodule at 112c096 on its main branch
```

No hardware was available while this snapshot was written. No firmware was
uploaded and no device command was sent.

The broad objective of the current work is to make calibration, Home, feedback
restart, and Cartesian motion behave safely and predictably:

- Home must detect the physical stop, back away by verified encoder movement,
  and enter feedback without a snap.
- A failed Home or invalid calibration/phase relationship must remain visible.
- Cartesian commands must never continue moving the other motors after one
  joint has exhausted its permitted range.
- A requested jog must execute exactly or fail visibly; it must never be
  shortened silently.
- Calibration measurement and its saved format should remain compatible with
  upstream behavior.

## Overall progress

| Workstream | State | Progress toward objective |
| --- | --- | --- |
| Measured G28 backoff | Implemented and offline-tested | Encoder movement, direction, status and settling are verified. Normal Home now requests 1.5 degrees of measured clearance. |
| Homing-to-servo handover | Implemented and offline-tested | Targets, PID/history and FK start from one final encoder snapshot. Prior live tests reported smooth Home without the original snap. |
| Phase/restart guard | Implemented and retained | Invalid phase references refuse restart instead of snapping. Fresh calibration allowed repeated Homes, but a repeatable phase-search offset remains under investigation. |
| Cartesian travel enforcement | Implemented and offline-tested; hardware validation pending | Whole paths and generated segments reject invalid IK/ranges atomically. The Home side now derives from measured clearance instead of a symmetric fixed inset. |
| Exact GUI rejection handling | Implemented and offline-tested | Jog, realtime, and Home errors remain visible and rejected targets do not advance GUI state. No shorter-step fallback exists or is planned. |
| Dynamic Home-clearance limits | Implemented and offline-tested | Each Home stores its final position, measured clearance and direction; 25% is retained while 75% is usable. |
| Diagnostic firmware modes | Implemented | Phase trace, RAM-only calibration comparison, early-current-restore comparison, and 8 MHz encoder diagnostics are available as opt-in environments. |
| Documentation and host regressions | Updated and code-only validation complete | The G4/tool-output regression is fixed; all host suites, all 17 firmware environments, and seven GUI regressions pass. Hardware results remain later validation. |

## Defect addressed in the firmware commit

The defective pre-fix implementation combined:

```text
HOMING_MEASURED_BACKOFF_ANGLE_DEG = 0.5 deg
JOINT_TRAVEL_MARGIN_DEG           = 0.5 deg
```

That lower motion boundary consumed approximately the entire requested
Home clearance. Recorded post-Home joint positions around 0.596, 0.666 and
1.246 degrees leave too little coupled travel for normal 1 mm Cartesian moves.

The message below also exposes a floating-point boundary issue:

```text
joint 1 target 0.499997 deg outside 0.500000..82.456589 deg
```

The implemented correction is documented in
[`home_clearance_cartesian_motion.md`](home_clearance_cartesian_motion.md):

- use 1.5 degrees of measured normal-Home backoff as the initial value;
- derive the home-side limit from the actual measured clearance;
- retain 25 percent of that clearance and permit motion to use 75 percent;
- keep the unmeasured opposite boundary conservative;
- tolerate only floating-point-sized boundary differences; and
- preserve exact-or-visible-failure GUI behavior.

These changes are implemented and covered by host/GUI regressions. They have not
been flashed or tested on hardware.

## Root repository changes by purpose

### Measured Home, handover, and phase investigation

| File | Current purpose | Progress/status |
| --- | --- | --- |
| `firmware/MotionControllerRP/src/hw_config.h` | Configures 1.5-degree measured Home backoff, 75% usable clearance, 0.5-degree opposite margin, 0.0001-degree numeric tolerance, timeout and settling checks. | Implemented and offline-tested. |
| `firmware/MotionControllerRP/src/servo_control/homing_controller.h` | Adds an explicit failed state, measured-backoff mode, optional early-current-restore mode, verification helpers, and final-snapshot clearance reporting. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/servo_control/homing_controller.cpp` | Verifies encoder status, backoff direction, measured clearance, calibration-domain entry and settling; reports clearance/direction at the handover sample; preserves calibration's fixed-field path. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/servo_control/servo_controller.h` | Allows feedback restart to reuse an already captured measured position. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/servo_control/servo_controller.cpp` | Initializes motor position and velocity history from the supplied Home snapshot without a second encoder read. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/robot.h` | Declares measured handover, per-joint Home references, asymmetric travel-limit calculation, motion reset and error state. | Implemented. |
| `firmware/MotionControllerRP/src/robot.cpp` | Coordinates measured G28, stores the final Home reference, calculates asymmetric limits, invalidates references on calibration/failure, reports M57 clearance/limits, and performs the single-snapshot guarded handover. | Implemented and host-tested; hardware validation pending. |
| `firmware/MotionControllerRP/platformio.ini` | Enables measured backoff in normal `pico`; adds guarded trace, RAM calibration-reference, early-current-restore and 8 MHz encoder environments. | Implemented. Diagnostic flags remain opt-in except measured backoff/guard/bumpless handover in normal firmware. |

### Cartesian command safety

| File | Current purpose | Progress/status |
| --- | --- | --- |
| `firmware/MotionControllerRP/src/motion_control/motion_limits.h` | New fail-closed per-joint calibrated interval abstraction with explicit comparison tolerance and bound accessors. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/motion_control/motion_limits.cpp` | New finite-pose/IK validation and Cartesian interior preflight with specific rejection text and tolerance reporting. | Implemented and host-tested; accepted targets remain unchanged. |
| `firmware/MotionControllerRP/src/motion_control/path_planner.h` | Stores joint limits, rejection reason and a latched execution fault; exposes reset/abort/status operations. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/motion_control/path_planner.cpp` | Rejects invalid paths before queuing, checks generated joint-segment endpoints, latches unexpected violations and supports Home recovery. | Implemented and host-tested. Existing compiler warnings in unrelated look-ahead variables remain. |
| `firmware/MotionControllerRP/src/motion_control/path_segment.cpp` | Uses checked inverse kinematics for motion and represents G4 as a fixed-target, zero-motion segment that does not invoke IK. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/motion_control/motion_controller.h` | Adds reset and active-segment status and reports whether the active segment requires joint-limit publication checks. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/motion_control/motion_controller.cpp` | Stops target generation when a motion-limit fault is latched and propagates the active segment's limit-enforcement policy. | Implemented and host-tested. |
| `firmware/MotionControllerRP/src/robot.cpp` | Applies limits to G0/G1/G24, makes rejection atomic, prevents planned/realtime races, validates feedrate/dwell, keeps G4 zero-motion, provides faulted-state zero-only tool shutoff, checks targets before ISR publication, and reports the offending joint/path fraction. | Implemented; exact 1 mm inward X/Y/Z requests and dwell/tool regressions pass. |

### Encoder and homing diagnostics

| File | Current purpose | Progress/status |
| --- | --- | --- |
| `firmware/MotionControllerRP/src/main.cpp` | Makes encoder-diagnostic SPI frequency configurable while retaining 1 MHz by default. | Implemented. |
| `documentation/setup_guide/encoder_wiggle_test.md` | Documents the normal-speed 8 MHz encoder test and its limitations. | Documentation complete for current diagnostic. |
| `firmware/MotionControllerRP/platformio.ini` | Defines `encoder_spi8m_test`, `homing_guard_trace_test`, `calibration_guard_reference_test`, and `homing_backoff_power_test`. | Implemented; no diagnostic runs automatically. |

### Documentation and work records

| File | Current purpose | Progress/status |
| --- | --- | --- |
| `changelog.md` | Records measured Home/handover, dynamic clearance limits, encoder-speed checks, phase/calibration experiments, motion limits and GUI rejection behavior. | Updated for the implemented fix; hardware validation remains pending. |
| `firmware/README.md` | Documents command limits, measured G28, diagnostic environments, M57 fields and host tests. | Updated for the implemented behavior and code-only validation. |
| `doc/workplan/home_clearance_cartesian_motion.md` | Records the exact-jog/Home-clearance design and its implementation state. | Implemented and offline-tested; hardware validation pending. |
| `doc/workplan/current_unstaged_repository_state.md` | Records this working-tree inventory and progress snapshot. | Current document. |

### Peripheral changes not required by the current objective

| File | Current purpose | Progress/status |
| --- | --- | --- |
| `.gitignore` | Ignores `construction/camera/*`. | Unrelated to homing/motion work; preserve and review separately before committing. |
| `firmware/MotionControllerRP/.vscode/extensions.json` | Recommends PlatformIO/PioArduino and ESP decoder; marks the C++ extension pack unwanted. | Editor preference, unrelated to firmware behavior; preserve and commit separately or leave to its owner. |

## Untracked and modified host tests

| File | Coverage | Current result |
| --- | --- | --- |
| `firmware/MotionControllerRP/test/host_measured_homing.cpp` | Actual 1.5-degree measured-backoff logic with varied stall lag, published clearance/direction, encoder wrap, frozen/reversed/faulty/noisy readings, restore displacement, timeout, invalid domains, and unchanged calibration backoff. | Passes. |
| `firmware/MotionControllerRP/test/host_homing_handover.cpp` | One encoder sample, exact target/PID-history reuse, FK-only pose, held-field preservation, dynamic Home references/limits in both directions, phase/status/range/FK refusal, and lock order. | Passes. |
| `firmware/MotionControllerRP/test/host_homing_current_order.cpp` | Normal versus diagnostic early amplitude restoration and trace order. | Passes. Diagnostic remains investigation-only. |
| `firmware/MotionControllerRP/test/host_calibration_reference.cpp` | Selected-axis calibration, explicit-only saving, RAM-reference restart suppression, failure propagation and invalid index handling. | Passes. Confirms calibration behavior is separate from measured normal G28. |
| `firmware/MotionControllerRP/test/host_motion_limits.cpp` | Finite IK, numeric-boundary tolerance, interior-path rejection, all axes/both ends, generated-segment faulting, valid paths, and fixed-target dwell without IK or limit enforcement. | Passes. |
| `firmware/MotionControllerRP/test/host_motion_commands.cpp` | Actual G0/G1/G24/G4/tool handlers, dynamic limits, exact 1 mm inward X/Y/Z requests, atomic rejection, readiness, LUT intersection, busy/fault behavior, pre-Home dwell, and faulted zero-only tool shutoff. | Passes. |
| `firmware/MotionControllerRP/test/host_servo_restart.cpp` | Existing restart behavior plus reuse of a supplied Home snapshot without an extra encoder read. | Modified tracked test; passes all runner variants. |
| `firmware/MotionControllerRP/test/host_stubs/hardware/sync.h` | Minimal interrupt stubs for host-building production motion code. | Present and used successfully. |
| `firmware/MotionControllerRP/test/run_host_measured_homing.sh` | Builds/runs measured Home and handover tests from production methods. | Passes. |
| `firmware/MotionControllerRP/test/run_host_motion_limits.sh` | Builds/runs motion-limit and actual Robot command tests. | Passes with pre-existing compiler warnings. |
| `firmware/MotionControllerRP/test/run_host_servo_restart.sh` | Existing restart matrix plus current-order and calibration-reference variants. | Passes. |

## GUI submodule state

The GUI changes are committed as `112c096` (`Handle rejected stage commands
safely`). Parent commit `ae148aa` advances the recorded submodule revision from
`56a4dce` to that commit and includes the matching root changelog entry.

| File | Current purpose | Progress/status |
| --- | --- | --- |
| `software/OpenMicroManipulatorGUI/source/mainwindow.py` | Checks Home replies, blocks conflicting realtime control, refreshes M50 before a jog, sends the exact selected target once, updates cache only on OK, and displays rejection details. | Implemented and tested. No step reduction/retry is present. |
| `software/OpenMicroManipulatorGUI/source/hardware/open_micro_stage_api.py` | Preserves controller error strings for Home, G0/G1 and G24 callers. | Implemented and tested. |
| `software/OpenMicroManipulatorGUI/source/gui_components/realtime_controller_widget.py` | Publishes a candidate only after controller acceptance; stops realtime control and warns on rejection. | Implemented and tested. |
| `software/OpenMicroManipulatorGUI/tests/test_motion_rejection.py` | Offline mocked Qt/API tests for axis-button wiring, stale-cache refresh, one exact request with no shorter retry, Home failures, realtime failure and error propagation. | Seven tests pass. |
| `software/OpenMicroManipulatorGUI/README.md` | Documents Home, jog and realtime rejection behavior and offline test command. | Matches current GUI behavior; test count is not stated. |
| `software/OpenMicroManipulatorGUI/changelog.md` | Records rejected-motion and failed-Home handling. | Matches current GUI changes. |

The previously committed GUI change `57bbae6` only prevents a stale Cartesian
box clamp from jumping to its boundary. It does not subdivide a controller-
rejected move.

## Verification performed for this snapshot

Fresh code-only checks on 2026-10-06:

```text
bash firmware/MotionControllerRP/test/run_host_motion_limits.sh   PASS
bash firmware/MotionControllerRP/test/run_host_measured_homing.sh PASS
bash firmware/MotionControllerRP/test/run_host_servo_restart.sh   PASS
All 17 PlatformIO environments (including pico)                  BUILD PASS
GUI mocked Qt/API suite                                           7 PASS
git diff --check (root and GUI submodule)                         PASS
```

The GUI suite was run directly with the repository virtual environment and an
offscreen Qt platform. An attempted `pytest` invocation was unavailable because
that environment does not install pytest; the suite uses `unittest` and passed
when invoked through its documented entry point.

The host motion builds report existing unused-variable and signedness warnings;
they do not fail the suites.

## Prior hardware evidence relevant to these changes

- The continuous-field/bumpless handover removed the observable post-Home snap
  in repeated user observations.
- Individual normal Home and motion checks were smooth in the tested direction.
- Encoder diagnostics helped identify wiring faults; later readings were stable
  enough to complete calibration and repeated Home tests.
- A fresh motor-2 RAM calibration allowed 20 consecutive guarded Homes. Their
  electrical disagreement was about 22.6 degrees for the first group and about
  29.0 degrees after a 6.18-degree step. Approximately 5.94 degrees of that step
  came from the held field and only about 0.24 degrees from the encoder-derived
  reference.
- The records are consistent with one additional 36-degree stall-assessment
  window being only partially compensated by backoff. This is evidence for
  search/backoff field-selection variability, not proof of a specific component
  fault.
- The new Cartesian limiter correctly prevented continued coupled movement at
  a stopped joint, but its former symmetric 0.5-degree inset rejected ordinary
  post-Home jogs. Commit `05d537e` replaces it with measured Home clearance;
  this correction still needs hardware validation.

These observations are not current validation because hardware is unavailable.

## Remaining work, in order

1. Commit these workplan records with their changelog entry.
2. Push GUI commit `112c096` to the GUI remote before publishing parent commit
   `ae148aa`; the remote GUI `main` currently still points to `56a4dce`.
3. Keep the unrelated `.gitignore` and VS Code recommendation changes out of
   those commits unless they are reviewed and committed separately.
4. Track the upstream/pre-existing defects below as later work rather than
   expanding the current Home-clearance commit.
5. When hardware returns, perform the deliberate validation procedure in the
   Home-clearance work plan before treating the motion limits as physically
   validated.

## Commit-readiness review and scope decisions

| Finding | Origin | Decision for current work |
| --- | --- | --- |
| `G4` required valid Home/travel references, which also obstructed fail-safe tool-off after a planner fault. | Introduced by the working-tree motion-limit gate that preceded `05d537e`. | **Fixed in `05d537e`.** Dwell is zero-motion and limit-independent; only explicit zero output bypasses a fault-stopped planner, while nonzero output is rejected and not retained. Targeted regressions pass. |
| `pose_from_joint_angles()` logs failed forward kinematics but returns a default pose; `M17` and calibration cannot reliably propagate that failure. | Upstream behavior from `d9888ef`; the current `set_pose()` return check exposes but does not create it. | **Later follow-up.** Do not claim that the current M17 check validates FK. Normal measured G28 already uses its separate checked FK handover. |
| Failed calibration can retain stale `is_homed`/`is_calibrated`/`all_joints_ready`; calibration completion also ignores save and pose-synchronization failures. | Upstream calibration lifecycle/error handling. The current work only invalidates the new Home travel reference, so Cartesian motion remains fail-closed. | **Later follow-up.** Remove any current-document claim that calibration save/error behavior was fully validated. |
| `M18` disables drivers without cancelling an active or queued trajectory, allowing planner progress and possible old-path resumption after `M17`. | Upstream `M18` behavior; not introduced by measured Home or travel limits. | **Later follow-up.** Treat as a separate motor-disable/recovery change with dedicated interruption tests. |

The later classification means “not part of the current focused commit,” not
“harmless.” Those defects should receive separate fixes and tests after the
current regression is removed.

## Commit-readiness assessment

- **Measured Home/handover core:** implementation and existing host regressions
  are complete; physical validation remains.
- **Motion limits:** limit enforcement and the G4/tool separation pass targeted
  checks and the complete code-only validation matrix.
- **GUI error handling:** committed and recorded by the parent repository,
  including the seven-test suite and explicit no-retry assertion.
- **Firmware, diagnostic environments, and host tests:** committed together as
  `05d537e`; the diagnostics are opt-in validation assets and do not alter the
  default image beyond the explicitly documented measured-Home flags.
- **Documentation:** updated for the completed G4 fix and explicit deferral of
  upstream lifecycle defects; this workplan commit is the remaining repository
  change for the objective.
- **Peripheral `.gitignore` and VS Code changes:** unrelated; do not mix into
  homing/motion commits without explicit review.
