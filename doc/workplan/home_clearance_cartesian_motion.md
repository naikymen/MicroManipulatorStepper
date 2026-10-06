# Home clearance and exact Cartesian motion

## Status

The Home-clearance and exact-motion design was implemented and passed its
targeted code-only validation on 2026-10-06. The `G4` regression introduced
while applying the travel-readiness gate was corrected: dwell is now an
explicit zero-motion segment that retains planner ordering without requiring
Home/travel metadata. No hardware validation is available at the time of
writing, so the work is not yet considered physically validated.

Implemented behavior:

- normal measured G28 clearance is 1.5 mechanical degrees;
- the Home-side command boundary retains 25 percent of the clearance measured
  during that specific Home;
- the opposite calibration boundary retains its fixed 0.5-degree margin;
- range comparison has a 0.0001-degree floating-point tolerance without target
  clamping; and
- the GUI sends one exact jog request and never retries a shorter displacement.

## Required behavior

- A requested Cartesian displacement must either execute completely and exactly
  or fail visibly.
- The GUI must not retry a rejected jog with a shorter displacement.
- A successful Home must leave a measured clearance from the physical stop.
- Normal motion may use a configured fraction of that measured clearance while
  retaining a physical reserve.
- Normal motion must remain inside both saved calibration-table domains.
- This work must not change calibration measurement, fitting, persistence, or
  saved-table format.
- Motion readiness and joint travel limits must apply to commands that can move
  the axes (`G0`, `G1`, and `G24`), not to a zero-motion `G4` dwell merely
  because dwell currently shares the Cartesian planner queue.
- A pending tool output must always have a fail-safe way to reach zero, including
  before Home and after a latched motion fault.

## Observed failure

After full calibration and Home, a 1 mm GUI jog can be rejected with a message
such as:

```text
joint 1 target 0.499997 deg outside 0.500000..82.456589 deg;
path fraction 0.110000
```

The rejection was produced by two then-uncommitted firmware changes acting
together before the correction committed as `05d537e`:

1. Normal Home requests only 0.5 degrees of encoder-measured shaft backoff via
   `HOMING_MEASURED_BACKOFF_ANGLE_DEG`.
2. The new calibrated motion limiter then removes 0.5 degrees from the lower
   calibration boundary via `JOINT_TRAVEL_MARGIN_DEG`.

Measured joint positions immediately after Home were approximately:

```text
0.596 deg, 0.666 deg, 1.246 deg
```

With a lower command limit of 0.500 degrees, the first two joints have very
little permitted travel toward Home. Cartesian motion couples all three joints,
so moving one Cartesian coordinate can require another joint to decrease. This
causes rejection almost immediately even when the requested stage motion is
toward its normal workspace.

The `0.499997` versus `0.500000` comparison is a separate numerical-boundary
problem. The difference is only 0.000003 degrees and is floating-point noise.

## Relevant branch history

- Commit `515d37f` changed normal G28 to a 3.6-degree mechanical-equivalent
  field-command backoff. It did not verify the resulting physical movement.
- The subsequent measured-backoff implementation replaced that normal
  backoff with 0.5 degrees of encoder-verified shaft movement.
- The initial motion-limit implementation introduced a fixed
  0.5-degree inset at both calibration boundaries.
- The GUI currently sends the selected jog distance once and displays a
  controller rejection. That exact-or-fail behavior must be retained.

## Offline kinematic result

Using the captured post-Home joint pose and the production Delta3D kinematic
model, the current 0.5-degree clearance is insufficient for the three inward
1 mm Cartesian jogs.

If Home backs off one additional degree, giving 1.5 degrees of measured total
clearance, the minimum joint angles required by those exact moves are
approximately:

| Move | Minimum joint angle along the move |
| --- | ---: |
| +X 1 mm | 0.857 deg |
| +Y 1 mm | 1.070 deg |
| +Z 1 mm | 0.793 deg |

This calculation supports 1.5 degrees as an initial measured-backoff value for
implementation and offline regression testing. It is not a substitute for later
hardware validation.

Moves outward from the homed corner can remain physically impossible. Those
moves must fail as complete requests rather than being shortened or partially
executed.

## Implemented behavior

### 1. Increase verified normal backoff

Set normal measured G28 backoff to 1.5 mechanical degrees. Keep the existing
encoder verification, direction checks, settling checks, field-advance bound,
timeout, current settings, and 3.6-degree maximum-clearance bound.

This changes only normal Home. Calibration retains its existing separate fixed
backoff and measurement origin.

### 2. Derive the home-side command limit from measured clearance

Retain 25 percent of each joint's actual measured Home clearance as a physical
reserve. Permit normal Cartesian motion to use the other 75 percent.

For each successfully homed joint, calculate:

```text
estimated_stop = final_position - away_from_stop_sign * measured_clearance

home_side_limit = estimated_stop
                + away_from_stop_sign
                * measured_clearance
                * retained_clearance_fraction
```

where:

```text
retained_clearance_fraction = 0.25
```

The calculation must support either joint direction rather than assuming that
motion away from the stop always increases the calibrated position.

The resulting interval must be intersected with:

- the encoder-to-position lookup-table output range;
- the position-to-field lookup-table input range; and
- `0..CALIBRATION_RANGE`.

No command may rely on lookup-table extrapolation.

### 3. Keep the opposite boundary conservative

Home locates only one physical end. Retain the existing 0.5-degree inset at the
opposite calibration boundary because no equivalent physical-stop measurement
exists there.

The implementation replaces the single `JOINT_TRAVEL_MARGIN_DEG` setting with
separate concepts:

- a measured home-side clearance fraction;
- an opposite calibration-boundary margin; and
- a tiny numerical comparison tolerance.

### 4. Treat floating-point noise as in-range

Use a small angular comparison tolerance for range checks so a computed value
such as 0.499997 degrees is not rejected against a nominal 0.500000-degree
boundary.

The tolerance must not alter the requested Cartesian pose or clamp the joint
target. It only recognizes equivalent boundary values produced by floating-point
kinematics. Targets beyond the tolerance continue to reject the entire path.

An initial tolerance of 0.0001 degrees is sufficient for the observed error and
corresponds to negligible linkage displacement.

### 5. Preserve exact GUI jog semantics

Do not add step subdivision, retry, clipping, or automatic shorter moves to the
GUI. A selected 1 mm jog sends exactly one 1 mm target. The controller either
accepts the full path or returns a visible error.

## Implementation record

### Homing controller — implemented

- Retain the final verified measured clearance after backoff and settling.
- Expose the clearance and the signed direction away from the physical stop.
- Only publish these values after successful finalization.
- Clear them on start and on every failed Home path.

Implemented files:

- `firmware/MotionControllerRP/src/servo_control/homing_controller.h`
- `firmware/MotionControllerRP/src/servo_control/homing_controller.cpp`

### Robot state and handover — implemented

- Store per-joint physical-stop/clearance metadata after successful normal G28.
- Use the exact final encoder snapshot already taken by the homing handover when
  deriving the stop estimate.
- Update metadata only for joints included in the successful Home operation.
- Invalidate a joint's metadata when its calibration changes.
- Require a subsequent successful normal Home before that joint receives the
  dynamic home-side motion limit.

Implemented files:

- `firmware/MotionControllerRP/src/robot.h`
- `firmware/MotionControllerRP/src/robot.cpp`

### Travel limits — implemented

- Extend `JointTravelLimits` to represent asymmetric per-joint bounds and a
  numerical comparison tolerance.
- Build the home-side boundary from the stored measured Home clearance.
- Build the opposite boundary from the calibrated domain and its fixed margin.
- Apply the same definitions during preflight, generated-segment checks, direct
  pose commands, and interrupt-time publication checks.
- Preserve atomic rejection: no queue, Cartesian target, feedrate, or joint
  target may advance when a path is rejected.

Implemented files:

- `firmware/MotionControllerRP/src/motion_control/motion_limits.h`
- `firmware/MotionControllerRP/src/motion_control/motion_limits.cpp`
- `firmware/MotionControllerRP/src/motion_control/path_planner.cpp`
- `firmware/MotionControllerRP/src/robot.cpp`

### Configuration and diagnostics — implemented

- Change `HOMING_MEASURED_BACKOFF_ANGLE_DEG` from 0.5 to 1.5.
- Add named settings for the retained-clearance fraction, opposite-end margin,
  and numerical tolerance.
- Extend `M57` to report, for each joint:
  - final measured Home position;
  - measured Home clearance;
  - estimated stop position;
  - calculated permitted interval; and
  - whether dynamic Home-limit metadata is valid.

Implemented files:

- `firmware/MotionControllerRP/src/hw_config.h`
- `firmware/MotionControllerRP/src/robot.cpp`

### GUI — implemented

No jog-distance fallback was implemented. Existing exact request and visible
failure handling remain unchanged, and the regression test explicitly verifies
one command per rejected click with the selected displacement unchanged.

## G4 and tool-output separation — implemented

The first unstaged implementation applied the same Home-reference gate used by
`G0`/`G1` to `process_dwell_command()`. This was introduced because `G4` had
been represented as a zero-distance Cartesian move through `PathPlanner`. It
was an implementation shortcut, not a deliberate G-code requirement, and has
now been removed.

`G4` is a dwell: it does not request a new Cartesian or joint position. It must
still validate its duration and preserve queue ordering during normal execution,
but it must not require joint travel limits solely to prove a zero-motion pose.
This firmware also uses `M3` followed by a short `G4` to apply a tool output, so
the current gate can prevent `M3 S0` from reaching the hardware when the machine
is unhomed or the planner has latched a fault.

Implemented behavior:

1. A distinct non-motion dwell representation preserves planner order
   but does not enter Cartesian IK, joint-segment generation, or joint-limit
   publication. During the wait it must leave the last joint targets unchanged.
2. Finite, non-negative duration validation and normal queue-full behavior are
   retained.
3. `G0`, `G1`, and `G24` remain subject to readiness, dynamic limits, and fault
   rejection exactly as implemented.
4. A zero tool output bypasses a planner stopped by a latched fault and reaches
   the hardware immediately. Nonzero, time-synchronized tool changes retain the
   existing queued behavior during normal operation; while faulted, nonzero
   values are rejected and not retained for later application.
5. Host command tests cover `G4` before Home, fixed-target dwell without IK or
   limit publication, fault rejection, and immediate faulted-state tool-off.

This separation was included in `05d537e` because the restrictive `G4` gate was
a regression introduced while developing the motion-limit changes.

## Offline verification

The host regressions now cover the following cases without accessing the
device:

1. Successful measured Home records the final position, clearance, direction,
   stop estimate, and retained reserve.
2. Failed, unsettled, reversed, or encoder-faulted Home never publishes valid
   limit metadata.
3. The captured post-Home configuration with 1.5 degrees of measured backoff
   accepts exact +1 mm X, Y, and Z paths.
4. A genuinely out-of-range path rejects atomically and is not shortened.
5. A target 0.000003 degrees beyond a nominal boundary is accepted by numerical
   tolerance without changing the Cartesian target.
6. A target beyond the numerical tolerance is rejected.
7. Every accepted point remains inside both lookup-table domains.
8. The home-side limit retains at least 25 percent of actual measured clearance.
9. The opposite-end 0.5-degree calibration margin remains enforced.
10. G24 realtime targets use the same limits as planned G0/G1 paths.
11. Calibration measurement, fitting, persistence code paths, and saved-table
    format are unchanged by this work. Pre-existing calibration error-propagation
    defects are tracked separately and are not claimed as validated here.
12. The GUI sends one exact jog and reports controller failure without retrying
    a shorter displacement.

The measured-homing, motion-limit/command, servo-restart, calibration-reference,
and seven-test mocked GUI suites pass. All 17 PlatformIO environments, including
normal `pico` and every optional diagnostic image, build successfully. No upload
is performed while hardware is unavailable.

The targeted host motion suite covers the `G4`/tool-output separation. After
this final change, all host suites, all 17 PlatformIO environments, and the
seven-test mocked GUI suite pass.

## Later hardware validation

When hardware access returns:

1. Flash the normal firmware and perform one full Home.
2. Read `M57` and verify that all three measured clearances and computed limits
   match the configured reserve.
3. From the fresh Home pose, command exactly +1 mm separately in X, Y, and Z,
   returning to the starting pose between tests.
4. Confirm that accepted moves complete exactly and that rejected moves produce
   no partial motion.
5. Approach the home-side limits in small explicit increments while observing
   the reported joint targets, stopping before any unexpected sound or contact.
6. Confirm that the retained physical reserve remains measurable and that the
   opposite-end protection is unchanged.

Hardware testing must be performed as a separate, deliberate step; the offline
kinematic result alone does not certify physical clearance.
