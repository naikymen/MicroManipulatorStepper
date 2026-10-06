# How a normal homing move works

This document describes the normal `G28` homing operation in the default
`pico` firmware. It first explains the motor-control information that homing
relies on, then follows the operation from start to finish.

## What homing relies on

The machine has three motor-driven axes. The firmware calls each axis a
*joint*. Each joint has an encoder that measures the motor's physical position.

During normal motion, the firmware repeatedly reads the encoder, compares the
measured position with the requested position, and adjusts the motor drive to
reduce the difference. This repeating correction is the *feedback loop*. The
requested position is the joint's *target*.

The motor contains stationary coils and a rotating part connected to its shaft.
Current through the stationary coils creates a magnetic field. The firmware
represents the direction of that field as an electrical angle, called the
*drive field*. Rotating the drive field normally makes the shaft rotate with
it. The encoder is mechanically coupled to the shaft and moves with it.

Before normal homing can be used, each joint must be calibrated. Calibration is
a separate setup procedure started with `M56`. It first moves the rotor until it
finds the physycal stop (i.e. by crashing softly into it), and then commands the
motor to slowly scan a predetermined angle range in both directions, while recording
the encoder reading and drive-field angle at many points along the path.

From those measurements, the firmware builds two "lookup tables". The first table
converts an encoder reading into the joint's mechanical position. The second
converts that mechanical position into the drive-field angle that should hold
the motor there. When calibration is saved (through a distinct command), these
tables are stored on the controller and loaded again at startup.

Normal `G28` homing uses the tables to interpret encoder readings and to validate its
final position and drive field. It does not repeat the calibration sweep or
rewrite the tables.

The machine has no electrical limit switches. It finds Home by driving each
selected joint toward its mechanical stop and detecting when the encoder no longer
follows the drive field (i.e. when the motor stalls).

## 1. Preparing to Home

When the controller receives `G28`, it discards any move that is running or
waiting to run. Homing is about to establish a new position reference, so an
older move must not resume afterward.

The firmware then pauses the normal feedback loops: a separate background task
usually reads the targets and controls the motors, so the firmware temporarily
prevents that task from reading or changing them while homing replaces the old
positions. This ensures that a position requested before homing cannot be sent
to the motors after homing finishes (as this could cause a sudden snapping move).

After a successful Home, the firmware stores three `backoff-end position` values for each joint: the motor position where backoff ended, the measured distance from that position to
the physical stop, and the direction that moves away from the stop. Together,
these values let the firmware estimate where the stop is and set the nearest
allowed motor position.

At the start of a new Home, the selected joint is marked unhomed. The firmware
also invalidates the previously stored `backoff-end position`, measured clearance
from the stop, and direction away from the stop. A new search and backoff will
measure and replace this information. This does not delete or change the
joint's calibration tables.

The motor electronics are then enabled, their previous drive strength (i.e. motor
current) is remembered, and that strength is smoothly changed to the configured homing
value.

Normal position feedback remains paused during the search. Instead, the firmware
rotates the drive field steadily toward Home and watches how the encoder reacts.
All selected joints (up to three) perform this search together.

## 2. Detecting the physical stop

While rotating the drive field, the firmware repeatedly reads the encoder.
Before the joint reaches the stop, the motor and encoder should move by a
predictable amount, directly following the amount of field rotation.

At regular intervals, the firmware compares the encoder's actual movement with
thet expected movement from the field rotation. If the encoder moved less than five percent of the expected amount, the firmware concludes that the motor has stalled against the physical stop (i.e. because the encoder no longer follows the field rotation).

Because a stalling event is almost instantaneous, this test requires a short observation
interval. The drive field can therefore continue advancing briefly after the mechanism first touches the stop, even though the shaft no longer moves.

The search fails if the encoder reports an error or if no stop is detected
within the allowed search distance. A failed search does not continue into a
normal backoff or claim that the joint is homed.

When the stop is detected, the firmware records the encoder's direct numerical
reading. It also resets the software count of complete encoder-angle
revolutions (not a count of motor-shaft revolutions). The encoder can only report
an angle within 0-360 degrees, while the software count records how many times
that angle has crossed from its highest value back to zero and adds a turn to the counter, or viceversa. Combining the two lets the firmware measure continuous movement away from the stop across that boundary. The reset is temporary position tracking;
it does not alter the saved calibration.

## 3. Backing away from the stop

After the search is complete, each selected joint is backed away from its stop.
The firmware slowly rotates the drive field in the direction opposite to the homing direction, while continuing to read the encoder.

The backoff ends only when the encoder confirms both conditions:

- the motor shaft has moved far enough from the detected stop; and
- the current encoder reading is covered by the look up tables.

The normal requested clearance is 1.5 motor shaft degrees. The firmware
checks actual encoder movement rather than assuming that a particular amount of
drive-field rotation caused the same amount of shaft movement.

Backoff fails if the encoder moves in the wrong direction, reports an error,
leaves the trusted calibration range, or indicates that the motor shaft has
rotated more than 3.6 mechanical degrees away from the detected stop. It also
fails if the destination cannot be reached within the allowed drive-field
rotation or within three seconds. The 3.6-degree limit refers to encoder-derived
motor-shaft motion, not electrical drive-field rotation.

Once the destination is reached, the encoder position must remain stable for
100 milliseconds within a 0.02-degree tolerance. The motor's previous drive
strength is then restored smoothly without changing the held field direction,
and the position must settle again. The second check makes sure that restoring
normal drive strength did not shift the motor.

## 4. Establishing the final position

After all selected joints have backed away and settled, normal feedback is still
paused. The firmware now reads every encoder once and treats that set of
readings as the final machine position.

It uses those same readings to:

- set each joint's new target to its measured position;
- set each joint's requested speed to zero;
- calculate the stage's reported X, Y, and Z position;
- record the measured distance and direction from the physical stop; and
- initialize the feedback loops for their restart.

Using the same readings for every purpose prevents the target, reported stage
position, and feedback controller from starting at slightly different
positions.

The reported X, Y, and Z position is calculated from the measured joint
positions. The firmware does not then calculate new joint positions back from
that result, because doing so could replace the encoder measurements with
slightly different calculated values.

## 5. Checking position and electrical phase

Before feedback can restart, the firmware checks that every ready joint has a
healthy encoder. Its measured position must be covered by both parts of the
saved calibration: the conversion from encoder reading to mechanical position,
and the conversion from mechanical position to drive-field angle.

The firmware also checks that it can combine the measured stop clearance with
the calibrated range to produce a valid interval of allowed joint positions.
The clearance retained at each end of that interval is explained in step 7.

The final check compares two electrical drive-field angles:

- the field angle currently holding the motor; and
- the field angle that the saved calibration predicts for the measured encoder
  position.

These are electrical angles, not physical shaft angles. In the configured
50-pole-pair motors, the electrical field repeats 50 times during one mechanical
shaft revolution. One degree of shaft rotation therefore corresponds to 50
electrical degrees. Because the pattern repeats, field angles separated by a
complete electrical turn are equivalent. The firmware compares the smallest
equivalent difference. If its magnitude is greater than 75 electrical degrees,
feedback restart is refused rather than allowing a large, sudden correction.

## 6. Restarting normal feedback

For a joint that passes every check, the feedback loop starts from the final
measured position. The firmware initializes its stored position and speed
history from that measurement. It also initializes the controller output so the
first feedback update continues from the drive field that is already holding
the motor.

The aim is a continuous handover: the homing code and the normal feedback loop
agree about both the motor's starting position and the field holding it. This
avoids applying an old target or abruptly switching to an unrelated field
angle.

If a joint fails a required check, it remains unhomed and its feedback updates
remain disabled. The held field stays powered to avoid another abrupt
transition; `M18` can be used to disable the motor outputs. `G28` reports an
error, and normal `G0`, `G1`, and `G24` motion remains blocked until every
joint has been successfully homed and has a valid record of its measured stop.

## 7. Creating the software travel limits

A successful Home provides a measured distance between the final position and
the physical stop. Normal motion may use 75 percent of that distance. The
remaining 25 percent is kept as clearance between the nearest allowed target
and the stop.

Homing measures only one of the two physical ends of each joint's travel. At the
opposite end, the firmware keeps a fixed 0.5-degree margin inside the range covered
by the saved calibration.

These are software command limits, not independent physical limit switches.
They prevent the motion system from knowingly requesting a position outside the
trusted range. They cannot protect against incorrect calibration, incorrect
machine geometry, a loose mechanism, or an unexpected obstruction inside that
range.

## What a successful Home means

A successful `G28` means that every requested joint:

1. reached a detected mechanical stop;
2. backed away by encoder-confirmed movement;
3. remained stable after normal drive strength was restored;
4. ended inside the calibrated and Home-derived travel range;
5. passed the electrical-phase check; and
6. entered feedback control from the same final measured position.

Only after those steps does the controller report `ok`. Normal X, Y, and Z
motion becomes available when every joint—not only the joints selected by a
partial `G28` command—has been successfully homed and has a valid measured-stop
record.
