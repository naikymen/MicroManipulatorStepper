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
a separate setup procedure started with `M56`. It first drives the motor gently
against the physical stop. It then backs away until the raw encoder reading
confirms 1.5 degrees of motor-shaft movement. This backoff does not use a saved
lookup table because calibration is about to create a new one. The motor stays
at the lower homing drive strength during this move and returns to its normal
drive strength only after the clearance has been measured.

From that cleared position, calibration slowly scans a predetermined angle
range in both directions while recording the encoder reading and drive-field
angle at many points along the path.

From those measurements, the firmware builds two "lookup tables". The first
table converts an encoder reading into the joint's mechanical position,
measured from the cleared calibration starting point. The second converts that
mechanical position into the drive-field angle that should hold the motor
there. When calibration is explicitly saved, these tables are stored on the
controller and loaded again at startup.

Normal `G28` homing uses the tables to interpret encoder readings and to
validate its final position and drive field. It does not repeat the calibration
sweep or rewrite the tables.

The machine has no electrical limit switches. It finds Home by driving each
selected joint toward its mechanical stop and detecting when the encoder no
longer follows the drive field (that is, when the motor stalls).

## 1. Preparing to Home

When the controller receives `G28`, it discards any move that is running or
waiting to run. Homing is about to establish a new position reference, so an
older move must not resume afterward.

The firmware then pauses the normal feedback loops: a separate background task
usually reads the targets and controls the motors, so the firmware temporarily
prevents that task from reading or changing them while homing replaces the old
positions. This ensures that a position requested before homing cannot be sent
to the motors after homing finishes (as this could cause a sudden snapping move).

After a successful Home, the firmware stores three values for each joint: the
motor position where backoff ended, the measured distance from that position to
the physical stop, and the direction that moves away from the stop. Together,
these values let the firmware estimate where the stop is and set the nearest
allowed motor position.

At the start of a new Home, the selected joint is marked unhomed. The firmware
also invalidates the previously stored backoff-end position, measured clearance
from the stop, and direction away from the stop. A new search and backoff will
measure and replace this information. This does not delete or change the
joint's calibration tables.

The motor electronics are then enabled, their previous drive strength (motor
current) is remembered, and that strength is smoothly changed to the configured
homing value.

Normal position feedback remains paused during the search. Instead, the firmware
rotates the drive field steadily toward Home and watches how the encoder reacts.
All selected joints (up to three) perform this search together.

## 2. Detecting the physical stop

While rotating the drive field, the firmware repeatedly reads the encoder.
Before the joint reaches the stop, the motor and encoder should move by a
predictable amount, directly following the amount of field rotation.

At regular intervals, the firmware compares the encoder's actual movement with
the expected movement from the field rotation. If the encoder moved less than
five percent of the expected amount, the firmware concludes that the motor has
stalled against the physical stop because the encoder no longer follows the
field rotation.

Because a stalling event is almost instantaneous, this test requires a short
observation interval. The drive field can therefore continue advancing briefly
after the mechanism first touches the stop, even though the shaft no longer
moves.

The search fails if the encoder reports an error or if no stop is detected
within the allowed search distance. A failed search does not continue into a
normal backoff or claim that the joint is homed.

When the stop is detected, the firmware records the encoder's direct numerical
reading. It also resets the software count of complete encoder-angle
revolutions (not a count of motor-shaft revolutions). The encoder can only report
an angle within 0-360 degrees, while the software count records how many times
that angle crosses from its highest value back to zero, or in the opposite
direction. Combining the two lets the firmware measure continuous movement
away from the stop across that boundary. The reset only changes this temporary
position tracking; it does not alter the saved calibration.

## 3. Backing away from the stop

After the search is complete, each selected joint is backed away from its stop.
The firmware slowly rotates the drive field in the direction opposite to the homing direction, while continuing to read the encoder.

The backoff ends only when the encoder confirms both conditions:

- the motor shaft has moved far enough from the detected stop; and
- the current encoder reading is covered by the look up tables.

The normal requested clearance is 2.5 motor-shaft degrees. This is one degree
farther from the stop than calibration's starting point, so the resulting
encoder reading is clearly inside the saved table rather than on its first
sample. The firmware checks actual encoder movement rather than assuming that a
particular amount of drive-field rotation caused the same shaft movement.

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

## 4. Establishing the feedback-handover position

After all selected joints have backed away and settled, normal feedback is still
paused. The firmware now reads every encoder once and treats that set of
readings as the position where feedback will restart. This is not yet the
post-Home position used for normal Cartesian motion.

It uses those same readings to:

- set each joint's new target to its measured position;
- set each joint's requested speed to zero;
- calculate the stage's reported X, Y, and Z position;
- record the measured distance and direction from the physical stop; and
- initialize the feedback loops for their restart.

Using the same readings for every purpose prevents the target, reported stage
position, and feedback controller from starting at slightly different
positions.

An initial X, Y, and Z position is calculated from these measured joint
positions. The firmware does not calculate new joint positions back from that
result, because doing so could replace the encoder measurements with slightly
different calculated values. The reported Cartesian position is updated again
after the post-Home move in step 7.

## 5. Checking position and electrical phase

Before feedback can restart, the firmware checks that every ready joint has a
healthy encoder. Its measured position must be covered by both parts of the
saved calibration: the conversion from encoder reading to mechanical position,
and the conversion from mechanical position to drive-field angle.

The firmware also checks that it can combine the measured stop clearance with
the calibrated range to produce a valid interval of allowed joint positions.
The clearance retained at each end of that interval is explained in step 8.

The last check before feedback restarts compares two electrical drive-field
angles:

- the field angle currently holding the motor; and
- the field angle that the saved calibration predicts for the measured encoder
  position.

These are electrical angles, not physical shaft angles. In the configured
50-pole-pair motors, the electrical field repeats 50 times during one mechanical
shaft revolution. One degree of shaft rotation therefore corresponds to 50
electrical degrees. Because the pattern repeats, field angles separated by a
complete electrical turn are equivalent. The firmware compares the smallest
equivalent difference. If its magnitude is greater than 75 electrical degrees,
feedback restart is refused. The gradual transition described next does not
bypass this check or make an invalid calibration acceptable.

## 6. Restarting normal feedback

For a joint that passes every check, the feedback loop starts from the
measured position. The firmware initializes its stored position and speed
history from that measurement and makes that same position the target.

The field currently holding the motor can differ slightly from the field in the
saved calibration table. Switching between them in one update can produce a
snap even though the position target has not changed. To prevent that, the
firmware calculates their difference and temporarily adds it to the calibrated
field. The first feedback command therefore reproduces the field already being
applied.

The firmware then reduces this temporary difference at 60 electrical degrees
per second while keeping the fresh encoder position as the target. The
temporary value must reach exactly zero; it is not saved or retained as another
calibration offset. The motor has then returned to the ordinary relationship
between encoder position and calibrated drive field.

During this transition, the encoder must remain healthy and the measured shaft
position must stay within 0.5 motor-shaft degrees of the target. After the
temporary difference reaches zero, the position must remain within 0.05
motor-shaft degrees for 200 milliseconds. The transition fails if these limits
are violated or if it does not finish within five seconds.

If a joint fails a required check, it remains unhomed and its feedback updates
remain disabled. The held field stays powered to avoid another abrupt
transition; `M18` can be used to disable the motor outputs. `G28` reports an
error, and normal `G0`, `G1`, and `G24` motion remains blocked until every
joint has been successfully homed and has a valid record of its measured stop.

## 7. Moving to the normal post-Home position

The 2.5-degree backoff only has to clear the stop and enter the calibrated
range. It is not a useful starting pose for Cartesian motion. Near that end of
the range, a 1 mm X, Y, or Z move can require one of the three coupled joints to
move about six motor-shaft degrees toward the stop.

After the field transition succeeds, the firmware therefore moves every
selected joint smoothly to 7 motor-shaft degrees in the calibrated coordinate
system. The requested joint target changes at no more than 3 degrees per
second. The move is accepted only if 7 degrees is inside that joint's saved
calibration and its Home-derived travel interval.

The encoder is monitored throughout this move. Home fails if an encoder reports
an error, if a joint falls more than 1 degree behind its changing target, or if
the final target cannot be reached within five seconds. Each joint must remain
within 0.05 degrees of exactly 7 degrees for 200 milliseconds before Home can
finish. The controller then calculates the reported X, Y, and Z position from
that exact final joint target.

If this move fails, the selected joints are marked unhomed and their motor
outputs are disabled. The controller reports that Home failed, so normal motion
cannot begin from an uncertain position.

With the current machine geometry, the six separate 1 mm moves in the positive
and negative X, Y, and Z directions require joint positions between about 1.82
and 11.85 degrees when starting from this position. Those values remain inside
the calibrated range. The controller still executes the requested Cartesian
move exactly or rejects it; it never shortens the move to fit the limits.

## 8. Creating the software travel limits

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
4. ended its measured backoff inside the calibrated and Home-derived travel range;
5. passed the electrical-phase check; and
6. entered feedback control from the same final measured position;
7. removed the temporary field difference without exceeding the monitored
   position error; and
8. reached and settled at the calibrated 7-degree post-Home target.

Only after those steps does the controller report `ok`. Normal X, Y, and Z
motion becomes available when every joint—not only the joints selected by a
partial `G28` command—has been successfully homed and has a valid measured-stop
record.
