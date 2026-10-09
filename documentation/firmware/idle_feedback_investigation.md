# Idle runaway investigation — 9 October 2026

The user reported increasing noise and vibration followed by unexpected
large Y-associated joint motion while the stage was idle. Initial tests kept
motor outputs disabled. After connector repairs passed communication checks,
the investigation resumed Home, protected motion, and initially unsaved
calibration comparisons. Those comparisons justified fresh calibrations for
joints 2/3, which were subsequently saved with the user's approval.

## What the live tests established

The optional normal-timing feedback diagnostic enables encoder CRC checks
without changing current limits, PID gains, or SPI speed (8 MHz).
After flashing it, startup reported
`Failed to initialize encoder for joint 1`. This is zero-based Joint 1,
encoder 2, the user’s Y-associated joint. That message comes from the
encoder’s register write/read connection test, before calibration loading.

After `M18`, `M57` reported PWM amplitude zero on every joint. Encoder 2 had
2,549 CRC errors in the initial status reading; encoders 1 and 3 had zero.
Later, eight snapshots separated by two-second waits gave these cumulative
CRC counts for encoder 2:

```text
475501, 499783, 520593, 542743, 567647, 595172, 628306, 664655
```

At snapshots two and six its accumulated encoder angle reported exactly
zero, instead of roughly −132 encoder degrees. The associated lookup-table
position changed from roughly 10.53 to 5.94 motor-shaft degrees, with the
motor outputs still off. This is an observed feedback discontinuity, not
evidence of an actual shaft movement of that size.

Next, standalone encoder-only firmware compared communication speeds.
Both builds leave the shared motor enable line off, use the same encoder
reader/reporting code, sample at roughly two-millisecond intervals, and
periodically check register communication. The sole intended build difference
is the SPI clock.

| Run, in order | SPI clock | Reports collected | CRC errors, E1/E2/E3 | E2 reports showing failed register check |
| --- | --- | --- | --- | --- |
| First | 8 MHz | 120 | 0 / 4,015 / 0 | 110 |
| Second | 1 MHz | 119 | 0 / 0 / 0 | 0 |
| Third | 8 MHz | 120 | 0 / 4,770 / 0 | 120 |

Each collection lasted approximately 12 seconds. Error totals sum the
per-report increments; they are not percentages of all transactions.
Register-check status is repeated between checks, so failed-report totals
are not the number of independent failed register transactions.
At 8 MHz, E2 alternated between raw values zero and −1 and status zero or
`0xF`; the latter includes all three device error bits and the CRC-error bit.
At 1 MHz, E2 reported roughly −766,100 accumulated counts and no errors.
No encoder-only test validates behavior under powered motion or prolonged
idle feedback.

## Interpretation and limits

After the user reseated encoder 2's connector and reconnected USB and power,
another 30-second collection at 8 MHz produced 299 reports. All three encoders
showed `id=FAIL` in every report. CRC-error totals were 98, 98, and 99 for
E1/E2/E3 respectively; printed angle readings were predominantly zero.
This differs from the earlier encoder-2-only failure. Motor outputs remained
disabled. A shared supply or SPI connection problem must now be considered;
the readings do not identify which one. At that stage, the requested check was
encoder supply voltage measured between encoder VCC and encoder GND,
distinct from the motor-driver supply.

Unreliable feedback on encoder 2 is demonstrated under the tested 8 MHz
conditions, independently of motor-drive noise: the motor outputs were off.
The first 1 MHz test passed, and returning to 8 MHz reproduced failure.
This supports a communication-speed-sensitive problem, but cannot identify
a wire, connector, electrical interface, module, or firmware timing problem.
The user also reported that encoder 2’s cable/connector may have moved out of
place. Any physical connection change during comparisons would confound them.

The current CRC calculation accepts zero angle/status/CRC data as valid.
Thus CRC alone cannot prevent a zero-filled reply becoming false feedback.
The optional diagnostic's additional position-error/range checks are needed
for that case. The zero readings and communication errors can plausibly drive
the existing servo into large corrections, but we have not recorded the
original runaway from its onset, and have not ruled out additional control
instability or mechanical changes.

Normal encoder initialization logs a failed connection check but continues
initializing the joint. Normal servo updates also lack the new diagnostic's
continuous shutdown checks. These are relevant software gaps, not proof of
which event initiated the reported runaway.

## Powered tests after the second connector repair

A 20-second encoder-only test at 8 MHz collected 200 reports with no CRC,
device-status, or register-check errors on any channel. The protected recorder
was then flashed. Encoder startup checks and a three-second disabled-motor
preflight passed, followed by normal all-joint Home using the saved tables.

The two-minute Home-position hold passed with no encoder errors or shutdown.
Maximum position errors in half-second status snapshots were 0.002683,
0.004406, and 0.001964 motor-shaft degrees for joints 1/2/3. Final half-second
high-rate traces showed mean PID field corrections of +1.004, −70.830, and
−58.609 electrical degrees respectively. Thus accurate position tracking did
not mean there was adequate remaining field-correction margin.

The +1 mm X move and return passed. The +1 mm Y move was accepted, but its
return triggered fault 5: correction stayed at least 78 electrical degrees
for 100 ms. All three PWM amplitudes became zero and homing was invalidated.
The frozen Y trace had no encoder status errors, maximum position error
0.020728 motor-shaft degrees, and peak correction magnitude 80.456 electrical
degrees. This was the conservative saturation diagnostic interrupting a
closely tracked move, not a reproduction of the original idle runaway.
No thresholds were weakened in response.

Using `servo_reference_diagnostic`, `M56 J1` then measured fresh Y-associated
calibration tables in RAM, without `S`. The algorithm, calibration speed,
current limits, gains, and files on flash remained unchanged. Across 256
common raw-encoder positions, fresh-minus-saved field references had mean
−68.601570 electrical degrees, standard deviation 3.199302, and range
−75.986732..−63.538464 degrees. Fresh-minus-saved calibrated mechanical
positions averaged +0.166848 motor-shaft degrees.

Home with the fresh Y table passed. Its initial field handover changed from
−66.248520 electrical degrees in the saved-table run to +2.990503 degrees;
stationary correction fell from roughly −71 to +3.9 electrical degrees.
All X/Y/Z +1 mm moves and returns then passed, followed by a +2 mm Y move.
Both two-minute holds passed with no encoder errors or diagnostic shutdown.
The largest half-second sampled position error across those holds was
0.002932 motor-shaft degrees. At the Y-offset pose, the Y correction was about
+3 electrical degrees, while joint 3 still needed about −65 degrees with its
saved table. A separate unsaved joint-3 comparison was therefore performed.

`M56 J2` measured fresh joint-3 tables, again without saving. At 256 common
encoder positions, fresh-minus-saved field references averaged −59.626759
electrical degrees, standard deviation 1.128769, range
−62.616661..−57.226276 degrees. Mechanical position differences averaged
−0.014317 motor-shaft degrees. Another all-joint Home passed with initial
handovers +3.059324/+2.363915 electrical degrees for joints 2/3. Two one-minute
holds and all +1 mm X/Y/Z moves, returns, and a +2 mm Y move then passed.
CRC/status counts and diagnostic fault numbers remained zero.

Mean stationary PID corrections in the final half-second Home-position
traces were:

| Joint (one-based) | With saved tables, electrical degrees | With fresh joints 2/3 tables, electrical degrees |
| --- | --- | --- |
| 1 | +1.004 | +1.157 |
| 2, Y-associated | −70.830 | +4.022 |
| 3 | −58.609 | +2.096 |

At the +2 mm Y pose, mean corrections were −4.357/+2.977/−3.789 electrical
degrees. Gains, PWM caps, the calibration algorithm, and diagnostic thresholds
were identical in all these comparisons. Joint 1's table was not replaced.

A final run repeated Home and the same +1 mm X/Y/Z moves, returns, and
+2 mm Y move at 1 mm/s rather than 0.5 mm/s. It also passed, including two
15-second holds, with zero encoder errors and no diagnostic shutdown. The
largest position error in its final half-second traces was 0.005973
motor-shaft degrees. This is sampled encoder evidence, not independent stage
displacement measurement or an audible-noise observation.

The measured table differences and improved correction margin establish a
current mismatch between the saved calibration of joints 2/3 and their present
behavior.
They do not establish when or why the relationship changed, or prove that
the original runaway was caused solely by calibration. Previously observed
communication corruption remains an independently demonstrated problem.
Raw serial logs and trace CSVs from these tests are in temporary directories
`/tmp/micro-stage-idle-egol03ke` (saved-table test) and
`/tmp/micro-stage-idle-_v3fo1m9` (fresh Y test), and
`/tmp/micro-stage-idle-v1c2d26d` (fresh joints 2/3 test), and
`/tmp/micro-stage-idle-asyvfek7` (1 mm/s confirmation). These are not durable
repository artifacts.

## Saving the verified calibration references

With the user's approval, `M56 J1 S` and `M56 J2 S` repeated the measurements
and saved joints 2/3 individually. Both measurements completed with zero
accumulated encoder CRC errors and clear final encoder status, and both pairs
of calibration files were written successfully. Joint 1's saved tables were
not replaced. The diagnostic now
rejects a CRC/status-contaminated calibration before saving, removes the
selected joint's power and readiness, and reports failure; host regression
tests cover this guard. Normal calibration behavior is unchanged.
This check does not record every transient non-CRC device-status error during
the open-loop sweep, so a clear final status cannot prove every sample valid.

The saved-table run passed Home, +1 mm X/Y/Z moves and returns, and a +2 mm
Y move at 1 mm/s, with two 30-second powered holds. Encoder CRC/status and
diagnostic fault counts stayed zero. Logs and traces are in
`/tmp/micro-stage-idle-tl1z5_eg`. These temporary artifacts are not durable
repository records.

The same protected firmware was then re-uploaded to reboot the controller
without replacing its calibration filesystem. Startup loaded all three pairs
of tables from flash. With no further calibration command, Home, the same
1 mm/s moves, and two further 30-second holds passed. All encoder CRC/status
counts and fault numbers remained zero. Initial Home handover differences for
joints 2/3 were +2.978127/+2.627273 electrical degrees, confirming that the
fresh references survived the restart rather than reverting to the old large
differences. The final Home-position traces had peak correction magnitudes
1.442/3.694/2.337 electrical degrees for joints 1/2/3. The largest position
error in the captured hold traces was 0.005317 motor-shaft degrees. This
verification is in `/tmp/micro-stage-idle-dr5hi896`.

## Current state and remaining limits

The device retains the `servo_reference_diagnostic` firmware used for the
saved-table verification, with the verified, saved joint-2/3 calibrations and
unchanged joint-1 tables. The final `M18` succeeded: motor outputs were disabled
and the serial port released. No gain change, normal-production shutdown
change, or SPI-speed change is included in this work.
Recalibration is not a remedy for corrupted encoder replies, even though it
corrected the separately demonstrated reference mismatch after communication
was repaired.

Final source cleanup moved the encoder regression fixture out of its shell
runner and made diagnostic error-duration timers reset on power/feedback
pause or restart. This prevents disabled time from being counted as continuous
powered error. Regression tests cover this, invalid numeric inputs and refusal
to re-enable a latched joint. All five host runners passed, as did builds for
`pico`, `servo_idle_diagnostic`, and `servo_reference_diagnostic`. Existing
PID initialization-order and unused-variable warnings remain outside this
change. The cleanup was checked offline; no further upload or device command
was performed, so the timer-reset refinement is not yet flashed.

The original runaway was not reproduced after connector repair. This is not
proof against intermittent communication, slip, or instability. Before
promoting protection into normal firmware, distinguish mandatory invalid
feedback shutdown from the conservative diagnostic saturation threshold:
the latter interrupted a closely tracked move with the old tables. Do not
silently treat its thresholds as validated limits for every normal motion.

The recorder's commands, column units, fault thresholds, and limitations are
documented in [the firmware README](../../firmware/README.md#idle-feedback-fault-recorder).
