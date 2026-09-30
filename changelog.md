# Changelog

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
