# Interactive Motor Step Test

This optional firmware mode tests each TB6612 output and motor independently of
the encoders, robot model, homing, and closed-loop controller. Use it to isolate
motor, driver, socket, or winding faults before calibration.

> **Caution:** The TB6612 does not measure or regulate winding current. The
> firmware only limits PWM duty. Use a current-limited bench supply, keep the
> mechanism clear, and be ready to remove motor power.

## Enable and flash

Build and flash the dedicated diagnostic environment from the repository root.
No source-file edit is required:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --environment motor_step_test --target upload
```

The separate PlatformIO environment prevents this mode and
`ENCODER_WIGGLE_TEST` from being enabled together.

Open the serial monitor at 921600 baud. The mode starts with all outputs
disabled and accepts one-character commands:

| Command | Action |
| --- | --- |
| `1`, `2`, `3` | Move that motor eight full steps forward and back using 1/16 microsteps. |
| `a` | Run the same movement on all three motors in sequence. |
| `p` | Hold motor 1 on phase A and then phase B for a torque check. |
| `b`, `c`, `d`, `e` | Hold motor 1 at B+, A+, A-, or B- for eight seconds while measuring its output. |
| `x` | Immediately set every amplitude to zero and put the shared driver enable in standby. |

The normal motion firmware does not run in this mode. Each movement uses a low
duty cap (15% for motors 1 and 2 and 14.5% for motor 3), ramps the output, and
disables the shared driver enable afterward.

After testing, restore normal operation by flashing the default environment:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run --environment pico --target upload
```
