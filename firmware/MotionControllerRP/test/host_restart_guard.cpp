// Mock robot/encoder/driver for the actual restart preflight method.
#include <cmath>
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <initializer_list>
#include "utilities/math_constants.h"
constexpr int NUM_JOINTS = 3;
#define LOG_DEBUG(...) ((void)0)
#define LOG_INFO(...) ((void)0)
void spin_lock_unsafe_blocking(int) {}
void spin_unlock_unsafe(int) {}
struct Encoder { int32_t raw = 100; int32_t read_abs_angle_raw() { return raw; } };
struct Lut { bool in_input_range(float raw) const { return raw >= 0 && raw <= 200; } };
struct Driver { float field = 0, amplitude = .3f; float get_field_angle() { return field; } float get_amplitude() { return amplitude; } };
struct Servo {
  Encoder encoder;
  Driver driver;
  Lut lut;
  float reference = 0;
  bool enabled = false;
  unsigned updates = 0;
  Encoder& get_encoder() { return encoder; }
  Driver& get_motor_driver() { return driver; }
  const Lut& get_enc_to_pos_lut() const { return lut; }
  float encoder_angle_to_motor_pos(int32_t) { return 0; }
  float motor_pos_to_field_angle(float) { return reference; }
  void set_motor_update_enabled(bool en) { enabled = en; updates++; }
};
struct Joint {
  bool is_homed = false, is_calibrated = true;
  Servo servo;
  Servo* servo_controller = &servo;
};
class Robot {
public:
  Joint storage[3];
  Joint* joints[3] = {&storage[0], &storage[1], &storage[2]};
  int joints_spin_lock = 0;
  bool enable_servo_control(bool);
};
int main() {
  Robot powered_off;
  powered_off.storage[1].is_homed = true;
  powered_off.storage[1].servo.driver.amplitude = 0;
  powered_off.enable_servo_control(true);
  assert(!powered_off.storage[1].servo.enabled);
  for(float mismatch: {0.0f, 39.0f, 75.0f, -75.0f, 76.0f, -76.0f, 167.0f, 350.0f}) {
    Robot robot;
    robot.storage[1].is_homed = true;
    robot.storage[1].servo.reference = mismatch * Constants::DEG2RAD;
    bool accepted = robot.enable_servo_control(true);
    #ifdef HOMING_RESTART_GUARD
      bool expected = fabsf(remainderf(mismatch*Constants::DEG2RAD, Constants::TWO_PI_F)) <= 75*Constants::DEG2RAD;
    #else
      bool expected = true;
    #endif
    assert(robot.storage[1].servo.enabled == expected);
    assert(accepted == expected);
    assert(robot.storage[1].is_homed == expected);
    assert(!robot.storage[0].servo.enabled && !robot.storage[2].servo.enabled);
    robot.enable_servo_control(false);
    assert(!robot.storage[1].servo.enabled);
  }
  Robot outside;
  outside.storage[1].is_homed = true;
  outside.storage[1].servo.encoder.raw = 201;
  bool outside_accepted = outside.enable_servo_control(true);
  Robot nonfinite;
  nonfinite.storage[1].is_homed = true;
  nonfinite.storage[1].servo.reference = std::numeric_limits<float>::quiet_NaN();
  bool nonfinite_accepted = nonfinite.enable_servo_control(true);
  #ifdef HOMING_RESTART_GUARD
    assert(!outside.storage[1].servo.enabled && !nonfinite.storage[1].servo.enabled);
    assert(!outside_accepted && !nonfinite_accepted);
    assert(!outside.storage[1].is_homed && !nonfinite.storage[1].is_homed);
  #else
    assert(outside.storage[1].servo.enabled && nonfinite.storage[1].servo.enabled);
    assert(outside_accepted && nonfinite_accepted);
  #endif
  Robot uncalibrated;
  uncalibrated.storage[1].is_homed = true;
  uncalibrated.storage[1].is_calibrated = false;
  uncalibrated.enable_servo_control(true);
  assert(!uncalibrated.storage[1].servo.enabled);
  puts("PASS: actual restart routine enforces configured range/phase/nonfinite guard, reports refusal and invalidates readiness.");
}
