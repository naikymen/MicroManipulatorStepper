// Deterministic host test for the temporary post-home field transition. The
// runner injects the production update/start methods; no hardware is accessed.
#define HOMING_TRANSITIONAL_FIELD_HANDOVER
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "hw_config.h"
#include "servo_control/pid.h"

struct Encoder {
  int32_t raw = 15000;
  uint8_t status = 0;
  int32_t read_abs_angle_raw() { return raw; }
  uint8_t get_status() { return status; }
};
struct Driver {
  float field = 0.0f, amplitude = .3f;
  unsigned writes = 0;
  float get_amplitude() { return amplitude; }
  float get_field_angle() { return field; }
  void set_field_angle(float value) { field=value; writes++; }
};
class ServoController {
public:
  struct HomingHandoverStatus {
    bool active=false, complete=false, failed=false;
    float initial_field_offset=0, field_offset=0, max_position_error=0, elapsed_s=0;
  };
  Encoder encoder;
  Driver motor_driver;
  PIDController pos_controller, velocity_controller;
  LowpassFilter velocity_lowpass;
  float motor_pos=0, motor_pos_prev=0, pos_error=0, velocity=0, output=0;
  float homing_field_origin_offset=0;
  HomingHandoverStatus homing_handover;
  float homing_handover_stable_s=0;
  bool encoder_update_enabled=true, motor_update_enabled=false;
  ServoController() {
    pos_controller.set_parameter(POS_KP, POS_KI, 0, Constants::PI_F*2, Constants::PI_F*.5f);
    velocity_controller.set_parameter(VEL_KP, VEL_KI, 0, Constants::PI_F*.45f, Constants::PI_F*.45f);
    velocity_lowpass.set_time_constant(VEL_LOWPASS_TC);
  }
  float encoder_angle_to_motor_pos(int32_t raw) { return float(raw)*1e-6f; }
  float motor_pos_to_field_angle(float pos) { return pos*50; }
  void update(float target_motor_pos, float dt, float one_over_dt);
  void start_homing_handover(float measured_position);
};

int main() {
  ServoController smooth;
  const float target = smooth.encoder_angle_to_motor_pos(smooth.encoder.raw);
  const float reference = smooth.motor_pos_to_field_angle(target);
  smooth.motor_driver.field = reference - 120*Constants::DEG2RAD;
  const float held = smooth.motor_driver.field;
  smooth.start_homing_handover(target);
  assert(smooth.homing_handover.active);
  smooth.update(target, .001f, 1000);
  assert(fabsf(smooth.motor_driver.field-held) < 1e-5f);
  for(int i=0; i<5000 && smooth.homing_handover.active; ++i)
    smooth.update(target, .001f, 1000);
  assert(smooth.homing_handover.complete && !smooth.homing_handover.failed);
  assert(smooth.homing_field_origin_offset == 0);
  assert(fabsf(smooth.motor_driver.field-reference) < 1e-4f);
  assert(smooth.homing_handover.elapsed_s > 2.1f && smooth.homing_handover.elapsed_s < 2.4f);

  ServoController excursion;
  float excursion_target = excursion.encoder_angle_to_motor_pos(excursion.encoder.raw);
  excursion.start_homing_handover(excursion_target);
  excursion.encoder.raw += 10000; // ~0.57 motor degrees in this mock
  excursion.update(excursion_target, .001f, 1000);
  assert(excursion.homing_handover.failed && !excursion.motor_update_enabled);
  assert(excursion.motor_driver.writes == 0);

  ServoController bad_encoder;
  float bad_target = bad_encoder.encoder_angle_to_motor_pos(bad_encoder.encoder.raw);
  bad_encoder.start_homing_handover(bad_target);
  bad_encoder.encoder.status = 8;
  bad_encoder.update(bad_target, .001f, 1000);
  assert(bad_encoder.homing_handover.failed && !bad_encoder.motor_update_enabled);

  ServoController timeout;
  float timeout_target = timeout.encoder_angle_to_motor_pos(timeout.encoder.raw);
  timeout.start_homing_handover(timeout_target);
  timeout.encoder.raw += 1000; // outside settle tolerance, inside excursion bound
  for(int i=0; i<6000 && timeout.homing_handover.active; ++i)
    timeout.update(timeout_target, .001f, 1000);
  assert(timeout.homing_handover.failed && !timeout.motor_update_enabled);

  puts("PASS: transitional handover preserves the first field, decays to calibrated control, and bounds excursion, encoder faults, and timeout.");
}
