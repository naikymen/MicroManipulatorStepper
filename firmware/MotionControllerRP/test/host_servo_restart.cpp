// Host mocks for the firmware restart regression. The runner injects the actual
// production update/restart methods; PID code is compiled directly from src.
#define HOMING_SERVO_PULSE_TEST
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "hw_config.h"
#include "servo_control/pid.h"
uint64_t clock_us = 1000;
uint64_t time_us_64() { return clock_us; }
struct Encoder {
  int32_t raw = 1000;
  int32_t read_abs_angle_raw() { return raw; }
  uint32_t get_rawcounts_per_rev() { return 2097152; }
};
struct Driver {
  float field = -0.2f;
  float amplitude = 0.3f;
  float get_amplitude() { return amplitude; }
  unsigned writes = 0;
  float get_field_angle() { return field; }
  void set_field_angle(float next) { field = next; writes++; }
};
class ServoController {
public:
#ifdef HOMING_SERVO_PULSE_US
  static constexpr uint64_t restart_pulse_duration_us = HOMING_SERVO_PULSE_US;
#else
  static constexpr uint64_t restart_pulse_duration_us = 50000;
#endif
  static constexpr uint64_t restart_pulse_sample_interval_us =
    restart_pulse_duration_us > 1000000 ? restart_pulse_duration_us / 10 : 100000;
  struct RestartPulseDiagnostic {
    struct Sample {
      uint32_t elapsed_us = 0;
      float raw_rotor_delta = 0, pos_error = 0, pid_output = 0, applied_field_delta = 0;
    };
    Sample samples[12]{};
    uint32_t sample_count = 0;
    bool captured = false, stopped = false, excursion_cutoff = false;
    uint64_t start_us = 0;
    uint32_t updates = 0;
    int32_t raw_start = 0;
    float target = 0, measured = 0, previous = 0, first_dt = 0;
    float first_velocity = 0, held_field = 0, reference_field = 0;
    float pid_output = 0, applied_field = 0, max_excursion = 0;
  } restart_pulse;
  Encoder encoder;
  Driver motor_driver;
  PIDController pos_controller, velocity_controller;
  LowpassFilter velocity_lowpass;
  bool encoder_update_enabled = true, motor_update_enabled = false;
  float motor_pos = -1, motor_pos_prev = -1, pos_error = 0, velocity = 0, output = 0;
  ServoController() {
    pos_controller.set_parameter(POS_KP, POS_KI, 0, Constants::PI_F*2, Constants::PI_F*0.5f);
    velocity_controller.set_parameter(VEL_KP, VEL_KI, 0, Constants::PI_F*0.45f, Constants::PI_F*0.45f);
    velocity_lowpass.set_time_constant(VEL_LOWPASS_TC);
  }
  float encoder_angle_to_motor_pos(int32_t raw) { return float(raw)*1e-6f; }
  float motor_pos_to_field_angle(float pos) { return pos*50; }
  float read_position() { return encoder_angle_to_motor_pos(encoder.read_abs_angle_raw()); }
  void update(float target_motor_pos, float dt, float one_over_dt);
  void set_motor_update_enabled(bool enable);
};
int main() {
  {
    ServoController off;
    off.motor_driver.amplitude = 0;
    off.set_motor_update_enabled(true);
    float held = off.motor_driver.field;
    for(int i=0; i<10000; i++) {
      off.encoder.raw = 1000+i;
      off.update(1.0f, .0001f, 10000);
    }
    assert(off.motor_driver.writes == 0 && off.motor_driver.field == held);
    assert(off.motor_pos == off.read_position() && off.motor_pos_prev == off.motor_pos);
    assert(off.output == 0 && off.velocity == 0 && !off.restart_pulse.captured);
    assert(off.pos_controller.compute(0,.0001f,10000) == 0);
    assert(off.velocity_controller.compute(0,.0001f,10000) == 0);
    puts("PASS: zero-amplitude motors retain their field; encoder tracking continues without PID windup.");
  }
  ServoController timed;
  timed.set_motor_update_enabled(true);
  assert(timed.motor_pos == timed.motor_pos_prev);
  float target = timed.motor_pos;
  timed.update(target, 0.0001f, 10000);
  assert(timed.restart_pulse.captured && timed.motor_driver.writes == 1);
  assert(timed.restart_pulse.held_field == -0.2f);
  assert(timed.restart_pulse.applied_field == timed.motor_driver.field);
  #ifdef HOMING_BUMPLESS_SERVO_RESTART
  assert(std::abs(timed.restart_pulse.pid_output + 0.25f) < 1e-6f);
  assert(std::abs(timed.restart_pulse.applied_field - timed.restart_pulse.held_field) < 1e-6f);
#else
  assert(timed.restart_pulse.pid_output == 0);
#endif
  assert(timed.restart_pulse.first_velocity == 0);
  clock_us = 1000 + ServoController::restart_pulse_duration_us - 1;
  timed.update(target, 0.0001f, 10000);
  assert(timed.motor_driver.writes == 2 && !timed.restart_pulse.stopped);
  clock_us = 1000 + ServoController::restart_pulse_duration_us;
  timed.update(target, 0.0001f, 10000);
  assert(timed.restart_pulse.stopped && !timed.motor_update_enabled);
  assert(timed.motor_driver.writes == 2);
  assert(timed.restart_pulse.samples[0].elapsed_us == 0);
  assert(timed.restart_pulse.samples[timed.restart_pulse.sample_count-1].elapsed_us == ServoController::restart_pulse_duration_us);
  float held = timed.motor_driver.field;
  timed.set_motor_update_enabled(false);
  assert(timed.restart_pulse.captured && timed.motor_driver.field == held);
  timed.update(target, 0.0001f, 10000);
  assert(timed.motor_driver.writes == 2);

  ServoController excursion;
  clock_us = 100000;
  excursion.set_motor_update_enabled(true);
  target = excursion.motor_pos;
  excursion.update(target, 0.0001f, 10000);
  float radians_per_count = Constants::TWO_PI_F / float(excursion.encoder.get_rawcounts_per_rev()) *
                            ENCODER_ANGLE_TO_ROTOR_ANGLE;
  excursion.encoder.raw += int32_t(std::ceil(2.0f*Constants::DEG2RAD / radians_per_count)) + 1;
  clock_us += 100;
  excursion.update(target, 0.0001f, 10000);
  assert(excursion.restart_pulse.stopped && excursion.restart_pulse.excursion_cutoff);
  assert(excursion.motor_driver.writes == 1);
  assert(excursion.restart_pulse.max_excursion >= 2.0f*Constants::DEG2RAD);
  excursion.set_motor_update_enabled(true);
  assert(!excursion.restart_pulse.captured && !excursion.restart_pulse.stopped);
  assert(excursion.restart_pulse.sample_count == 0);
#ifdef HOMING_BUMPLESS_SERVO_RESTART
  ServoController limited;
  limited.motor_driver.field = limited.motor_pos_to_field_angle(limited.read_position()) -
                               83.692795f*Constants::DEG2RAD;
  limited.set_motor_update_enabled(true);
  limited.update(limited.motor_pos, 0.0001f, 10000);
  assert(std::abs(limited.restart_pulse.pid_output + 81.0f*Constants::DEG2RAD) < 1e-6f);
  float jump_deg = (limited.restart_pulse.applied_field-limited.restart_pulse.held_field)*Constants::RAD2DEG;
  assert(std::abs(jump_deg - 2.692795f) < 0.0001f);

  ServoController wrapped;
  wrapped.motor_driver.field = -0.2f + Constants::TWO_PI_F*7;
  wrapped.set_motor_update_enabled(true);
  wrapped.update(wrapped.motor_pos, 0.0001f, 10000);
  assert(std::abs(remainderf(wrapped.restart_pulse.applied_field-wrapped.restart_pulse.held_field,
                            Constants::TWO_PI_F)) < 1e-5f);
  std::puts("PASS: held-field preload retains first field, respects 81-degree integral limit, handles phase wrapping.");
#endif
  ServoController traced;
  clock_us = 1000;
  traced.set_motor_update_enabled(true);
  target = traced.motor_pos;
  traced.update(target, 0.0001f, 10000);
  for(uint64_t elapsed=100000; elapsed < ServoController::restart_pulse_duration_us; elapsed+=100000) {
    clock_us = 1000 + elapsed;
    traced.update(target, 0.0001f, 10000);
  }
  clock_us = 1000 + ServoController::restart_pulse_duration_us;
  traced.update(target, 0.0001f, 10000);
  assert(traced.restart_pulse.sample_count <= 12);
  unsigned trace_count = traced.restart_pulse.sample_count;
  if(ServoController::restart_pulse_duration_us >= 1000000) assert(trace_count == 11);
  traced.update(target, 0.0001f, 10000);
  assert(traced.restart_pulse.sample_count == trace_count);
  std::puts("PASS: trace captures first/last samples, respects cadence/capacity, stays frozen after cutoff, clears on rearm.");
  std::puts("PASS: actual servo methods capture first field, refresh history, enforce duration/2-degree cutoffs, retain held field, rearm.");
}
