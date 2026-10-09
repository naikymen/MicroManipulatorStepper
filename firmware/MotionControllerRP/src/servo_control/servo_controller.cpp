// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include "hardware/timer.h"
#include "pico/stdlib.h"

#include "servo_controller.h"
#include "utilities/logging.h"
#include "utilities/math_constants.h"

#include "hw_config.h"

#include <algorithm>

ServoController::ServoController(
  MOTOR_DRIVER_TYPE& motor_driver, 
  ENCODER_TYPE& encoder, 
  int32_t motor_pole_pair_count) :
    motor_driver(motor_driver),
    encoder(encoder)
{
  ServoController::motor_pole_pair_count = motor_pole_pair_count;
  ServoController::motor_update_enabled = false;
  ServoController::encoder_update_enabled = true;
  ServoController::motor_pos = 0.0f;
  ServoController::pos_error = 0.0f;

  // set default encoder lut
  using namespace Constants;
  float magnet_array_radius = 30.0f; // mm
  float magnet_pitch = 3.0f;         // mm
  float g = float(encoder.get_rawcounts_per_rev())*(TWO_PI_F*magnet_array_radius/magnet_pitch)*0.5f;
  build_linear_lut(encoder_raw_to_motor_pos_lut, -g, g, -TWO_PI_F, TWO_PI_F);
}

void ServoController::init(float max_motor_amplitude) {
  ServoController::motor_current_amplitude = max_motor_amplitude;

  // setup motor driver
  motor_driver.begin();
  motor_driver.set_amplitude(0.0f, true); // correct amplitude will be set by 'set_motor_enabled()' 
  // begin() leaves the shared standby line low. Keep it there until a motor is
  // explicitly enabled so the first motion gets a real standby-to-active edge.
  motor_driver.set_field_angle(0.0f);

  velocity_lowpass.set_time_constant(VEL_LOWPASS_TC);

  pos_controller.set_parameter(POS_KP, POS_KI, 0.0f, Constants::PI_F*2.0F, Constants::PI_F*0.5F);
  velocity_controller.set_parameter(VEL_KP, VEL_KI, 0.0f, Constants::PI_F*0.45f, Constants::PI_F*0.45f);

  // large 0.9° steppers
//  pos_controller.set_parameter(75.0f, 50000.0f, 0.0f, Constants::PI_F*2.0F, Constants::PI_F*0.5F);
//  velocity_controller.set_parameter(0.2f, 150.0f, 0.0f, Constants::PI_F*0.45f, Constants::PI_F*0.45f);

 // pos_controller.set_parameter(75.0f, 2000.0f, 0.0f, Constants::PI_F*2.0F, Constants::PI_F*0.5F);
 // velocity_controller.set_parameter(0.2f, 0.0f, 0.0f, Constants::PI_F*0.45f, Constants::PI_F*0.45f);
}

void ServoController::set_enc_to_pos_lut(LookupTable& lut) {
  ServoController::encoder_raw_to_motor_pos_lut = lut;
}

// get the motor position to field angle lookup table
const LookupTable& ServoController::get_enc_to_pos_lut() const {
  return encoder_raw_to_motor_pos_lut;
}

void ServoController::set_pos_to_field_lut(LookupTable& lut) {
  ServoController::motor_pos_to_field_angle_lut = lut;
}

// get the motor position to field angle lookup table
const LookupTable& ServoController::get_pos_to_field_lut() const {
  return motor_pos_to_field_angle_lut;
}


void ServoController::update(float target_motor_pos, float dt, float one_over_dt) {
  if(encoder_update_enabled == false)
    return;

  // read encoder
  int32_t encoder_angle_raw = encoder.read_abs_angle_raw();

  #ifdef SERVO_IDLE_DIAGNOSTIC
    if(feedback_fault != FeedbackFault::None) return;
    if(!motor_update_enabled || motor_driver.get_amplitude() == 0.0f)
      feedback_tracking_since = feedback_correction_since = 0;
    if(motor_update_enabled && motor_driver.get_amplitude() > 0.0f) {
      if(encoder.get_status() != 0) {
        trip_feedback(FeedbackFault::EncoderStatus, encoder_angle_raw, target_motor_pos, dt);
        return;
      }
      if(!encoder_raw_to_motor_pos_lut.in_input_range(encoder_angle_raw)) {
        trip_feedback(FeedbackFault::CalibrationRange, encoder_angle_raw, target_motor_pos, dt);
        return;
      }
      if(!std::isfinite(target_motor_pos) || !std::isfinite(dt) || dt <= 0.0f ||
         !std::isfinite(one_over_dt) || one_over_dt <= 0.0f) {
        trip_feedback(FeedbackFault::InvalidNumber, encoder_angle_raw, target_motor_pos, dt);
        return;
      }
    }
  #endif

  // convert encoder angle to motor pos using LUT and compute field angle
  motor_pos = encoder_angle_to_motor_pos(encoder_angle_raw);
  pos_error = target_motor_pos-motor_pos;
  if(motor_driver.get_amplitude() == 0.0f) {
    // Keep encoder tracking alive while off, but do not wind up the controllers
    // or advance a field that could pull the rotor on its next power-up.
    pos_controller.reset();
    velocity_controller.reset();
    velocity_lowpass.reset(0.0f);
    velocity = output = 0.0f;
    motor_pos_prev = motor_pos;
    return;
  }
  #ifdef HOMING_TRANSITIONAL_FIELD_HANDOVER
    if(homing_handover.active) {
      homing_handover.elapsed_s += dt;
      homing_handover.max_position_error =
        std::max(homing_handover.max_position_error, fabsf(pos_error));
      if(encoder.get_status() != 0 || !std::isfinite(pos_error) ||
         fabsf(pos_error) > HOMING_HANDOVER_MAX_POSITION_ERROR_DEG*Constants::DEG2RAD ||
         homing_handover.elapsed_s > HOMING_HANDOVER_TIMEOUT_MS*0.001f) {
        homing_handover.active = false;
        homing_handover.failed = true;
        motor_update_enabled = false;
        #ifdef SERVO_IDLE_DIAGNOSTIC
          trip_feedback(FeedbackFault::HandoverFailed, encoder_angle_raw, target_motor_pos, dt);
        #endif
        return;
      }
    }
  #endif

  float field_angle = motor_pos_to_field_angle(motor_pos);
  #ifdef HOMING_TRANSITIONAL_FIELD_HANDOVER
    field_angle += homing_field_origin_offset;
  #endif

  // position controll loop
  float velocity_target = pos_controller.compute(pos_error, dt, one_over_dt);

  // velocity controll loop
  float velocity_unfiltered = (motor_pos - motor_pos_prev)*one_over_dt;
  velocity = velocity_lowpass.update(velocity_unfiltered, dt);
  float torque_target = velocity_controller.compute(velocity_target-velocity, dt, one_over_dt);

  // torque controll loop
  output = torque_target;

  #ifdef SERVO_IDLE_DIAGNOSTIC
    if(motor_update_enabled) {
      if(!std::isfinite(motor_pos) || !std::isfinite(pos_error) ||
         !std::isfinite(velocity) || !std::isfinite(output) || !std::isfinite(field_angle)) {
        trip_feedback(FeedbackFault::InvalidNumber, encoder_angle_raw, target_motor_pos, dt);
        return;
      }
      uint64_t now = time_us_64();
      if(fabsf(pos_error) > 0.15f*Constants::DEG2RAD) {
        if(feedback_tracking_since == 0) feedback_tracking_since = now;
      } else feedback_tracking_since = 0;
      if(fabsf(output) >= 78.0f*Constants::DEG2RAD && !homing_handover.active) {
        if(feedback_correction_since == 0) feedback_correction_since = now;
      } else feedback_correction_since = 0;
      if(fabsf(pos_error) > 1.0f*Constants::DEG2RAD ||
         (feedback_tracking_since && now-feedback_tracking_since >= 50000)) {
        trip_feedback(FeedbackFault::TrackingError, encoder_angle_raw, target_motor_pos, dt);
        return;
      }
      if(feedback_correction_since && now-feedback_correction_since >= 100000) {
        trip_feedback(FeedbackFault::SustainedCorrection, encoder_angle_raw, target_motor_pos, dt);
        return;
      }
    }
  #endif

  // set new field direction
  // motor_driver.set_amplitude(std::clamp(abs(output*10.0f), 0.1f, 0.5f), false);
  if(motor_update_enabled) {
    #ifdef HOMING_SERVO_PULSE_TEST
      // Capture in memory only: serial logging here would disturb core 1 timing.
      uint64_t now_us = time_us_64();
      if(!restart_pulse.captured) {
        restart_pulse.captured = true;
        restart_pulse.start_us = now_us;
        restart_pulse.raw_start = encoder_angle_raw;
        restart_pulse.target = target_motor_pos;
        restart_pulse.measured = motor_pos;
        restart_pulse.previous = motor_pos_prev;
        restart_pulse.first_dt = dt;
        restart_pulse.first_velocity = velocity;
        restart_pulse.held_field = motor_driver.get_field_angle();
        restart_pulse.reference_field = field_angle;
        restart_pulse.pid_output = output;
        restart_pulse.applied_field = field_angle + output;
      }
      float raw_rotor_delta = float(int64_t(encoder_angle_raw)-restart_pulse.raw_start) *
                              (Constants::TWO_PI_F / float(encoder.get_rawcounts_per_rev())) *
                              ENCODER_ANGLE_TO_ROTOR_ANGLE;
      restart_pulse.max_excursion = std::max(restart_pulse.max_excursion, fabsf(raw_rotor_delta));
      restart_pulse.excursion_cutoff = fabsf(raw_rotor_delta) >= 2.0f*Constants::DEG2RAD;
      if(now_us-restart_pulse.start_us >= restart_pulse_duration_us || restart_pulse.excursion_cutoff) {
        motor_update_enabled = false;
        restart_pulse.stopped = true;
      }
      if(motor_update_enabled) {
        motor_driver.set_field_angle(field_angle + output);
        restart_pulse.updates++;
      }
      // Spread snapshots over the bounded window and reserve room for cutoff.
      uint32_t elapsed_us = uint32_t(now_us-restart_pulse.start_us);
      if(restart_pulse.sample_count < 12 &&
         (restart_pulse.sample_count == 0 || !motor_update_enabled ||
          elapsed_us-restart_pulse.samples[restart_pulse.sample_count-1].elapsed_us >= restart_pulse_sample_interval_us)) {
        auto& sample = restart_pulse.samples[restart_pulse.sample_count++];
        sample.elapsed_us = elapsed_us;
        sample.raw_rotor_delta = raw_rotor_delta;
        sample.pos_error = pos_error;
        sample.pid_output = output;
        sample.applied_field_delta = remainderf(motor_driver.get_field_angle() -
                                                restart_pulse.held_field, Constants::TWO_PI_F);
      }
    #else
      motor_driver.set_field_angle(field_angle + output);
    #endif
  } 

  #ifdef HOMING_TRANSITIONAL_FIELD_HANDOVER
    if(homing_handover.active) {
      const float step = HOMING_HANDOVER_FIELD_RATE_DEG_S*Constants::DEG2RAD*dt;
      if(fabsf(homing_field_origin_offset) <= step)
        homing_field_origin_offset = 0.0f;
      else
        homing_field_origin_offset += homing_field_origin_offset > 0.0f ? -step : step;
      homing_handover.field_offset = homing_field_origin_offset;

      if(homing_field_origin_offset == 0.0f &&
         fabsf(pos_error) <= HOMING_HANDOVER_SETTLE_TOLERANCE_DEG*Constants::DEG2RAD) {
        homing_handover_stable_s += dt;
        if(homing_handover_stable_s >= HOMING_HANDOVER_SETTLE_MS*0.001f) {
          homing_handover.active = false;
          homing_handover.complete = true;
        }
      } else {
        homing_handover_stable_s = 0.0f;
      }
    }
  #endif

  // store values for next update
  motor_pos_prev = motor_pos;
  #ifdef SERVO_IDLE_DIAGNOSTIC
    if(motor_update_enabled) record_feedback(encoder_angle_raw, target_motor_pos, dt);
  #endif
}

#ifdef SERVO_IDLE_DIAGNOSTIC
void ServoController::record_feedback(int32_t raw, float target, float dt, bool force) {
  if(feedback_trace_frozen) return;
  uint64_t now = time_us_64();
  if(!force && now-feedback_last_sample_us < 1000) return;
  feedback_last_sample_us = now;
  auto& sample = feedback_trace[feedback_trace_next];
  sample = FeedbackSample{now, raw, encoder.get_status(), target, motor_pos,
                          velocity, output, motor_driver.get_field_angle(), dt};
  feedback_trace_next = (feedback_trace_next+1)%feedback_trace_capacity;
  feedback_trace_count = std::min(feedback_trace_count+1, feedback_trace_capacity);
}

void ServoController::stop_feedback_output() {
  feedback_trace_frozen = true;
  motor_update_enabled = false;
  motor_driver.set_amplitude(0.0f, true);
  pos_controller.reset();
  velocity_controller.reset();
}

void ServoController::trip_feedback(FeedbackFault reason, int32_t raw, float target, float dt) {
  record_feedback(raw, target, dt, true);
  feedback_fault = reason;
  stop_feedback_output();
}
#endif

bool ServoController::at_position(float motor_pos_eps) {
  return fabs(pos_error) < motor_pos_eps;
}

float ServoController::read_position() {
  return encoder_angle_to_motor_pos(encoder.read_abs_angle_raw());
}

float ServoController::get_position() {
  return motor_pos;
}

float ServoController::get_position_error() {
  return pos_error;
}

bool ServoController::move_to(float target_motor_pos, float at_pos_eps, float settle_time_s, float timeout_s) {
  uint64_t start_time_us = time_us_64();
  uint64_t time_us = start_time_us;
  uint64_t pos_reached_time_us = 0;
  uint64_t last_time = time_us;
  uint32_t settle_time_us = settle_time_s*1e6f;
  uint32_t timeout_us = timeout_s*1e6f;

  do {
    // get time and detla time
    time_us = time_us_64();
    float dt = float(time_us - last_time)*1e-6f;
    last_time = time_us;

    float pos_error;
    update(target_motor_pos, dt, 1.0f/dt);

    // check if traget position reached
    if(pos_reached_time_us == 0) {
      if(at_position(at_pos_eps))
        pos_reached_time_us = time_us;
    } else {
      if(time_us-pos_reached_time_us > settle_time_us)
        return true;
    }

  } while(time_us-start_time_us < timeout_us);

  return false;
}

void ServoController::move_to_open_loop(float delta_motor_pos, float motor_angular_velocity) {
  // Determine direction of movement at the start
  const bool moving_forward = delta_motor_pos > 0.0f;

  uint64_t last_time = time_us_64();
  float pos = 0.0f;
  while (fabs(pos) < delta_motor_pos)
  {
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time) * 1e-6f;
    last_time = time_us;

    // update encoder regularly
    if(encoder_update_enabled)
      encoder.read_abs_angle_raw();

    // update motor position
    pos += moving_forward ? motor_angular_velocity * dt : -motor_angular_velocity * dt;

    // set field ange to new position
    float clamped_motor_pos = moving_forward ? std::min(pos, delta_motor_pos) : 
                                               std::max(pos, -delta_motor_pos);
    motor_driver.set_field_angle(clamped_motor_pos*motor_pole_pair_count);
    sleep_us(100);
  }

  motor_pos += delta_motor_pos;
}

ServoController::ENCODER_TYPE& ServoController::get_encoder() {
  return encoder;
}

ServoController::MOTOR_DRIVER_TYPE& ServoController::get_motor_driver() {
  return motor_driver;
}

float ServoController::get_pole_pair_count() {
  return motor_pole_pair_count;
}

void ServoController::set_motor_enabled(bool enable, bool synchronize_field_angle) {
  #ifdef SERVO_IDLE_DIAGNOSTIC
    if(enable && feedback_trace_frozen) return;
    // Threshold durations count continuous powered feedback, not time off.
    feedback_tracking_since = feedback_correction_since = 0;
  #endif
  if(enable) {
    // All three drivers share the hardware enable pin. It may have been put in
    // standby by M18 or a diagnostic, so always reassert it before ramping PWM.
    motor_driver.enable();

    // synchronize field angle to motor_pos
    if(synchronize_field_angle) {
      float start_field_angle = motor_pos_to_field_angle(motor_pos);
      motor_driver.set_field_angle(start_field_angle);
    }

    motor_driver.set_amplitude_smooth(motor_current_amplitude, 100);
    pos_controller.reset();
    velocity_controller.reset();
    velocity_lowpass.reset(0.0f);
    motor_pos_prev = motor_pos;

  } else {
    motor_driver.set_amplitude_smooth(0.0f, 100);
  }
}

// enable or disable servo loop update and encoder reads
void ServoController::set_motor_update_enabled(bool enable, const float* measured_position) {
  #ifdef SERVO_IDLE_DIAGNOSTIC
    if(enable && feedback_trace_frozen) return;
    feedback_tracking_since = feedback_correction_since = 0;
  #endif
  if(enable) {
    // Open-loop homing may have moved the motor while servo updates were blocked.
    // Start velocity estimation from a fresh encoder position, not the old cache.
    motor_pos = measured_position ? *measured_position : read_position();
    #ifdef HOMING_SERVO_PULSE_TEST
      restart_pulse = RestartPulseDiagnostic{};
    #endif
  }
  pos_controller.reset();
  #ifdef HOMING_BUMPLESS_SERVO_RESTART
    // Diagnostic only: retaining this offset was shown to carry an arbitrary
    // homing stall phase into later cycles. Normal firmware resets the PID and
    // resumes from the calibrated encoder-to-field relationship below.
    float held_field_offset = enable ? remainderf(motor_driver.get_field_angle() -
                                                  motor_pos_to_field_angle(motor_pos),
                                                  Constants::TWO_PI_F) : 0.0f;
    velocity_controller.reset(held_field_offset);
  #else
    velocity_controller.reset();
  #endif
  velocity_lowpass.reset(0.0f);
  motor_pos_prev = motor_pos;
  motor_update_enabled = enable;
}

void ServoController::start_homing_handover(float measured_position) {
  #ifdef SERVO_IDLE_DIAGNOSTIC
    if(feedback_trace_frozen) return;
    feedback_tracking_since = feedback_correction_since = 0;
  #endif
  motor_pos = measured_position;
  motor_pos_prev = measured_position;
  pos_error = 0.0f;
  velocity = output = 0.0f;
  pos_controller.reset();
  velocity_controller.reset();
  velocity_lowpass.reset(0.0f);
  homing_field_origin_offset = remainderf(
    motor_driver.get_field_angle() - motor_pos_to_field_angle(measured_position),
    Constants::TWO_PI_F);
  homing_handover = HomingHandoverStatus{};
  homing_handover.active = true;
  homing_handover.initial_field_offset = homing_field_origin_offset;
  homing_handover.field_offset = homing_field_origin_offset;
  homing_handover_stable_s = 0.0f;
  motor_update_enabled = true;
}

void ServoController::set_encoder_update_enabled(bool enable) {
  pos_controller.reset();
  velocity_controller.reset();
  motor_pos_prev = motor_pos;
  encoder_update_enabled = enable;
}

float ServoController::encoder_angle_to_motor_pos(int32_t encoder_angle_raw) {
  return encoder_raw_to_motor_pos_lut.evaluate(encoder_angle_raw);
}

float ServoController::motor_pos_to_field_angle(float motor_pos) {
  return motor_pos_to_field_angle_lut.evaluate(motor_pos);
}
