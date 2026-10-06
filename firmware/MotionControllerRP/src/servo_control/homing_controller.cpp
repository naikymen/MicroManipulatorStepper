#include "homing_controller.h"
#include "utilities/math_constants.h"
#include "utilities/logging.h"
#include "pico/time.h"
#include "hw_config.h"
#include <algorithm>

#if defined(HOMING_RESTORE_BEFORE_BACKOFF_TEST) && defined(HOMING_TRANSITION_TEST)
  #error "Use the early-restore test separately from the staged transition test"
#endif

HomingController::HomingController() {
  // Electrical rad/s: slow the post-home backoff without changing its distance.
  retract_field_velocity = 10.0f;
  retract_field_angle = Constants::TWO_PI_F*0.25f;
}

bool HomingController::run_blocking(ServoController* servo_controller, 
                                    float motor_velocity, 
                                    float search_range_angle, 
                                    float current, 
                                    float encoder_angle_to_motor_angle,
                                    float retract_angle_rad)
{
  start(servo_controller, motor_velocity, search_range_angle, current, 
        encoder_angle_to_motor_angle, retract_angle_rad);

  while(is_finished() == false) {
    update();
  }

  finalize();
  return is_successful();
}

void HomingController::start(ServoController* servo_controller, 
                            float velocity, 
                            float search_range_angle, 
                            float current, 
                            float encoder_angle_to_motor_angle,
                            float retract_angle_rad,
                            bool restore_amplitude_before_backoff,
                            bool measured_backoff)
{
  restore_before_backoff = restore_amplitude_before_backoff;
  use_measured_backoff = measured_backoff;
  finalized = false;
  field_angle_offset = last_eval_field_angle_offset = 0.0f;
  search_failed = false;
  servo_ctrl = servo_controller;
  encoder_to_motor_angle = encoder_angle_to_motor_angle;
  float pole_pair_count = servo_controller->get_pole_pair_count();
  float field_angle_to_encoder_angle = 1.0f / pole_pair_count / encoder_angle_to_motor_angle;

  retract_field_angle = retract_angle_rad > 0.0f ? retract_angle_rad * pole_pair_count :
                        Constants::TWO_PI_F*0.25f;

  if(use_measured_backoff) {
    const auto& enc_lut = servo_ctrl->get_enc_to_pos_lut();
    const auto& field_lut = servo_ctrl->get_pos_to_field_lut();
    requested_clearance = retract_angle_rad;
    if(!std::isfinite(velocity) || velocity == 0.0f ||
       !std::isfinite(search_range_angle) || search_range_angle <= 0.0f ||
       !std::isfinite(pole_pair_count) || pole_pair_count <= 0.0f ||
       !std::isfinite(encoder_to_motor_angle) || encoder_to_motor_angle <= 0.0f ||
       !std::isfinite(requested_clearance) || requested_clearance <= 0.0f ||
       requested_clearance > HOMING_BACKOFF_MAX_CLEARANCE_DEG*Constants::DEG2RAD ||
       enc_lut.size() < 2 || field_lut.size() < 2 || !enc_lut.is_monotonic()) {
      LOG_ERROR("HOME BACKOFF: invalid parameters or calibration domain");
      search_failed = true;
      state = State::Failed;
      return;
    }
    float field_min, field_max;
    field_lut.get_intput_range(field_min, field_max);
    float a = enc_lut.get_entry(0), b = enc_lut.get_entry(enc_lut.size()-1);
    trusted_position_min = std::max(0.0f, std::max(std::min(a,b), field_min));
    trusted_position_max = std::min(CALIBRATION_RANGE*Constants::DEG2RAD,
                                    std::min(std::max(a,b), field_max));
    if(!std::isfinite(a) || !std::isfinite(b) || a == b ||
       !std::isfinite(field_min) || !std::isfinite(field_max) || field_min >= field_max ||
       !std::isfinite(trusted_position_min) || !std::isfinite(trusted_position_max) ||
       trusted_position_min >= trusted_position_max) {
      LOG_ERROR("HOME BACKOFF: invalid calibrated travel interval");
      search_failed = true;
      state = State::Failed;
      return;
    }
    // The LUT identifies encoder polarity. Do not guess it from a stalled
    // encoder at startup, and do not change the calibration's unwrap convention.
    encoder_backoff_direction = (b > a ? 1.0f : -1.0f) * (velocity < 0 ? 1.0f : -1.0f);
    away_from_stop_sign = velocity < 0 ? 1.0f : -1.0f;
  }

  // field_angle_to_rotor_angle = 1.0 / pole_pair_count
  // encoder_angle_to_rotor_angle = encoder_period_pitch/encoder_radius
  // field_angle_to_encoder_angle = field_angle_to_rotor_angle/encoder_angle_to_rotor_angle
  
  eval_field_angle_delta = Constants::TWO_PI_F*0.1f;
  expected_encoder_delta = eval_field_angle_delta * field_angle_to_encoder_angle;
  
  servo_ctrl = servo_controller;
  field_velocity = velocity * pole_pair_count;
  field_angle_search_range = search_range_angle * pole_pair_count;
  homing_current = current;

  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  // perform 'soft start'
  servo_ctrl->set_motor_enabled(true, false);

  initial_current = servo_ctrl->get_motor_driver().get_amplitude();
  servo_ctrl->get_motor_driver().set_amplitude_smooth(homing_current, 100);
  search_failed = false;

  // motor_driver.rotate_field(Constants::TWO_PI_F*0.5f * (field_velocity>0.0f ? -1.0f : 1.0f), 12.0f);
  start_field_angle = fmodf(motor_driver.get_field_angle(), Constants::TWO_PI_F);

  last_eval_encoder_angle = encoder.read_abs_angle();
  last_time = 0;

  state = State::Homing;
}

void HomingController::update() {
  if (state != State::Homing)
    return;

  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  uint64_t time_us = time_us_64();
  if(last_time == 0) last_time = time_us;
  float dt = float(time_us - last_time) * 1e-6f;
  last_time = time_us;

  // Move motor and read encoder
  field_angle_offset += field_velocity * dt;
  motor_driver.set_field_angle(start_field_angle+field_angle_offset);

  float encoder_angle = encoder.read_abs_angle();
  if(use_measured_backoff && encoder.get_status() != 0) {
    LOG_ERROR("HOME SEARCH: invalid encoder sample");
    search_failed = true;
    state = State::Failed;
    return;
  }

  if (fabs(last_eval_field_angle_offset - field_angle_offset) > eval_field_angle_delta) {
    float encoder_delta = encoder_angle - last_eval_encoder_angle;
    float encoder_velocity_ratio = encoder_delta / expected_encoder_delta;

    // LOG_DEBUG("encoder_delta=%f/ %f", encoder_delta, expected_encoder_delta);
    // LOG_DEBUG("encoder_velocity_ratio=%f", encoder_velocity_ratio);

    if (fabsf(encoder_velocity_ratio) < 0.05f) {
      LOG_DEBUG("End stop detected");
      on_endstop_detected();
      return;
    }

    last_eval_encoder_angle = encoder_angle;
    last_eval_field_angle_offset = field_angle_offset;
  }

  if (fabs(field_angle_offset) > field_angle_search_range) {
    LOG_INFO("End stop not detected");
    search_failed = true;
    state = State::Failed;
  }
}

void HomingController::on_endstop_detected() {
  state = State::Done;
  auto& encoder = servo_ctrl->get_encoder();

  if(search_failed) {
    servo_ctrl->set_motor_enabled(false, false);
    return;
  }

  // reset encoder period, the remainder will provide a very repeatable position reference
  encoder.read_abs_angle();
  servo_ctrl->get_encoder().reset_abs_angle_period();

  // the motor is currently held against the end stop by the field, defining a geometric reference
  home_encoder_angle = encoder.read_abs_angle();
  home_raw = encoder.get_last_abs_raw_angle();
  if(use_measured_backoff && encoder.get_status() != 0) {
    search_failed = true;
    state = State::Failed;
    LOG_ERROR("HOME SEARCH: invalid endstop encoder sample");
    return;
  }
  #ifdef HOMING_PHASE_TRACE
    // Capture only here: logging would delay the other axes still homing.
    endstop_raw = encoder.get_last_abs_raw_angle();
    endstop_field = servo_ctrl->get_motor_driver().get_field_angle();
  #endif
  LOG_DEBUG("home_encoder_angle=%f deg", home_encoder_angle*Constants::RAD2DEG);
  if(home_encoder_angle < Constants::TWO_PI_F*0.01 || home_encoder_angle > Constants::TWO_PI_F*0.99)
    LOG_WARNING("encoder angle at home position close to wrap around point !");
}

void HomingController::finalize() {
  // A search failure is terminal. Never perform backoff repeatedly on timeout.
  if(state != State::Done || finalized) return;
  finalized = true;
   auto& motor_driver = servo_ctrl->get_motor_driver();
  #ifdef HOMING_PHASE_TRACE
    // Robot logs the one-based axis immediately before this call.
    log_phase_sample("endstop", endstop_raw, endstop_field);
  #endif
  if(restore_before_backoff) {
    // Compare backoff at the calibration amplitude, without raising its cap.
    // Search still uses homing_current; the field stays fixed during this ramp.
    #ifdef HOMING_PHASE_TRACE
      log_phase_sample("before_restore", servo_ctrl->get_encoder().read_abs_angle_raw(),
                       motor_driver.get_field_angle());
    #endif
    motor_driver.set_amplitude_smooth(initial_current, 100);
    #ifdef HOMING_PHASE_TRACE
      log_phase_sample("after_restore", servo_ctrl->get_encoder().read_abs_angle_raw(),
                       motor_driver.get_field_angle());
    #endif
  }
  #ifdef HOMING_PHASE_TRACE
    log_phase_sample("before_backoff", servo_ctrl->get_encoder().read_abs_angle_raw(),
                     motor_driver.get_field_angle());
  #endif
 
  // back off from home position
  if(use_measured_backoff) {
    if(!backoff_to_measured_clearance()) {
      search_failed = true;
      state = State::Failed;
      return;
    }
  } else {
    // Preserve calibration's original fixed-field backoff and origin.
    motor_driver.rotate_field(retract_field_angle * (field_velocity>0.0f ? -1.0f : 1.0f),
                            retract_field_velocity, [this](){
                              // update encoder so it doesnt miss a period
                              servo_ctrl->get_encoder().read_abs_angle();
                            });
  }

  #ifdef HOMING_PHASE_TRACE
    log_phase_sample("after_backoff", servo_ctrl->get_encoder().read_abs_angle_raw(),
                     motor_driver.get_field_angle());
  #endif

  #ifdef HOMING_TRANSITION_TEST
    LOG_INFO("HOME TEST: backoff finished; hold amplitude=%f for 1 second",
             motor_driver.get_amplitude());
    sleep_ms(1000);
    float raw_before_restore = servo_ctrl->get_encoder().read_abs_angle_raw();
    #ifdef HOMING_PHASE_TRACE
      log_phase_sample("before_restore", int32_t(raw_before_restore), motor_driver.get_field_angle());
    #endif
    LOG_INFO("HOME TEST: restoring amplitude %f -> %f; raw_before=%f",
             motor_driver.get_amplitude(), initial_current, raw_before_restore);
  #endif

  if(!restore_before_backoff) {
    #if defined(HOMING_PHASE_TRACE) && !defined(HOMING_TRANSITION_TEST)
      log_phase_sample("before_restore", servo_ctrl->get_encoder().read_abs_angle_raw(),
                       motor_driver.get_field_angle());
    #endif
    // restore previous motor current
    motor_driver.set_amplitude_smooth(initial_current, 100);
    #if defined(HOMING_PHASE_TRACE) && !defined(HOMING_TRANSITION_TEST)
      log_phase_sample("after_restore", servo_ctrl->get_encoder().read_abs_angle_raw(),
                       motor_driver.get_field_angle());
    #endif
  }

  #ifdef HOMING_TRANSITION_TEST
    LOG_INFO("HOME TEST: amplitude restored; hold field unchanged for 1 second");
    sleep_ms(1000);
    float raw_after_restore = servo_ctrl->get_encoder().read_abs_angle_raw();
    #ifdef HOMING_PHASE_TRACE
      log_phase_sample("after_restore", int32_t(raw_after_restore), motor_driver.get_field_angle());
    #endif
    LOG_INFO("HOME TEST: raw_after=%f, raw_change_during_restore=%f",
             raw_after_restore, raw_after_restore - raw_before_restore);
  #endif

  if(use_measured_backoff && !settle_backoff()) {
    LOG_ERROR("HOME BACKOFF: position did not settle inside verified clearance");
    search_failed = true;
    state = State::Failed;
  }
}

bool HomingController::read_backoff_position(float& clearance, float& position, bool& in_range) {
  auto& encoder = servo_ctrl->get_encoder();
  int32_t raw = encoder.read_abs_angle_raw();
  if(encoder.get_status() != 0) {
    LOG_ERROR("HOME BACKOFF: invalid encoder sample");
    return false;
  }
  // int64 subtraction avoids overflow; reads remain unwrapped across periods.
  clearance = float(int64_t(raw)-int64_t(home_raw)) * encoder_backoff_direction *
              (Constants::TWO_PI_F / float(encoder.get_rawcounts_per_rev())) * encoder_to_motor_angle;
  const float tolerance = HOMING_BACKOFF_SETTLE_TOLERANCE_DEG*Constants::DEG2RAD;
  if(!std::isfinite(clearance) || clearance < -tolerance ||
     clearance > HOMING_BACKOFF_MAX_CLEARANCE_DEG*Constants::DEG2RAD + tolerance) {
    LOG_ERROR("HOME BACKOFF: wrong direction or excessive measured travel");
    return false;
  }
  // No extrapolation is allowed to certify backoff completion.
  in_range = servo_ctrl->get_enc_to_pos_lut().in_input_range(raw);
  position = in_range ? servo_ctrl->encoder_angle_to_motor_pos(raw) : 0.0f;
  if(in_range && !std::isfinite(position)) return false;
  if(in_range && ((field_velocity < 0 && position > trusted_position_max) ||
                  (field_velocity > 0 && position < trusted_position_min))) {
    LOG_ERROR("HOME BACKOFF: opposite calibrated travel boundary reached");
    return false;
  }
  in_range = in_range && position >= trusted_position_min && position <= trusted_position_max;
  return true;
}

bool HomingController::backoff_to_measured_clearance() {
  auto& driver = servo_ctrl->get_motor_driver();
  const float direction = field_velocity < 0 ? 1.0f : -1.0f;
  // Allow at most one additional electrical revolution for taking up stall
  // phase lag; actual encoder travel has an independent, tighter bound.
  const float max_field_advance = requested_clearance*servo_ctrl->get_pole_pair_count() + Constants::TWO_PI_F;
  const float field_step = retract_field_velocity * 0.001f;
  float advance = 0.0f;
  float held_field = driver.get_field_angle();
  uint64_t begin = time_us_64();
  while(time_us_64()-begin < HOMING_BACKOFF_TIMEOUT_MS*1000ULL) {
    float clearance, position;
    bool in_range;
    if(!read_backoff_position(clearance, position, in_range)) return false;
    if(clearance >= requested_clearance && in_range) {
      if(!settle_backoff()) {
        LOG_ERROR("HOME BACKOFF: measured destination did not settle");
        return false;
      }
      LOG_INFO("HOME BACKOFF: verified clearance_deg=%f", clearance*Constants::RAD2DEG);
      return true;
    }
    if(advance + field_step > max_field_advance) break;
    advance += field_step;
    driver.set_field_angle(held_field + direction*advance);
    sleep_ms(1);
  }
  LOG_ERROR("HOME BACKOFF: measured destination not reached within field/time limit");
  return false;
}

bool HomingController::settle_backoff() {
  const float tolerance = HOMING_BACKOFF_SETTLE_TOLERANCE_DEG*Constants::DEG2RAD;
  uint64_t begin = time_us_64(), stable_since = begin;
  float anchor = 0.0f;
  bool first = true;
  while(time_us_64()-begin < HOMING_BACKOFF_SETTLE_TIMEOUT_MS*1000ULL) {
    float clearance, position;
    bool in_range;
    if(!read_backoff_position(clearance, position, in_range)) return false;
    if(!in_range || clearance < requested_clearance-tolerance) return false;
    uint64_t now = time_us_64();
    if(first || fabsf(clearance-anchor) > tolerance) {
      anchor = clearance;
      stable_since = now;
      first = false;
    }
    if(now-stable_since >= HOMING_BACKOFF_SETTLE_MS*1000ULL) return true;
    sleep_ms(1);
  }
  return false;
}

#ifdef HOMING_PHASE_TRACE
void HomingController::log_phase_sample(const char* stage, int32_t raw, float field) {
  float pos = servo_ctrl->encoder_angle_to_motor_pos(raw);
  float calibrated_field = servo_ctrl->motor_pos_to_field_angle(pos);
  float enc_min, enc_max;
  servo_ctrl->get_enc_to_pos_lut().get_intput_range(enc_min, enc_max);
  // Separate short lines keep all values within the logger's 128-byte buffer.
  LOG_INFO("HOME PHASE: %s raw=%li in_lut=%i pos_deg=%f amplitude=%f", stage, (long)raw,
           int(servo_ctrl->get_enc_to_pos_lut().in_input_range(raw)), pos*Constants::RAD2DEG,
           servo_ctrl->get_motor_driver().get_amplitude());
  LOG_INFO("HOME PHASE: %s field_deg=%f reference_deg=%f mismatch_deg=%f", stage,
           field*Constants::RAD2DEG, calibrated_field*Constants::RAD2DEG,
           remainderf(calibrated_field-field, Constants::TWO_PI_F)*Constants::RAD2DEG);
  LOG_INFO("HOME PHASE: enc_lut_min=%f enc_lut_max=%f", enc_min, enc_max);
}
#endif

bool HomingController::is_finished() const {
  return state == State::Done || state == State::Failed;
}

bool HomingController::is_successful() const {
  return state == State::Done && finalized && !search_failed;
}

float HomingController::get_home_encoder_angle() const {
  return home_encoder_angle;
}

bool HomingController::get_measured_clearance_at_raw(
    int32_t raw, float& clearance, float& direction) const {
  clearance = 0.0f;
  direction = 0.0f;
  if(!use_measured_backoff || !is_successful() ||
     !std::isfinite(encoder_backoff_direction) ||
     !std::isfinite(encoder_to_motor_angle) ||
     !std::isfinite(away_from_stop_sign) || fabsf(away_from_stop_sign) != 1.0f)
    return false;
  clearance = float(int64_t(raw)-int64_t(home_raw)) * encoder_backoff_direction *
              (Constants::TWO_PI_F /
               float(servo_ctrl->get_encoder().get_rawcounts_per_rev())) *
              encoder_to_motor_angle;
  const float tolerance = HOMING_BACKOFF_SETTLE_TOLERANCE_DEG*Constants::DEG2RAD;
  if(!std::isfinite(clearance) || clearance < requested_clearance-tolerance ||
     clearance > HOMING_BACKOFF_MAX_CLEARANCE_DEG*Constants::DEG2RAD+tolerance)
    return false;
  direction = away_from_stop_sign;
  return true;
}
