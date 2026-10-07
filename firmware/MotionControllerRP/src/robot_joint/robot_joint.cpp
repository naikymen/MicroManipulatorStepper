// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include <LittleFS.h> 
#include <algorithm>
#include <cmath>
#include <limits>
#include "robot_joint.h"
#include "hw_config.h"
#include "utilities/logging.h"
#include "utilities/utilities.h"
#include "servo_control/homing_controller.h"
#include "servo_control/actuator_calibration.h"

//*** FUNCTION **************************************************************************

#ifdef CALIBRATION_REFERENCE_TEST
namespace {
struct CalibrationDifferenceStats {
  float sum = 0.0f;
  float sum_squared = 0.0f;
  float minimum = std::numeric_limits<float>::infinity();
  float maximum = -std::numeric_limits<float>::infinity();
  uint32_t count = 0;

  void add(float value) {
    sum += value;
    sum_squared += value*value;
    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
    ++count;
  }

  void log_degrees(const char* label) const {
    if(count == 0) {
      LOG_INFO("CAL COMP: %s has no overlapping samples", label);
      return;
    }
    float mean = sum/float(count);
    float variance = std::max(0.0f, sum_squared/float(count)-mean*mean);
    LOG_INFO("CAL COMP: %s mean_deg=%f std_deg=%f", label,
             mean*Constants::RAD2DEG, sqrtf(variance)*Constants::RAD2DEG);
    LOG_INFO("CAL COMP: %s min_deg=%f max_deg=%f n=%lu", label,
             minimum*Constants::RAD2DEG, maximum*Constants::RAD2DEG,
             (unsigned long)count);
  }
};

void compare_calibrations(const LookupTable& saved_encoder,
                          const LookupTable& saved_field,
                          const LookupTable& fresh_encoder,
                          const LookupTable& fresh_field) {
  constexpr uint32_t sample_count = 256;

  float saved_raw_min, saved_raw_max, fresh_raw_min, fresh_raw_max;
  saved_encoder.get_intput_range(saved_raw_min, saved_raw_max);
  fresh_encoder.get_intput_range(fresh_raw_min, fresh_raw_max);
  float raw_min = std::max(saved_raw_min, fresh_raw_min);
  float raw_max = std::min(saved_raw_max, fresh_raw_max);
  LOG_INFO("CAL COMP: raw_overlap=%f..%f", raw_min, raw_max);

  CalibrationDifferenceStats position_difference;
  CalibrationDifferenceStats composed_field_difference;
  float composed_anchor = 0.0f;
  bool composed_first = true;
  if(std::isfinite(raw_min) && std::isfinite(raw_max) && raw_min < raw_max) {
    for(uint32_t i=0; i<sample_count; ++i) {
      float raw = raw_min + (raw_max-raw_min)*float(i)/float(sample_count-1);
      float saved_position = saved_encoder.evaluate(raw);
      float fresh_position = fresh_encoder.evaluate(raw);
      position_difference.add(fresh_position-saved_position);

      float delta = remainderf(fresh_field.evaluate(fresh_position) -
                               saved_field.evaluate(saved_position),
                               Constants::TWO_PI_F);
      if(composed_first) {
        composed_anchor = delta;
        composed_first = false;
      }
      composed_field_difference.add(
        composed_anchor + remainderf(delta-composed_anchor, Constants::TWO_PI_F));
    }
  }
  position_difference.log_degrees("raw_to_position fresh-saved");
  composed_field_difference.log_degrees("raw_to_field fresh-saved electrical");

  float saved_pos_min, saved_pos_max, fresh_pos_min, fresh_pos_max;
  saved_field.get_intput_range(saved_pos_min, saved_pos_max);
  fresh_field.get_intput_range(fresh_pos_min, fresh_pos_max);
  float pos_min = std::max(saved_pos_min, fresh_pos_min);
  float pos_max = std::min(saved_pos_max, fresh_pos_max);
  LOG_INFO("CAL COMP: position_overlap_deg=%f..%f",
           pos_min*Constants::RAD2DEG, pos_max*Constants::RAD2DEG);

  CalibrationDifferenceStats field_origin_difference;
  float field_anchor = 0.0f;
  bool field_first = true;
  if(std::isfinite(pos_min) && std::isfinite(pos_max) && pos_min < pos_max) {
    for(uint32_t i=0; i<sample_count; ++i) {
      float position = pos_min + (pos_max-pos_min)*float(i)/float(sample_count-1);
      float delta = remainderf(fresh_field.evaluate(position) -
                               saved_field.evaluate(position),
                               Constants::TWO_PI_F);
      if(field_first) {
        field_anchor = delta;
        field_first = false;
      }
      field_origin_difference.add(
        field_anchor + remainderf(delta-field_anchor, Constants::TWO_PI_F));
    }
  }
  field_origin_difference.log_degrees("position_to_field fresh-saved electrical");
}
} // namespace
#endif

//*** CLASS *****************************************************************************

//--- RobotJoint ------------------------------------------------------------------------

RobotJoint::RobotJoint(MT6835Encoder* encoder, 
                       TB6612MotorDriver* motor_driver,
                       int pole_pairs) 
{
  RobotJoint::encoder = encoder;
  RobotJoint::motor_driver = motor_driver;
  servo_controller = new ServoController(*motor_driver, *encoder, pole_pairs);
  position = 0.0f;
  velocity = 0.0f;
}

RobotJoint::~RobotJoint() {
  delete servo_controller;
  delete motor_driver;
  delete encoder;
  servo_controller = nullptr;
  motor_driver = nullptr;
  encoder = nullptr;
}

void RobotJoint::init(int joint_idx) {
  RobotJoint::joint_idx = joint_idx;

  if(encoder->init(0x5, 0x4) == false) {
    LOG_ERROR("Failed to initialize encoder for joint %i", joint_idx);
  }

  servo_controller->init(MOTOR_MAX_CURRENT_FACTOR);
  servo_controller->set_motor_enabled(false, false);
}

bool RobotJoint::calibrate(bool print_measurements) {
  LOG_INFO("Joint-%i: calibrating joint...", joint_idx);

  #ifdef CALIBRATION_REFERENCE_TEST
    LookupTable saved_encoder_lut = servo_controller->get_enc_to_pos_lut();
    LookupTable saved_field_lut = servo_controller->get_pos_to_field_lut();
  #endif

  HomingController homing_controller;
  bool homing_ok = homing_controller.run_blocking(servo_controller, -HOMING_VELOCITY, 
                                                  360.0f*DEG_TO_RAD, HOMING_CURRENT,
                                                  ENCODER_ANGLE_TO_ROTOR_ANGLE,
                                                  CALIBRATION_BACKOFF_CLEARANCE_DEG*DEG_TO_RAD);
  if(homing_ok == false) {
    LOG_ERROR("Joint-%i: Calibration failed due to unsuccessful homing sequence", joint_idx);
    return false;
  }

  // measure lookup tables
  LookupTable encoder_raw_to_motor_pos_lut;
  LookupTable motor_pos_to_field_angle_lut;
  bool ok = measure_calibration_data(encoder_raw_to_motor_pos_lut, 
                                     motor_pos_to_field_angle_lut, 
                                     *servo_controller, 
                                     CALIBRATION_RANGE*DEG_TO_RAD,
                                     CALIBRATION_FIELD_VELOCITY, 
                                     256,
                                     print_measurements);
  if(!ok) {
    LOG_ERROR("Joint-%i: calibrating failed", joint_idx);
    return false;
  }

  #ifdef CALIBRATION_REFERENCE_TEST
    compare_calibrations(saved_encoder_lut, saved_field_lut,
                         encoder_raw_to_motor_pos_lut, motor_pos_to_field_angle_lut);
  #endif

  servo_controller->set_enc_to_pos_lut(encoder_raw_to_motor_pos_lut);
  servo_controller->set_pos_to_field_lut(motor_pos_to_field_angle_lut);
  is_calibrated = true;
  is_homed = true;

  LOG_INFO("Joint-%i: calibrating joint successful.", joint_idx);

  return true;
}

void RobotJoint::update(float dt, float one_over_dt) {
  servo_controller->update(position, dt, one_over_dt);
}

void RobotJoint::update_target(float p, float v) {
  position = p;
  velocity = v;
}

bool RobotJoint::load_calibration() {
  std::string fn1 = calib_data_filename("enc_to_pos_lut").c_str();
  std::string fn2 = calib_data_filename("pos_to_field_lut").c_str();
  if(!LittleFS.exists(fn1.c_str()) || !LittleFS.exists(fn2.c_str())) {
    LOG_WARNING("Joint-%i: Not all calibration files found. Run joint calibration with M56.", joint_idx);
    return false;
  }

  LookupTable enc_to_pos_lut;
  LookupTable pos_to_field_lut;
  bool res = true; 
  res &= load_lut_from_file(enc_to_pos_lut, fn1.c_str());
  res &= load_lut_from_file(pos_to_field_lut, fn2.c_str());
  if(res == false)
    return false;

  servo_controller->set_enc_to_pos_lut(enc_to_pos_lut);
  servo_controller->set_pos_to_field_lut(pos_to_field_lut);

  is_calibrated = true;
  LOG_INFO("Joint-%i: Encoder lookup tables loaded (size=%i,%i)", 
           joint_idx, enc_to_pos_lut.size(), pos_to_field_lut.size());

  return true;
}

bool RobotJoint::store_calibration() {
  bool res = true;

  res &= save_lut_to_file(servo_controller->get_enc_to_pos_lut(), 
                          calib_data_filename("enc_to_pos_lut").c_str());
  res &= save_lut_to_file(servo_controller->get_pos_to_field_lut(), 
                          calib_data_filename("pos_to_field_lut").c_str());

  return res;
}

std::string RobotJoint::calib_data_filename(std::string data_name) const {
  return std::string("joint")+std::to_string(joint_idx)+"_"+data_name+".dat";
}
