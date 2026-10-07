// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "servo_controller.h"

//*** CLASS *****************************************************************************

/**
 * This class implements the homing procedure for a single actuator. To allow for parallel 
 * homing the class is stateful and has an update() function, that can be called togeather
 * with the updates of other homing controllers inside a loop.
 * 
 * Homing procedure:
 *   1. move axis in negative direction until a physical hard stop is reached
 *   2. reset encoder period
 *   3. back off slightly from the hard stop
 */
class HomingController {
  public:
    HomingController();

    // Starts a blocking homing cycle, motor_velocity can be negative and defines the homing direction.
    // WARNING: Servo loop updates (including encoder reads) must be completely disabled during homing.
    // @param servo_controller: servo controller instance used for homing
    // @param search_range_angle: search range angle for finding the endstop 
    // @param current: motor current factor used during homing, in range [0..1]
    // @param encoder_angle_to_motor_angle: conversion factor from encoder angle to motor angle
    // @param retract_angle_rad: encoder-measured calibration clearance
    bool run_blocking(ServoController* servo_controller, 
                      float motor_velocity, 
                      float search_range_angle, 
                      float current,
                      float encoder_angle_to_motor_angle,
                      float retract_angle_rad=-1.0f);
                      
    // Starts a non blocking homing cycle, motor_velocity can be negative and defines the homing direction.
    // WARNING: Servo loop updates (including encoder reads) must be completely disabled during homing.
    // Same parameters as run_blocking. The optional parameters select current
    // ordering, encoder-verified backoff, and a calibration-only raw-encoder
    // mode which does not require lookup tables that have not been measured yet.
    void start(ServoController* servo_controller, 
               float motor_velocity, 
               float search_range, 
               float current,
               float encoder_angle_to_motor_angle,
               float retract_angle_rad=-1.0f,
               bool restore_amplitude_before_backoff=false,
               bool measured_backoff=false,
               bool raw_encoder_backoff=false);

    void update();
    void finalize();

    bool is_finished() const;
    bool is_successful() const;
    float get_home_encoder_angle() const;
    // Re-evaluate clearance from the physical stop at a caller-supplied encoder
    // snapshot. This lets handover targets and travel limits use the same read.
    bool get_measured_clearance_at_raw(int32_t raw, float& clearance,
                                       float& away_from_stop_sign) const;

  private:
    void on_endstop_detected();
    bool backoff_to_measured_clearance();
    bool read_backoff_position(float& clearance, float& position, bool& in_range);
    bool settle_backoff();
    float compute_eval_pos_delta(float pos, float field_angle_delta);
    #ifdef HOMING_PHASE_TRACE
      void log_phase_sample(const char* stage, int32_t raw, float field);
      int32_t endstop_raw = 0;
      float endstop_field = 0.0f;
    #endif

  private:
    enum class State {
      Idle,
      Initializing,
      Homing,
      Done,
      Failed
    };

    ServoController* servo_ctrl;

    // Configuration params
    float field_velocity = 0.0f; // defines homing direction
    float field_angle_search_range = 0.0f;
    float homing_current = 0.0f;
    float initial_current = 0.0f;
    bool restore_before_backoff = false;
    bool use_measured_backoff = false;
    bool use_raw_encoder_backoff = false;
    bool finalized = false;
    float encoder_to_motor_angle = 0.0f;
    float requested_clearance = 0.0f;
    float encoder_backoff_direction = 0.0f;
    float away_from_stop_sign = 0.0f;
    float trusted_position_min = 0.0f;
    float trusted_position_max = 0.0f;
    int32_t home_raw = 0;
    float retract_field_angle = 0.0f;
    float retract_field_velocity = 0.0f;

    // State machine
    State state = State::Idle;
    bool search_failed = false;

    // Timing
    uint64_t last_time = 0;

    // Offsets and tracking
    float start_field_angle = 0.0f;
    float field_angle_offset = 0.0f;
    float last_eval_field_angle_offset = 0.0f;
    float last_eval_encoder_angle = 0.0f;
    float eval_field_angle_delta = 0.0f;
    float expected_encoder_delta = 0.0f;

    float home_encoder_angle = 0.0f;
};
