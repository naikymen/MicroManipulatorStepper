#pragma once
#include <cstdint>
#include "utilities/math_constants.h"

// Optional standalone diagnostics are selected through the encoder_wiggle_test
// and motor_step_test environments in platformio.ini; no source edit is needed.
#if defined(ENCODER_WIGGLE_TEST) && defined(MOTOR_STEP_TEST)
  #error "Select only one firmware diagnostic environment"
#endif
#if defined(HOMING_ENCODER_BACKOFF) && (!defined(HOMING_RESTART_GUARD) || \
    !defined(HOMING_TRANSITIONAL_FIELD_HANDOVER))
  #error "Measured G28 requires its guarded transitional field handover"
#endif

// #define DEMO_MODE

//--- MOTORS ------------------------------------------------------------------

// motor pole pair count
//  * 100 for 0.9deg stepper motors
//  * 50  for 1.8deg stepper motors
constexpr float MOTOR1_POLE_PAIRS = 50;
constexpr float MOTOR2_POLE_PAIRS = 50;
constexpr float MOTOR3_POLE_PAIRS = 50;

// max current factor in range [0..1]. Lower values reduce pwm resolution so a
// value above 0.4 is recommended.
// This machine uses a 12 V motor supply and approximately 6 ohm windings.
// Limit PWM to roughly 0.6 A average winding current. The TB6612 has no
// current feedback, so this remains a voltage-duty approximation.
constexpr float MOTOR_MAX_CURRENT_FACTOR = 0.30f;

//--- ENCODERS ----------------------------------------------------------------

// Conversion factor from encoder angle (one 2pi period every two magnets) to rotor angle.
// Used when the system can not rely on calibration data being present (e.g. during homing)
constexpr float ENCODER_MAGNET_PITCH = 3.0f;    // [mm]
constexpr float ENCODER_MAGNET_RADIUS = 30.0f;  // [mm]
constexpr float ENCODER_ANGLE_TO_ROTOR_ANGLE = (ENCODER_MAGNET_PITCH*2.0f) / 
                                               (ENCODER_MAGNET_RADIUS * Constants::TWO_PI_F);
											   
// enables error checking for encoders (slow) - useful for debugging
// Note: some chips seem to return always a crc of 0 producing massiv false errors       
#ifdef SERVO_IDLE_DIAGNOSTIC
constexpr bool ENABLE_ENCODER_CRC = true;
#else
constexpr bool ENABLE_ENCODER_CRC = false;
#endif

//--- HOMING ------------------------------------------------------------------

// Mechanical velocity. With 50 pole pairs this produces a 10 rad/s electrical
// field, still well below the original 50 rad/s setting.
constexpr float HOMING_VELOCITY   = 0.2f;        // rad per s
constexpr float HOMING_CURRENT    = 0.15f;       // range 0..1
// Legacy fixed-field backoff, expressed as mechanical motor degrees. Normal
// firmware uses the encoder-measured distances below instead.
constexpr float HOMING_BACKOFF_ANGLE_DEG = 3.6f;
// Calibration has no lookup table yet, so establish its starting point from
// raw encoder travel away from the stop. Normal G28 backs off farther and
// therefore finishes inside the range measured from this origin.
constexpr float CALIBRATION_BACKOFF_CLEARANCE_DEG = 1.5f;
// Normal G28 backs farther away than calibration's origin so its initial
// encoder position is unambiguously inside both saved lookup-table domains.
constexpr float HOMING_MEASURED_BACKOFF_ANGLE_DEG = 2.5f;
constexpr float HOMING_BACKOFF_MAX_CLEARANCE_DEG = 3.6f;
constexpr uint32_t HOMING_BACKOFF_TIMEOUT_MS = 3000;
constexpr uint32_t HOMING_BACKOFF_SETTLE_MS = 100;
constexpr uint32_t HOMING_BACKOFF_SETTLE_TIMEOUT_MS = 1000;
constexpr float HOMING_BACKOFF_SETTLE_TOLERANCE_DEG = 0.02f;
// Normal motion may use this fraction of the clearance measured by G28. The
// remainder stays between a commanded target and the detected physical stop.
constexpr float HOMING_USABLE_CLEARANCE_FRACTION = 0.75f;
// A temporary field offset makes the first feedback command identical to the
// field already holding the rotor. It then decays to zero while feedback holds
// the fresh encoder target. These are electrical degrees except where noted.
constexpr float HOMING_HANDOVER_FIELD_RATE_DEG_S = 60.0f;
constexpr float HOMING_HANDOVER_MAX_POSITION_ERROR_DEG = 0.5f;
constexpr float HOMING_HANDOVER_SETTLE_TOLERANCE_DEG = 0.05f;
constexpr uint32_t HOMING_HANDOVER_SETTLE_MS = 200;
constexpr uint32_t HOMING_HANDOVER_TIMEOUT_MS = 5000;
// After the field handover, move every selected joint to a common calibrated
// position with enough room for a full 1 mm Cartesian jog in every direction.
constexpr float HOMING_FINISH_POSITION_DEG = 7.0f;
constexpr float HOMING_FINISH_VELOCITY_DEG_S = 3.0f;
constexpr float HOMING_FINISH_MAX_TRACKING_ERROR_DEG = 1.0f;
constexpr float HOMING_FINISH_SETTLE_TOLERANCE_DEG = 0.05f;
constexpr uint32_t HOMING_FINISH_SETTLE_MS = 200;
constexpr uint32_t HOMING_FINISH_TIMEOUT_MS = 5000;
static_assert(HOMING_MEASURED_BACKOFF_ANGLE_DEG > CALIBRATION_BACKOFF_CLEARANCE_DEG,
              "Normal Home must finish beyond calibration's starting point");
static_assert(HOMING_MEASURED_BACKOFF_ANGLE_DEG <= HOMING_BACKOFF_MAX_CLEARANCE_DEG,
              "Normal Home clearance exceeds its measured safety bound");

//--- CALIBRATION -------------------------------------------------------------

// degrees from home position
constexpr float CALIBRATION_RANGE = 83; 
static_assert(HOMING_FINISH_POSITION_DEG > 0.0f &&
              HOMING_FINISH_POSITION_DEG < CALIBRATION_RANGE,
              "Post-Home target must be inside the calibrated motor range");

// The physical Home side uses the measured G28 clearance above. The other side
// has no measured stop reference and retains a fixed calibration-domain margin.
constexpr float JOINT_OPPOSITE_TRAVEL_MARGIN_DEG = 0.5f;
// Comparison tolerance only: accepted Cartesian/joint targets are not clamped.
constexpr float JOINT_LIMIT_NUMERIC_TOLERANCE_DEG = 0.0001f;

// velocity of the magnetic field during calibration (lower is more accurate)
constexpr float CALIBRATION_FIELD_VELOCITY = 10.0f;

// size of the calibration lookup table
constexpr int ENCODER_LUT_SIZE = 256;

//--- CLOSED LOOP CONTROL -----------------------------------------------------

// position controller 
constexpr float POS_KP = 60.0f;
constexpr float POS_KI = 30000.0f;

// velocity controller
constexpr float VEL_LOWPASS_TC = 0.004f;
constexpr float VEL_KP = 0.2f;
constexpr float VEL_KI = 90.0f;

//--- KINEMATIC ---------------------------------------------------------------

// Kinematic Parameters are defined kinematic_modes/kinematic_model_delta3d.cpp

// NUM_JOINTS and NUM_TOOLS are defined in 'path_segment.h'. Note that changing
// the number of joints requires changing the kinematic model accordingly and
// also requires the initialization of the correct number of 'RobotJoint' objects
// in the Robtos init method.

//--- PINS --------------------------------------------------------------------

#define JOINT_READY_OVERRIDE

// #define SINGLE_AXIS_BOARD
#ifndef SINGLE_AXIS_BOARD
  // Pins for 3Axis Board
  // #define PIN_BUILTIN_LED 23 // RP2040 pico clone
  // #define PIN_USER_BUTTON 24 // RP2040 pico clone
  #define PIN_BUILTIN_LED 25

  #define PIN_M1_PWM_A_POS  13
  #define PIN_M1_PWM_A_NEG  12
  #define PIN_M1_PWM_B_POS  14
  #define PIN_M1_PWM_B_NEG  15

  #define PIN_M2_PWM_A_POS  9
  #define PIN_M2_PWM_A_NEG  8
  #define PIN_M2_PWM_B_POS  10
  #define PIN_M2_PWM_B_NEG  11

  #define PIN_M3_PWM_A_POS  5
  #define PIN_M3_PWM_A_NEG  4
  #define PIN_M3_PWM_B_POS  6
  #define PIN_M3_PWM_B_NEG  7


  #define PIN_MOTOR_EN      18
  #define PIN_MOTOR_PWMAB   19

  #define PIN_ENCODER1_CS 20
  #define PIN_ENCODER2_CS 21
  #define PIN_ENCODER3_CS 22
  #define PIN_ENCODER_SCK 2
  #define PIN_ENCODER_MISO 0
  #define PIN_ENCODER_MOSI 3

  #define PIN_TOOL1 16
  #define PIN_TOOL2 17

#else
  // Single Axis Board
  #define PIN_BUILTIN_LED 16
  #define PIN_USER_BUTTON 24

  #define PIN_M1_PWM_A_POS  13
  #define PIN_M1_PWM_A_NEG  12
  #define PIN_M1_PWM_B_POS  14
  #define PIN_M1_PWM_B_NEG  15

  #define PIN_M2_PWM_A_POS  9
  #define PIN_M2_PWM_A_NEG  8
  #define PIN_M2_PWM_B_POS  10
  #define PIN_M2_PWM_B_NEG  11

  #define PIN_M3_PWM_A_POS  5
  #define PIN_M3_PWM_A_NEG  4
  #define PIN_M3_PWM_B_POS  6
  #define PIN_M3_PWM_B_NEG  7


  #define PIN_MOTOR_EN      18
  #define PIN_MOTOR_PWMAB   19

  #define PIN_ENCODER1_CS 20
  #define PIN_ENCODER2_CS 21
  #define PIN_ENCODER3_CS 22
  #define PIN_ENCODER_SCK 2
  #define PIN_ENCODER_MISO 0
  #define PIN_ENCODER_MOSI 3
#endif
