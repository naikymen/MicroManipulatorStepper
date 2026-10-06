// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include "main.h"

#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"

#include <Wire.h>
#include <algorithm>

#include "robot.h"
#include "utilities/logging.h"
#include "utilities/frequency_counter.h"
#include "kinematic_models/kinematic_model_delta3d.h"
#include "version.h"
#include "hw_config.h"
#include "LittleFS.h"
#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"

#include "demo_gcode_generator.h"

//*** GLOBALS ***************************************************************************

// NeoPixelConnect strip(PIN_BUILTIN_LED, 1);
Robot robot(0.01f);

#ifdef ENCODER_WIGGLE_TEST
namespace {

#ifndef ENCODER_DIAGNOSTIC_SPI_HZ
  #define ENCODER_DIAGNOSTIC_SPI_HZ 1000000
#endif
constexpr uint32_t DIAGNOSTIC_SPI_HZ = ENCODER_DIAGNOSTIC_SPI_HZ;
constexpr uint32_t DIAGNOSTIC_REPORT_INTERVAL_MS = 100;
constexpr uint32_t DIAGNOSTIC_ID_CHECK_INTERVAL_MS = 1000;

struct EncoderDiagnosticState {
  MT6835Encoder* encoder = nullptr;
  int32_t raw = 0;
  int32_t previous_raw = 0;
  int32_t last_delta = 0;
  uint32_t max_abs_delta = 0;
  uint32_t bad_status_samples = 0;
  uint32_t crc_errors_at_last_report = 0;
  bool id_ok = false;
};

EncoderDiagnosticState encoder_diagnostics[3];
uint32_t next_diagnostic_report_ms = 0;
uint32_t next_id_check_ms = 0;

uint32_t abs_delta(int32_t value) {
  return value < 0 ? uint32_t(-int64_t(value)) : uint32_t(value);
}

void setup_encoder_wiggle_test() {
  // Keep all motor drivers in standby. No PWM outputs or servo loops are started
  // in this diagnostic mode.
  gpio_init(PIN_MOTOR_EN);
  gpio_set_dir(PIN_MOTOR_EN, GPIO_OUT);
  gpio_put(PIN_MOTOR_EN, 0);

  MT6835Encoder::setup_spi(
    spi0, PIN_ENCODER_SCK, PIN_ENCODER_MOSI, PIN_ENCODER_MISO,
    DIAGNOSTIC_SPI_HZ
  );

  const int32_t cs_pins[3] = {
    PIN_ENCODER1_CS,
    PIN_ENCODER2_CS,
    PIN_ENCODER3_CS,
  };

  for (int i = 0; i < 3; ++i) {
    auto& state = encoder_diagnostics[i];
    state.encoder = new MT6835Encoder(spi0, cs_pins[i]);
    state.encoder->set_crc_enabled(true);
  }

  // All CS outputs are high before the first transaction.
  for (int i = 0; i < 3; ++i) {
    auto& state = encoder_diagnostics[i];
    state.id_ok = state.encoder->is_connected();
    state.raw = state.encoder->read_abs_angle_raw();
    state.previous_raw = state.raw;
    state.crc_errors_at_last_report = state.encoder->get_crc_error_count(false);
  }

  Serial.println();
  Serial.println("=== ENCODER WIGGLE TEST ===");
  Serial.println("Motors are disabled. Wiggle one wire or connector at a time.");
  Serial.println("Healthy stationary channel: small delta/max, st=0x0, bad=0, id=OK.");
  Serial.println("Status flags: O=overspeed W=weak-field U=undervoltage C=CRC.");
  Serial.println("CRC may be unsupported by some MT6835 modules; also watch raw/status/id.");
  Serial.printf("SPI=%lu Hz, report=%lu ms\n\n",
                (unsigned long)DIAGNOSTIC_SPI_HZ,
                (unsigned long)DIAGNOSTIC_REPORT_INTERVAL_MS);

  uint32_t now = millis();
  next_diagnostic_report_ms = now;
  next_id_check_ms = now + DIAGNOSTIC_ID_CHECK_INTERVAL_MS;
}

void update_encoder_wiggle_test() {
  uint32_t now = millis();

  for (int i = 0; i < 3; ++i) {
    auto& state = encoder_diagnostics[i];
    state.raw = state.encoder->read_abs_angle_raw();
    state.last_delta = state.raw - state.previous_raw;
    state.previous_raw = state.raw;
    state.max_abs_delta = std::max(state.max_abs_delta, abs_delta(state.last_delta));

    if ((state.encoder->get_status() & 0x0F) != 0)
      ++state.bad_status_samples;
  }

  if ((int32_t)(now - next_id_check_ms) >= 0) {
    for (auto& state : encoder_diagnostics)
      state.id_ok = state.encoder->is_connected();
    next_id_check_ms = now + DIAGNOSTIC_ID_CHECK_INTERVAL_MS;
  }

  if ((int32_t)(now - next_diagnostic_report_ms) >= 0) {
    Serial.printf("t=%8lu", (unsigned long)now);

    for (int i = 0; i < 3; ++i) {
      auto& state = encoder_diagnostics[i];
      uint8_t status = state.encoder->get_status() & 0x0F;
      uint32_t crc_errors = state.encoder->get_crc_error_count(false);
      uint32_t new_crc_errors = crc_errors - state.crc_errors_at_last_report;

      char flags[5] = {
        (status & MT6835_STATUS_OVERSPEED) ? 'O' : '-',
        (status & MT6835_STATUS_WEAKFIELD) ? 'W' : '-',
        (status & MT6835_STATUS_UNDERVOLT) ? 'U' : '-',
        (status & MT6835_CRC_ERROR) ? 'C' : '-',
        '\0'
      };

      Serial.printf(" | E%d raw=%10ld d=%8ld max=%8lu st=%X[%s] bad=%4lu crc+=%3lu id=%s",
                    i + 1,
                    (long)state.raw,
                    (long)state.last_delta,
                    (unsigned long)state.max_abs_delta,
                    status,
                    flags,
                    (unsigned long)state.bad_status_samples,
                    (unsigned long)new_crc_errors,
                    state.id_ok ? "OK" : "FAIL");

      state.max_abs_delta = 0;
      state.bad_status_samples = 0;
      state.crc_errors_at_last_report = crc_errors;
    }

    Serial.println();
    next_diagnostic_report_ms = now + DIAGNOSTIC_REPORT_INTERVAL_MS;
  }

  delay(2);
}

}  // namespace
#endif

#ifdef MOTOR_STEP_TEST
namespace {

// The TB6612 has no current feedback. These values are deliberately low PWM
// duty caps for the tested 12 V supply, with its current limit as a backstop.
constexpr float MOTOR_TEST_MAX_AMPLITUDE = 0.15f;
constexpr float MOTOR3_TEST_MAX_AMPLITUDE = 0.145f;
constexpr int MOTOR_TEST_MICROSTEPS_PER_FULL_STEP = 16;
constexpr int MOTOR_TEST_FULL_STEP_COUNT = 8;
constexpr int MOTOR_TEST_MICROSTEP_COUNT =
  MOTOR_TEST_FULL_STEP_COUNT * MOTOR_TEST_MICROSTEPS_PER_FULL_STEP;
constexpr float MOTOR_TEST_FIELD_MICROSTEP =
  (Constants::PI_F * 0.5f) / MOTOR_TEST_MICROSTEPS_PER_FULL_STEP;
constexpr uint32_t MOTOR_TEST_MICROSTEP_DELAY_MS = 15;

TB6612MotorDriver motor_test_drivers[3] = {
  TB6612MotorDriver(
    PIN_MOTOR_EN, PIN_M1_PWM_A_POS, PIN_M1_PWM_A_NEG, PIN_MOTOR_PWMAB,
    PIN_MOTOR_EN, PIN_M1_PWM_B_POS, PIN_M1_PWM_B_NEG, PIN_MOTOR_PWMAB),
  TB6612MotorDriver(
    PIN_MOTOR_EN, PIN_M2_PWM_A_POS, PIN_M2_PWM_A_NEG, PIN_MOTOR_PWMAB,
    PIN_MOTOR_EN, PIN_M2_PWM_B_POS, PIN_M2_PWM_B_NEG, PIN_MOTOR_PWMAB),
  TB6612MotorDriver(
    PIN_MOTOR_EN, PIN_M3_PWM_A_POS, PIN_M3_PWM_A_NEG, PIN_MOTOR_PWMAB,
    PIN_MOTOR_EN, PIN_M3_PWM_B_POS, PIN_M3_PWM_B_NEG, PIN_MOTOR_PWMAB),
};

void stop_motor_step_test() {
  for (auto& driver : motor_test_drivers)
    driver.set_amplitude(0.0f, true);
  motor_test_drivers[0].disable();  // PIN_MOTOR_EN is shared by all drivers.
}

void test_motor_steps(int motor_index) {
  stop_motor_step_test();
  auto& driver = motor_test_drivers[motor_index];
  const float test_amplitude =
    motor_index == 2 ? MOTOR3_TEST_MAX_AMPLITUDE : MOTOR_TEST_MAX_AMPLITUDE;

  Serial.printf("M%d: ramp to %.1f%% PWM; %d full steps forward/back at 1/%d microstepping\n",
                motor_index + 1,
                test_amplitude * 100.0f,
                MOTOR_TEST_FULL_STEP_COUNT,
                MOTOR_TEST_MICROSTEPS_PER_FULL_STEP);

  driver.set_field_angle(0.0f);
  driver.enable();
  driver.set_amplitude_smooth(test_amplitude, 250);
  delay(250);

  for (int i = 1; i <= MOTOR_TEST_MICROSTEP_COUNT; ++i) {
    driver.set_field_angle(i * MOTOR_TEST_FIELD_MICROSTEP);
    if ((i % MOTOR_TEST_MICROSTEPS_PER_FULL_STEP) == 0)
      Serial.printf("M%d forward step %d/%d\n", motor_index + 1,
                    i / MOTOR_TEST_MICROSTEPS_PER_FULL_STEP, MOTOR_TEST_FULL_STEP_COUNT);
    delay(MOTOR_TEST_MICROSTEP_DELAY_MS);
  }

  for (int i = MOTOR_TEST_MICROSTEP_COUNT - 1; i >= 0; --i) {
    driver.set_field_angle(i * MOTOR_TEST_FIELD_MICROSTEP);
    int completed = MOTOR_TEST_MICROSTEP_COUNT - i;
    if ((completed % MOTOR_TEST_MICROSTEPS_PER_FULL_STEP) == 0)
      Serial.printf("M%d reverse step %d/%d\n", motor_index + 1,
                    completed / MOTOR_TEST_MICROSTEPS_PER_FULL_STEP,
                    MOTOR_TEST_FULL_STEP_COUNT);
    delay(MOTOR_TEST_MICROSTEP_DELAY_MS);
  }

  driver.set_amplitude_smooth(0.0f, 100);
  driver.disable();
  Serial.printf("M%d: disabled\n", motor_index + 1);
}

void test_motor_phases(int motor_index) {
  stop_motor_step_test();
  auto& driver = motor_test_drivers[motor_index];

  auto hold_phase = [&](const char* phase_name, float field_angle) {
    driver.set_field_angle(field_angle);
    driver.enable();
    driver.set_amplitude_smooth(MOTOR_TEST_MAX_AMPLITUDE, 250);
    Serial.printf("M%d PHASE %s HOLD -- check holding torque now\n",
                  motor_index + 1, phase_name);
    delay(2000);
    driver.set_amplitude_smooth(0.0f, 100);
    driver.disable();
    Serial.printf("M%d PHASE %s OFF\n", motor_index + 1, phase_name);
    delay(750);
  };

  hold_phase("A", Constants::PI_F * 0.5f);
  hold_phase("B", 0.0f);
  stop_motor_step_test();
  Serial.printf("M%d phase test complete; all outputs disabled\n", motor_index + 1);
}

void measure_motor1_phase(const char* phase_name, float field_angle,
                          const char* terminal_pair) {
  stop_motor_step_test();
  auto& driver = motor_test_drivers[0];
  driver.set_field_angle(field_angle);
  driver.enable();
  driver.set_amplitude_smooth(MOTOR_TEST_MAX_AMPLITUDE, 250);
  Serial.printf("M1 PHASE %s MEASUREMENT HOLD (8 seconds) -- read %s DC volts now\n",
                phase_name, terminal_pair);
  delay(8000);
  driver.set_amplitude_smooth(0.0f, 100);
  driver.disable();
  Serial.printf("M1 PHASE %s OFF; all outputs disabled\n", phase_name);
}

void setup_motor_step_test() {
  for (auto& driver : motor_test_drivers) {
    driver.begin();
    driver.set_amplitude(0.0f, true);
  }
  stop_motor_step_test();

  Serial.println();
  Serial.println("=== INTERACTIVE MOTOR STEP TEST ===");
  Serial.println("Robot, encoders, homing, and calibration are bypassed.");
  Serial.println("Commands: 1/2/3=test motor, a=all, p=M1 phases,");
  Serial.println("          b=B+, c=A+, d=A-, e=B- measurement holds, x=disable.");
  Serial.printf("Each test: M1/M2 %.1f%%, M3 %.1f%% PWM; %d full steps forward/back, 1/%d microsteps.\n",
                MOTOR_TEST_MAX_AMPLITUDE * 100.0f,
                MOTOR3_TEST_MAX_AMPLITUDE * 100.0f,
                MOTOR_TEST_FULL_STEP_COUNT,
                MOTOR_TEST_MICROSTEPS_PER_FULL_STEP);
  Serial.println("PWM is only a voltage-duty cap; set an external current limit.");
  Serial.println("Outputs are currently disabled.");
}

void update_motor_step_test() {
  if (!Serial.available()) {
    delay(2);
    return;
  }

  char command = Serial.read();
  if (command >= '1' && command <= '3') {
    test_motor_steps(command - '1');
  } else if (command == 'a' || command == 'A') {
    for (int i = 0; i < 3; ++i) {
      test_motor_steps(i);
      delay(500);
    }
  } else if (command == 'p' || command == 'P') {
    test_motor_phases(0);
  } else if (command == 'b' || command == 'B') {
    measure_motor1_phase("B+", 0.0f, "B1-B2");
  } else if (command == 'c' || command == 'C') {
    measure_motor1_phase("A+", Constants::PI_F * 0.5f, "A1-A2");
  } else if (command == 'd' || command == 'D') {
    measure_motor1_phase("A-", -Constants::PI_F * 0.5f, "A1-A2");
  } else if (command == 'e' || command == 'E') {
    measure_motor1_phase("B-", Constants::PI_F, "B1-B2");
  } else if (command == 'x' || command == 'X') {
    stop_motor_step_test();
    Serial.println("All motor outputs disabled.");
  }
}

}  // namespace
#endif

/*
MT6835Encoder encoder1(spi0, PIN_ENCODER1_CS);
MT6835Encoder encoder2(spi0, PIN_ENCODER2_CS);
MT6835Encoder encoder3(spi0, PIN_ENCODER3_CS);

TB6612MotorDriver motor_driver1(
  PIN_MOTOR_EN, PIN_M1_PWM_A_POS, PIN_M1_PWM_A_NEG, PIN_MOTOR_PWMAB,
  PIN_MOTOR_EN, PIN_M1_PWM_B_POS, PIN_M1_PWM_B_NEG, PIN_MOTOR_PWMAB
);
TB6612MotorDriver motor_driver2(
  PIN_MOTOR_EN, PIN_M2_PWM_A_POS, PIN_M2_PWM_A_NEG, PIN_MOTOR_PWMAB,
  PIN_MOTOR_EN, PIN_M2_PWM_B_POS, PIN_M2_PWM_B_NEG, PIN_MOTOR_PWMAB
);
TB6612MotorDriver motor_driver3(
  PIN_MOTOR_EN, PIN_M3_PWM_A_POS, PIN_M3_PWM_A_NEG, PIN_MOTOR_PWMAB,
  PIN_MOTOR_EN, PIN_M3_PWM_B_POS, PIN_M3_PWM_B_NEG, PIN_MOTOR_PWMAB
);

ServoController servo_controller1(motor_driver1, encoder1, 400/4);
ServoController servo_controller2(motor_driver2, encoder2, 400/4);
ServoController servo_controller3(motor_driver3, encoder3, 400/4);
FrequencyCounter loop_freq_counter(1000);

PathPlanner planner(0.01f);
MotionController motion_controller(&planner);
Pose6DF current_pose;

CommandParser command_parser; */

//*** FUNCTIONS *************************************************************************

// Run before setup()
//__attribute__((constructor))
void overclock() {
  vreg_set_voltage(VREG_VOLTAGE_1_20);         // For >133 MHz
  busy_wait_us(10 * 1000);  // 10 ms delay
  set_sys_clock_khz(250000, true);             // Set to 250 MHz
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
 /* strip.neoPixelSetValue(0, r, g, b, false);
  delayMicroseconds(2000);
  strip.neoPixelShow(); */
}

void led_blink(uint8_t r, uint8_t g, uint8_t b, int count, int period_time_ms) {
  for(int i=0; i<count; i++) {
    gpio_put(PIN_BUILTIN_LED, 1);
    // set_led_color(r, g, b);
    sleep_ms(period_time_ms/2);
    // set_led_color(0, 0, 0);
    gpio_put(PIN_BUILTIN_LED, 0);
    sleep_ms(period_time_ms/2);
  }
}

void main_core0() {
  uint64_t last_time = time_us_64();

  #ifdef DEMO_MODE
    auto* demo_gcode_generator = new DemoGcodeGenerator(&robot);
    demo_gcode_generator->run();  // blocks forever
  #endif

  while(true) {
    // update motion controller
    robot.update_command_parser();
    robot.update_path_planner();
  }
}

void main_core1() {
  LOG_INFO("Starting servo controll loops on core 1...");

  uint64_t last_time = time_us_64();
  while(true) {
    // get time and detla time
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time)*1e-6f;
    last_time = time_us;

    // limit time delta
    dt = std::min(dt, 0.0001f);

    // update servo loops
    robot.update_servo_controllers(dt);
  }
}

void setup() {
  gpio_init(PIN_BUILTIN_LED);
  gpio_set_dir(PIN_BUILTIN_LED, GPIO_OUT);
  
  led_blink(0, 0, 30, 3, 100);
  // stdio_init_all();  // Initializes USB or UART stdio
  overclock();
  // Serial.begin(921600);
  Logger::instance().begin(921600, false);
  #ifndef DEMO_MODE
  while(!Serial);
  #endif

  set_led_color(50, 10, 0);
  // auto* test = new KinematicModel_Delta3D(); test->test(); delete test;

  delay(100);  // Allow time for serial monitor to connect

  #ifdef ENCODER_WIGGLE_TEST
    setup_encoder_wiggle_test();
    gpio_put(PIN_BUILTIN_LED, 1);
    return;
  #endif

  #ifdef MOTOR_STEP_TEST
    setup_motor_step_test();
    gpio_put(PIN_BUILTIN_LED, 1);
    return;
  #endif
  
  LOG_INFO("Open Micro Stage Firmware: %s", FIRMWARE_VERSION);
  LOG_INFO("System clock: %i Mhz", int32_t(clock_get_hz(clk_sys))/1000/1000);

  // LittleFS.format();
  if (!LittleFS.begin()) {
    LOG_ERROR("Mounting filesystem failed");
  } else {
    FSInfo fs_info;
    LittleFS.info(fs_info);
    LOG_INFO("Mounting filesystem successfully [%i/%i bytes used]", 
             (int)fs_info.usedBytes, (int)fs_info.totalBytes);
  }

  LOG_INFO("Initializing device...");
  robot.init();

  multicore_launch_core1(&main_core1);
  sleep_ms(100);
  
  set_led_color(0, 20, 0);
  LOG_INFO("Initialization finished");
  LOG_INFO(" ");

  gpio_put(PIN_BUILTIN_LED, 1);

  return;

  /*
  rotencoder_wire.setSDA(PIN_ENCODER_SDA);
  rotencoder_wire.setSCL(PIN_ENCODER_SCL);
  rotencoder_wire.begin();
  rotencoder_wire.setClock(1000000);
  encoder.init();
  encoder.set_hysteresis(0x4); //0x6);
  */
}

void loop() {
  #ifdef ENCODER_WIGGLE_TEST
    update_encoder_wiggle_test();
    return;
  #endif

  #ifdef MOTOR_STEP_TEST
    update_motor_step_test();
    return;
  #endif

  main_core0();
}
