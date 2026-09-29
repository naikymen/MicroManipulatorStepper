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

#include "demo_gcode_generator.h"

//*** GLOBALS ***************************************************************************

// NeoPixelConnect strip(PIN_BUILTIN_LED, 1);
Robot robot(0.01f);

#ifdef ENCODER_WIGGLE_TEST
namespace {

constexpr uint32_t DIAGNOSTIC_SPI_HZ = 1000000;
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

  main_core0();
}
