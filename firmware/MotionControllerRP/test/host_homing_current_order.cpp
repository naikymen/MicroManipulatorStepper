// The runner inserts the actual HomingController::finalize method before main.
#define HOMING_PHASE_TRACE
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "utilities/math_constants.h"
#define LOG_ERROR(...) ((void)0)
struct Driver {
  float amplitude = 0.15f, field = 0, backoff_amplitude = 0;
  std::vector<char> actions;
  float get_field_angle() { return field; }
  void set_amplitude_smooth(float next, int ramp_ms) {
    assert(ramp_ms == 100 && next == 0.30f);
    amplitude = next;
    actions.push_back('R');
  }
  void rotate_field(float delta, float velocity, const std::function<void()>& step) {
    assert(delta == Constants::PI_F && velocity == 10.0f);
    backoff_amplitude = amplitude;
    field += delta;
    actions.push_back('B');
    step();
  }
};
struct Encoder {
  int32_t read_abs_angle_raw() { return 100; }
  float read_abs_angle() { return 1; }
};
struct ServoController {
  Driver driver;
  Encoder encoder;
  Driver& get_motor_driver() { return driver; }
  Encoder& get_encoder() { return encoder; }
  float get_pole_pair_count() { return 50; }
};
class HomingController {
public:
  enum class State { Done, Failed };
  State state = State::Done;
  bool finalized = false, search_failed = false, use_measured_backoff = false;
  bool backoff_to_measured_clearance() { assert(false); return false; }
  bool settle_backoff() { assert(false); return false; }
  ServoController servo;
  ServoController* servo_ctrl = &servo;
  float initial_current = .30f, field_velocity = -10;
  float retract_field_angle = Constants::PI_F, retract_field_velocity = 10;
  int32_t endstop_raw = 100;
  float endstop_field = 0;
  bool restore_before_backoff = false;
  std::vector<std::string> stages;
  void log_phase_sample(const char* stage, int32_t, float) { stages.push_back(stage); }
  void finalize();
};
int main() {
  HomingController home;
  #ifdef HOMING_RESTORE_BEFORE_BACKOFF_TEST
    home.restore_before_backoff = true;
  #endif
  home.finalize();
  assert(home.servo.driver.amplitude == .30f);
  auto actions = home.servo.driver.actions;
  home.finalize();
  assert(home.servo.driver.actions == actions); // exactly once
#ifdef HOMING_RESTORE_BEFORE_BACKOFF_TEST
  assert(home.servo.driver.backoff_amplitude == .30f);
  assert((home.servo.driver.actions == std::vector<char>{'R', 'B'}));
  assert((home.stages == std::vector<std::string>{"endstop", "before_restore",
          "after_restore", "before_backoff", "after_backoff"}));
#else
  assert(home.servo.driver.backoff_amplitude == .15f);
  assert((home.servo.driver.actions == std::vector<char>{'B', 'R'}));
  assert((home.stages == std::vector<std::string>{"endstop", "before_backoff",
          "after_backoff", "before_restore", "after_restore"}));
#endif
  puts("PASS: actual homing finalization preserves default order; optional test restores once before unchanged backoff, with accurate trace ordering.");
}
