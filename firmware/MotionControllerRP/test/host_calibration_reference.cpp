// The runner inserts the actual Robot calibration method before main.
#include <cassert>
#include <cstdio>
#include <vector>
constexpr int NUM_JOINTS = 3;
#define LOG_INFO(...) ((void)0)
#define LOG_ERROR(...) ((void)0)
void spin_lock_unsafe_blocking(int) {}
void spin_unlock_unsafe(int) {}
struct Encoder {
  unsigned crc_errors = 0, status = 0;
  unsigned get_crc_error_count(bool) { return crc_errors; }
  unsigned get_status() { return status; }
};
struct Driver {
  float amplitude = .3f;
  void set_amplitude(float value, bool) { amplitude = value; }
};
struct Servo {
  Driver driver;
  Driver& get_motor_driver() { return driver; }
};
struct JointHomeReference {
  bool valid = false;
  float final_position = 0, measured_clearance = 0, away_from_stop_sign = 1;
};
struct RobotJoint {
  Encoder enc;
  Encoder* encoder = &enc;
  Servo servo;
  Servo* servo_controller = &servo;
  bool success = true;
  int measurements = 0, saves = 0;
  bool save_success = true;
  bool is_homed = true, is_calibrated = true;
  bool calibrate(bool) { measurements++; return success; }
  bool store_calibration() { saves++; return save_success; }
};
class Robot {
public:
  Robot() { for(auto& home : joint_home_references) home.valid = true; }
  RobotJoint storage[NUM_JOINTS];
  RobotJoint* joints[NUM_JOINTS] = {&storage[0], &storage[1], &storage[2]};
  int joints_spin_lock = 0, resets = 0;
  struct { int lock = 0; float joint_target_positions[NUM_JOINTS]{}; } shared_data;
  bool all_joints_ready = false, restart_result = true;
  JointHomeReference joint_home_references[NUM_JOINTS];
  std::vector<bool> feedback_calls;
  void reset_motion_path() { resets++; }
  bool enable_servo_control(bool en) {
    feedback_calls.push_back(en);
    return !en || restart_result;
  }
  int pose_from_joint_angles() { return 0; }
  bool set_pose(int, bool) { return true; }
  bool check_all_joints_ready() { return true; }
  bool calibrate_joint(int, bool, bool);
};

int main() {
  Robot measured;
  assert(measured.calibrate_joint(2, false, true));
  assert(measured.resets == 1);
  assert(measured.storage[2].measurements == 1 && measured.storage[2].saves == 0);
  assert(!measured.joint_home_references[2].valid);
  assert(measured.storage[0].measurements == 0 && measured.storage[1].measurements == 0);
  assert(!measured.feedback_calls.front());
#ifdef CALIBRATION_REFERENCE_TEST
  assert(measured.feedback_calls.size() == 1);
#else
  assert(measured.feedback_calls.size() == 2 && measured.feedback_calls.back());
#endif
  Robot failed;
  failed.storage[2].success = false;
  assert(!failed.calibrate_joint(2, false, true));
  assert(failed.feedback_calls.size() == 1 && !failed.feedback_calls[0]);
  assert(failed.storage[2].saves == 0);
  Robot refused;
  refused.restart_result = false;
#ifdef CALIBRATION_REFERENCE_TEST
  assert(refused.calibrate_joint(2, false, false)); // No restart attempted.
#else
  assert(!refused.calibrate_joint(2, false, false)); // Propagate restart refusal.
#endif
  Robot invalid;
  assert(!invalid.calibrate_joint(-1, false, false));
  assert(!invalid.calibrate_joint(3, false, false));
  assert(invalid.feedback_calls.empty() && invalid.resets == 0);
  Robot saved;
  assert(saved.calibrate_joint(2, true, false));
  assert(saved.storage[2].saves == 1); // Saving requires explicit request.
  Robot save_failed;
  save_failed.storage[2].save_success = false;
  assert(!save_failed.calibrate_joint(2, true, false));
  assert(save_failed.storage[2].saves == 1);
  #ifdef SERVO_IDLE_DIAGNOSTIC
    assert(!failed.storage[2].is_homed && !failed.storage[2].is_calibrated);
    assert(failed.storage[2].servo.driver.amplitude == 0);
    for(bool save : {false, true}) {
      for(bool crc_fault : {true, false}) {
        Robot corrupt;
        corrupt.all_joints_ready = true;
        corrupt.storage[2].enc.crc_errors = crc_fault ? 1 : 0;
        corrupt.storage[2].enc.status = crc_fault ? 0 : 2;
        assert(!corrupt.calibrate_joint(2, save, false));
        assert(corrupt.storage[2].measurements == 1 && corrupt.storage[2].saves == 0);
        assert(!corrupt.storage[2].is_homed && !corrupt.storage[2].is_calibrated);
        assert(corrupt.storage[2].servo.driver.amplitude == 0 && !corrupt.all_joints_ready);
        assert(corrupt.storage[0].servo.driver.amplitude == .3f);
        assert(corrupt.storage[1].servo.driver.amplitude == .3f);
      }
    }
    puts("PASS: diagnostic refuses CRC/status-contaminated measurements before flash writes and removes motor power/readiness.");
  #endif
  puts("PASS: actual calibration isolates the selected axis, preserves flash without S, reports save failures, skips diagnostic restart, preserves normal refusal handling.");
}
