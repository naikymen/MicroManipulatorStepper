// Runner injects the actual Robot handover/update methods. Stubs record reads,
// published targets and lock order; no serial port or hardware is involved.
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>
#include "hw_config.h"
#define LOG_INFO(...) ((void)0)
#define LOG_ERROR(...) ((void)0)
constexpr int NUM_JOINTS = 3;
struct JointHomeReference {
  bool valid = false;
  float final_position = 0.0f;
  float measured_clearance = 0.0f;
  float away_from_stop_sign = 0.0f;
};
struct HomingController {
  bool valid = true;
  float clearance = 1.5f*Constants::DEG2RAD;
  float away_sign = 1.0f;
  bool get_measured_clearance_at_raw(int32_t, float& value, float& direction) const {
    if(!valid) return false;
    value = clearance;
    direction = away_sign;
    return true;
  }
};
std::vector<int> held_locks, lock_order;
void spin_lock_unsafe_blocking(int lock) {
  assert(std::find(held_locks.begin(), held_locks.end(), lock) == held_locks.end());
  // A shared-data acquisition must occur inside the joint lock, never before.
  if(lock == 2) assert(held_locks == std::vector<int>{1});
  held_locks.push_back(lock);
  lock_order.push_back(lock);
}
void spin_unlock_unsafe(int lock) {
  assert(!held_locks.empty() && held_locks.back() == lock);
  held_locks.pop_back();
}
struct Pose6DF { std::array<float, 3> values{}; };
struct Kinematic {
  bool valid = true;
  unsigned calls = 0;
  bool foreward(const float positions[3], Pose6DF& pose) {
    calls++;
    for(int i=0; i<3; ++i) pose.values[i] = positions[i];
    return valid;
  }
};
struct Encoder {
  int32_t raw = 1000;
  unsigned reads = 0;
  uint8_t status = 0;
  int32_t read_abs_angle_raw() { reads++; return raw++; }
  uint8_t get_status() { return status; }
};
struct Driver {
  float field = .5f, amplitude = .3f;
  float get_field_angle() { return field; }
  float get_amplitude() { return amplitude; }
};
struct Lut {
  float min = 0, max = 83*Constants::DEG2RAD;
  bool raw_input = false;
  uint32_t size() const { return 2; }
  bool is_monotonic() const { return true; }
  float get_entry(uint32_t i) const { return i ? max : min; }
  bool in_input_range(float x) const { return raw_input ? x >= 0 && x <= 100000 : x >= min && x <= max; }
  void get_intput_range(float& a, float& b) const { a=min; b=max; }
};
struct Servo {
  Encoder encoder;
  Driver driver;
  Lut field_lut, enc_lut;
  float measured = 0, reference_bias = 0;
  bool enabled = false;
  Servo() { enc_lut.raw_input = true; }
  Encoder& get_encoder() { return encoder; }
  Driver& get_motor_driver() { return driver; }
  const Lut& get_enc_to_pos_lut() const { return enc_lut; }
  const Lut& get_pos_to_field_lut() const { return field_lut; }
  float encoder_angle_to_motor_pos(int32_t raw) { return float(raw)*.00001f; }
  float motor_pos_to_field_angle(float pos) { return 50*pos + reference_bias; }
  void set_motor_update_enabled(bool enable, const float* pos=nullptr) {
    assert(held_locks == std::vector<int>{1});
    enabled=enable;
    if(enable) { assert(pos); measured=*pos; }
  }
};
struct Joint {
  bool is_homed = true, is_calibrated = true;
  Servo servo;
  Servo* servo_controller = &servo;
  float target = 0;
  void update_target(float pos, float vel) { assert(vel==0); target=pos; }
  void update(float, float) { if(servo.enabled) assert(target==servo.measured); }
};
struct Shared {
  int lock = 2;
  float joint_target_positions[3] = {9, 9, 9};
  float joint_target_velocities[3] = {1, 1, 1};
};
struct Frequency { void update(float) {} };
class Robot {
public:
  Joint storage[3];
  Joint* joints[3] = {&storage[0], &storage[1], &storage[2]};
  int joints_spin_lock = 1;
  Shared shared_data;
  JointHomeReference joint_home_references[NUM_JOINTS];
  Kinematic model;
  Kinematic* kinematic_model = &model;
  Pose6DF current_pose;
  float planned_joint_positions[NUM_JOINTS]{};
  Frequency servo_loop_frequency_counter;
  bool calculate_joint_travel_limit(int, const JointHomeReference&, float&, float&) const;
  bool finish_homing_handover(bool, HomingController[NUM_JOINTS], uint8_t);
  void update_servo_controllers(float);
};
int main() {
  auto finish = [](Robot& robot, HomingController home[NUM_JOINTS]) {
    spin_lock_unsafe_blocking(1);
    bool result = robot.finish_homing_handover(true, home, 0x7);
    spin_unlock_unsafe(1);
    return result;
  };
  Robot robot;
  HomingController home[NUM_JOINTS];
  assert(finish(robot, home));
  for(int i=0; i<3; ++i) {
    auto& servo = robot.storage[i].servo;
    assert(servo.encoder.reads==1 && servo.encoder.raw==1001);
    assert(servo.enabled && servo.measured==.01f);
    assert(robot.shared_data.joint_target_positions[i]==servo.measured);
    assert(robot.shared_data.joint_target_velocities[i]==0);
    assert(robot.planned_joint_positions[i]==servo.measured);
    assert(robot.current_pose.values[i]==servo.measured);
    assert(servo.driver.field==.5f);
    assert(robot.joint_home_references[i].valid);
    assert(robot.joint_home_references[i].final_position==servo.measured);
    assert(robot.joint_home_references[i].measured_clearance==home[i].clearance);
  }
  assert(robot.model.calls==1);
  {
    JointHomeReference reference{true, 2.0f*Constants::DEG2RAD,
                                 1.5f*Constants::DEG2RAD, 1.0f};
    float lower, upper;
    assert(robot.calculate_joint_travel_limit(0, reference, lower, upper));
    assert(fabsf(lower-0.875f*Constants::DEG2RAD) < 1e-6f);
    assert(fabsf(upper-82.5f*Constants::DEG2RAD) < 1e-6f);
    reference.final_position = 81.0f*Constants::DEG2RAD;
    reference.away_from_stop_sign = -1.0f;
    assert(robot.calculate_joint_travel_limit(0, reference, lower, upper));
    assert(fabsf(lower-0.5f*Constants::DEG2RAD) < 1e-6f);
    assert(fabsf(upper-82.125f*Constants::DEG2RAD) < 1e-6f);
  }
  lock_order.clear();
  robot.update_servo_controllers(.0001f);
  assert((lock_order == std::vector<int>{1, 2}));
  puts("PASS: actual handover samples once, publishes zero-velocity targets, preserves field, and servo locks before copying targets.");
  for(int fault=0; fault<4; ++fault) {
    Robot bad;
    HomingController bad_home[NUM_JOINTS];
    if(fault==0) bad.storage[1].servo.reference_bias=100*Constants::DEG2RAD;
    if(fault==1) bad.storage[1].servo.encoder.status=8;
    if(fault==2) bad.storage[1].servo.encoder.raw=-1; // outside calibrated raw domain
    if(fault==3) bad.model.valid=false;
    assert(!finish(bad, bad_home));
    assert(!bad.storage[1].is_homed && !bad.storage[1].servo.enabled);
    assert(bad.storage[1].servo.driver.field==.5f);
    assert(!bad.joint_home_references[1].valid);
  }
  Robot missing;
  HomingController missing_home[NUM_JOINTS];
  missing_home[1].valid=false;
  assert(!finish(missing, missing_home));
  assert(!missing.storage[1].is_homed && !missing.joint_home_references[1].valid);
  puts("PASS: phase mismatch, encoder fault, unsafe final position and invalid FK refuse restart without changing held field.");
}
