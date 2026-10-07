// Deterministic host test for the post-Home move to a usable Cartesian pose.
// The runner injects the production method; no hardware is accessed.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "hw_config.h"
#include "utilities/math3d.h"

constexpr int NUM_JOINTS = 3;
#define LOG_INFO(...) ((void)0)
#define LOG_ERROR(...) ((void)0)

uint64_t clock_us = 1000;
uint64_t time_us_64() { return clock_us; }
void spin_lock_unsafe_blocking(int*) {}
void spin_unlock_unsafe(int*) {}

struct JointHomeReference {
  bool valid = true;
  float final_position = 1.0f*Constants::DEG2RAD;
  float measured_clearance = HOMING_MEASURED_BACKOFF_ANGLE_DEG*Constants::DEG2RAD;
  float away_from_stop_sign = 1.0f;
};
struct Encoder {
  uint8_t status = 0;
  uint8_t get_status() const { return status; }
};
struct Lut {
  float lower = 0.02f*Constants::DEG2RAD;
  float upper = 82.5f*Constants::DEG2RAD;
  bool in_input_range(float value) const { return value >= lower && value <= upper; }
};
struct Servo {
  float position = 1.0f*Constants::DEG2RAD;
  bool updates = true, powered = true;
  Encoder encoder;
  Lut field;
  float read_position() const { return position; }
  float get_position() const { return position; }
  Encoder& get_encoder() { return encoder; }
  const Lut& get_pos_to_field_lut() const { return field; }
  void set_motor_update_enabled(bool value) { updates = value; }
  void set_motor_enabled(bool value, bool) { powered = value; }
};
struct Joint {
  bool is_homed = true, is_calibrated = true;
  Servo servo;
  Servo* servo_controller = &servo;
};
struct Model {
  bool valid = true;
  bool foreward(const float positions[NUM_JOINTS], Pose6DF& pose) {
    if(!valid) return false;
    pose.translation = Vec3F(positions[0], positions[1], positions[2]);
    return true;
  }
};

class Robot;
Robot* active_robot = nullptr;
bool follow_targets = true;
int fault_axis = -1;
uint64_t fault_time_us = 0;

class Robot {
 public:
  Joint storage[NUM_JOINTS];
  Joint* joints[NUM_JOINTS]{&storage[0], &storage[1], &storage[2]};
  int joint_lock_storage = 0, shared_lock_storage = 0;
  int* joints_spin_lock = &joint_lock_storage;
  struct Shared {
    int* lock = nullptr;
    volatile float joint_target_positions[NUM_JOINTS]{};
    volatile float joint_target_velocities[NUM_JOINTS]{};
  } shared_data;
  JointHomeReference joint_home_references[NUM_JOINTS];
  float planned_joint_positions[NUM_JOINTS]{};
  Pose6DF current_pose;
  Model model;
  Model* kinematic_model = &model;

  Robot() {
    shared_data.lock = &shared_lock_storage;
    active_robot = this;
    for(int i=0; i<NUM_JOINTS; ++i) {
      shared_data.joint_target_positions[i] = storage[i].servo.position;
      planned_joint_positions[i] = storage[i].servo.position;
    }
  }
  bool calculate_joint_travel_limit(int i, const JointHomeReference& home,
                                    float& lower, float& upper) const {
    lower = storage[i].servo.field.lower;
    upper = storage[i].servo.field.upper;
    return home.valid && lower < upper;
  }
  bool move_to_homing_finish_position(uint8_t joint_mask);
};

void sleep_ms(uint32_t milliseconds) {
  clock_us += uint64_t(milliseconds)*1000;
  if(!active_robot) return;
  if(fault_axis >= 0 && clock_us >= fault_time_us)
    active_robot->storage[fault_axis].servo.encoder.status = 8;
  if(!follow_targets) return;
  for(int i=0; i<NUM_JOINTS; ++i) {
    auto& servo = active_robot->storage[i].servo;
    if(servo.updates)
      servo.position = active_robot->shared_data.joint_target_positions[i];
  }
}

int main() {
  {
    clock_us = 1000;
    Robot robot;
    assert(robot.move_to_homing_finish_position(0x7));
    for(int i=0; i<NUM_JOINTS; ++i) {
      assert(fabsf(robot.storage[i].servo.position-
                   HOMING_FINISH_POSITION_DEG*Constants::DEG2RAD) < 1e-6f);
      assert(robot.storage[i].is_homed && robot.storage[i].servo.powered);
      assert(robot.shared_data.joint_target_velocities[i] == 0.0f);
    }
  }
  {
    clock_us = 1000;
    Robot partial;
    assert(partial.move_to_homing_finish_position(0x2));
    assert(fabsf(partial.storage[1].servo.position-
                 HOMING_FINISH_POSITION_DEG*Constants::DEG2RAD) < 1e-6f);
    assert(fabsf(partial.storage[0].servo.position-1.0f*Constants::DEG2RAD) < 1e-6f);
    assert(fabsf(partial.storage[2].servo.position-1.0f*Constants::DEG2RAD) < 1e-6f);
  }
  {
    clock_us = 1000;
    Robot invalid_target;
    invalid_target.storage[0].servo.field.upper = 6.0f*Constants::DEG2RAD;
    assert(!invalid_target.move_to_homing_finish_position(0x1));
    assert(!invalid_target.storage[0].is_homed &&
           !invalid_target.storage[0].servo.updates &&
           !invalid_target.storage[0].servo.powered);
  }
  {
    clock_us = 1000;
    Robot encoder_fault;
    fault_axis = 1;
    fault_time_us = 101000;
    assert(!encoder_fault.move_to_homing_finish_position(0x7));
    assert(!encoder_fault.storage[1].is_homed);
    fault_axis = -1;
  }
  {
    clock_us = 1000;
    Robot tracking_fault;
    follow_targets = false;
    assert(!tracking_fault.move_to_homing_finish_position(0x7));
    for(int i=0; i<NUM_JOINTS; ++i)
      assert(!tracking_fault.storage[i].is_homed);
    follow_targets = true;
  }
  puts("PASS: Home finish reaches the exact calibrated target and fails closed on range, encoder, or tracking errors.");
}
