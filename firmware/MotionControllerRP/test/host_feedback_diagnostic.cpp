// Runner inserts the real ServoController declaration and update/recorder
// methods between these fixtures and main. No hardware is accessed.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include "hw_config.h"
#include "servo_control/pid.h"
uint64_t clock_us = 1000;
uint64_t time_us_64() { return clock_us; }
struct MT6835Encoder {
  int32_t raw = 15000;
  uint8_t status = 0;
  int32_t read_abs_angle_raw() { return raw; }
  uint8_t get_status() { return status; }
};
struct TB6612MotorDriver {
  float amplitude = .3f, field = .75f;
  uint32_t writes = 0;
  float get_amplitude() { return amplitude; }
  float get_field_angle() { return field; }
  void set_field_angle(float f) { field=f; ++writes; }
  void set_amplitude(float a, bool) { amplitude=a; }
  void set_amplitude_smooth(float a, uint32_t) { amplitude=a; }
  void enable() {}
};
struct LookupTable {
  float scale = 1e-6f;
  bool in_input_range(float raw) const { return std::isfinite(raw) && fabsf(raw)<=500000; }
  float evaluate(float raw) const { return raw*scale; }
};
// Production declaration and methods injected here.
// FIXTURE_METHODS
ServoController::ServoController(TB6612MotorDriver& d, MT6835Encoder& e, int32_t poles)
  : encoder(e), motor_driver(d) {
  motor_pole_pair_count=poles;
  motor_pos_to_field_angle_lut.scale=50;
  pos_controller.set_parameter(60,30000,0,6.283185f,1.570796f);
  velocity_controller.set_parameter(.2f,90,0,1.4137167f,1.4137167f);
  velocity_lowpass.set_time_constant(.004f);
}
float ServoController::encoder_angle_to_motor_pos(int32_t raw) {
  return encoder_raw_to_motor_pos_lut.evaluate(raw);
}
float ServoController::motor_pos_to_field_angle(float pos) {
  return motor_pos_to_field_angle_lut.evaluate(pos);
}
float ServoController::read_position() {
  return encoder_angle_to_motor_pos(encoder.read_abs_angle_raw());
}
// Mock Robot surroundings; the runner injects the actual all-joint update.
constexpr int NUM_JOINTS=3;
void spin_lock_unsafe_blocking(int) {}
void spin_unlock_unsafe(int) {}
struct RobotJoint {
  ServoController* servo_controller;
  bool is_homed=true;
  float target=.015f;
  void update_target(float p,float) { target=p; }
  void update(float dt,float inv) { servo_controller->update(target,dt,inv); }
};
struct Robot {
  struct JointHomeReference { bool valid=false; };
  RobotJoint* joints[3];
  int joints_spin_lock=0;
  struct {
    int lock=0;
    float joint_target_positions[3]={.015f,.015f,.015f};
    float joint_target_velocities[3]={};
  } shared_data;
  JointHomeReference joint_home_references[3]={{true},{true},{true}};
  bool all_joints_ready=true;
  struct { void update(float) {} } servo_loop_frequency_counter;
  void update_servo_controllers(float dt);
};
// ROBOT_METHODS
// TEST_MAIN
int main() {
  using Fault=ServoController::FeedbackFault;
  auto step=[](ServoController& s,float target=.015f) {
    clock_us+=1000;
    s.update(target,.001f,1000);
  };
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);
    for(int n=0;n<1000;++n) step(s);
    assert(s.feedback_fault==Fault::None && d.amplitude==.3f);
    assert(s.feedback_trace_count==512);
    uint32_t oldest=s.feedback_trace_next;
    assert(s.feedback_trace[oldest].time_us < s.feedback_trace[(oldest+511)%512].time_us);
    auto writes=d.writes;
    e.status=8;
    step(s);
    assert(s.feedback_fault==Fault::EncoderStatus && d.amplitude==0 && d.writes==writes);
    auto head=s.feedback_trace_next;
    e.status=0; step(s);
    assert(s.feedback_trace_next==head && d.writes==writes);
    s.set_motor_enabled(true, true);
    s.set_motor_update_enabled(true);
    s.start_homing_handover(.015f);
    assert(d.amplitude==0 && d.writes==writes);
    assert(!s.get_homing_handover_status().active && s.feedback_fault==Fault::EncoderStatus);
  }
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);e.raw=600000;step(s);
    assert(s.feedback_fault==Fault::CalibrationRange && d.amplitude==0 && d.writes==0);
  }
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);e.raw+=20000;step(s);
    assert(s.feedback_fault==Fault::HandoverFailed && d.amplitude==0 && d.writes==0);
  }
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);
    for(int n=0;n<250;++n)step(s);
    auto writes=d.writes;
    e.raw+=20000;step(s);
    assert(s.feedback_fault==Fault::TrackingError && d.amplitude==0 && d.writes==writes);
  }
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);
    for(int n=0;n<250;++n)step(s);
    e.raw+=4000;
    for(int n=0;n<55;++n)step(s);
    assert(s.feedback_fault==Fault::TrackingError && d.amplitude==0);
  }
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);
    for(int n=0;n<250;++n)step(s);
    s.velocity_controller.set_parameter(100,0,0,1.4137167f,1.4137167f);
    for(int n=0;n<105;++n)step(s,.016f);
    assert(s.feedback_fault==Fault::SustainedCorrection && d.amplitude==0);
  }
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);
    step(s, std::numeric_limits<float>::quiet_NaN());
    assert(s.feedback_fault==Fault::InvalidNumber && d.amplitude==0 && d.writes==0);
  }
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);
    s.update(.015f, 0.0f, 1000);
    assert(s.feedback_fault==Fault::InvalidNumber && d.amplitude==0 && d.writes==0);
  }
  {
    MT6835Encoder e; TB6612MotorDriver d; ServoController s(d,e,50);
    s.start_homing_handover(.015f);
    for(int n=0;n<250;++n) step(s);
    e.raw+=4000;
    step(s);
    assert(s.feedback_tracking_since!=0);
    s.set_motor_update_enabled(false);
    assert(s.feedback_tracking_since==0 && s.feedback_correction_since==0);
    clock_us+=1000000;
    s.set_motor_update_enabled(true);
    step(s);
    assert(s.feedback_fault==Fault::None); // Disabled time cannot trigger 50 ms cutoff.
    s.set_motor_enabled(false, false);
    assert(s.feedback_tracking_since==0 && s.feedback_correction_since==0);
    step(s);
    assert(s.feedback_fault==Fault::None && d.amplitude==0);
    s.set_motor_enabled(true, false);
    s.start_homing_handover(.019f);
    assert(s.feedback_tracking_since==0 && s.feedback_correction_since==0);
  }
  {
    MT6835Encoder enc[3]; TB6612MotorDriver drivers[3];
    ServoController a(drivers[0],enc[0],50), b(drivers[1],enc[1],50), c(drivers[2],enc[2],50);
    a.start_homing_handover(.015f);b.start_homing_handover(.015f);c.start_homing_handover(.015f);
    RobotJoint joints[3]={{&a},{&b},{&c}};
    Robot robot{{&joints[0],&joints[1],&joints[2]}};
    clock_us+=1000; robot.update_servo_controllers(.001f);
    enc[1].status=8;
    clock_us+=1000; robot.update_servo_controllers(.001f);
    assert(b.feedback_fault==Fault::EncoderStatus && !robot.all_joints_ready);
    for(int i=0;i<3;++i) {
      assert(drivers[i].amplitude==0 && joints[i].servo_controller->feedback_trace_frozen);
      assert(!joints[i].is_homed && !robot.joint_home_references[i].valid);
    }
    auto writes=drivers[0].writes+drivers[1].writes+drivers[2].writes;
    enc[1].status=0;
    clock_us+=1000; robot.update_servo_controllers(.001f);
    assert(writes==drivers[0].writes+drivers[1].writes+drivers[2].writes);
  }
  puts("PASS: healthy holding, chronological ring, invalid feedback, tracking and sustained correction shutdown; all joints stop and retain frozen records after a single-joint fault.");
}
