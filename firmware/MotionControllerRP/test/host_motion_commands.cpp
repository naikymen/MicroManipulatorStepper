// The runner inserts actual Robot command/validation methods before main.
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>
#include "hardware/sync.h"
#include "command_parser/command_parser.h"
#include "motion_control/path_planner.h"
#include "motion_control/motion_controller.h"
#include "robot_tool/robot_tool_interface.h"
#include "kinematic_models/kinematic_model_delta3d.h"
#include "utilities/math_constants.h"
#include "utilities/logging.h"

Logger& Logger::instance() { static Logger logger; return logger; }
void Logger::info(const char*, ...) {}
void Logger::debug(const char*, ...) {}
void Logger::warn(const char*, ...) {}
void Logger::error(const char*, ...) {}
void Logger::raw(const char*, ...) {}
void error_trap(const char*) { std::abort(); }
int IKinematicModel::get_joint_count() { return NUM_JOINTS; }
bool spin_try_lock_unsafe(int) { return true; }
void spin_lock_unsafe_blocking(int) {}
void spin_unlock_unsafe(int) {}
constexpr float CALIBRATION_RANGE = 83.0f;
constexpr float HOMING_USABLE_CLEARANCE_FRACTION = 0.75f;
constexpr float HOMING_MEASURED_BACKOFF_ANGLE_DEG = 2.5f;
constexpr float HOMING_FINISH_POSITION_DEG = 7.0f;
constexpr float JOINT_OPPOSITE_TRAVEL_MARGIN_DEG = 0.5f;
constexpr float JOINT_LIMIT_NUMERIC_TOLERANCE_DEG = 0.0001f;
enum class ERobotState { IDLE, EXECUTING_PATH };

struct JointHomeReference {
  bool valid = false;
  float final_position = 0.0f;
  float measured_clearance = 0.0f;
  float away_from_stop_sign = 0.0f;
};

struct Lut {
  std::vector<float> entries{0.0f, CALIBRATION_RANGE*Constants::DEG2RAD};
  float lower = 0.0f, upper = CALIBRATION_RANGE*Constants::DEG2RAD;
  uint32_t size() const { return entries.size(); }
  float get_entry(int i) const { return entries[i]; }
  bool is_monotonic() const {
    return std::is_sorted(entries.begin(), entries.end()) ||
           std::is_sorted(entries.rbegin(), entries.rend());
  }
  void get_intput_range(float& a, float& b) const { a = lower; b = upper; }
};
struct Servo {
  Lut encoder, field;
  const Lut& get_enc_to_pos_lut() const { return encoder; }
  const Lut& get_pos_to_field_lut() const { return field; }
};
struct Joint {
  bool is_homed = true, is_calibrated = true;
  Servo servo;
  Servo* servo_controller = &servo;
};
struct Tool : IRobotTool {
  float value = -1.0f;
  int writes = 0;
  void set_value(float v) override { value = v; writes++; }
};
class Robot {
 public:
  Robot() { for(int i=0; i<NUM_TOOLS; ++i) robot_tools[i] = &tool_storage[i]; }
  KinematicModel_Delta3D model;
  IKinematicModel* kinematic_model = &model;
  PathPlanner path_planner{kinematic_model, 0.005f};
  MotionController motion_controller{&path_planner};
  Joint storage[NUM_JOINTS];
  Joint* joints[NUM_JOINTS]{&storage[0], &storage[1], &storage[2]};
  bool all_joints_ready = true;
  JointHomeReference joint_home_references[NUM_JOINTS]{
    {true, 1.5f*Constants::DEG2RAD, 1.5f*Constants::DEG2RAD, 1.0f},
    {true, 1.5f*Constants::DEG2RAD, 1.5f*Constants::DEG2RAD, 1.0f},
    {true, 1.5f*Constants::DEG2RAD, 1.5f*Constants::DEG2RAD, 1.0f}
  };
  JointTravelLimits joint_limits;
  std::string last_pose_error;
  Pose6DF current_pose;
  float planned_joint_positions[NUM_JOINTS]{1, 2, 3};
  LinearAngular current_feedrate{10, 1}, max_acceleration{500, 50};
  ERobotState state = ERobotState::IDLE;
  float current_tool_outputs[NUM_TOOLS]{};
  Tool tool_storage[NUM_TOOLS];
  std::array<IRobotTool*, NUM_TOOLS> robot_tools{};
  struct { int lock = 0; float joint_target_positions[NUM_JOINTS]{1, 2, 3};
           float joint_target_velocities[NUM_JOINTS]{4, 5, 6}; } shared_data;
  bool calculate_joint_travel_limit(int, const JointHomeReference&, float&, float&) const;
  bool update_travel_limits();
  bool set_pose(const Pose6DF&, bool enforce_travel_limits = true);
  void process_motion_command(const GCodeCommand&, std::string&);
  void process_set_pose_command(const GCodeCommand&, std::string&);
  void process_dwell_command(const GCodeCommand&, std::string&);
  void process_tool_output_command(const GCodeCommand&, std::string&);
};

int main() {
  Robot robot;
  std::string reply;
  GCodeCommand cmd;
  assert(robot.update_travel_limits());
  float at_lower[NUM_JOINTS]{0.375f*Constants::DEG2RAD, 0.375f*Constants::DEG2RAD,
                             0.375f*Constants::DEG2RAD};
  assert(robot.joint_limits.contains(at_lower));
  robot.storage[0].servo.field.lower = 1.0f*Constants::DEG2RAD;
  robot.storage[0].servo.field.upper = 80.0f*Constants::DEG2RAD;
  assert(robot.update_travel_limits());
  assert(!robot.joint_limits.contains(at_lower)); // intersection includes field input range
  robot.storage[0].servo.field = Lut();
  for(int i=0; i<NUM_JOINTS; ++i) {
    robot.storage[i].is_homed = false;
    assert(!robot.update_travel_limits());
    cmd.from_command_str("G0 X0.1");
    robot.process_motion_command(cmd, reply);
    assert(reply.find("error") == 0 && robot.path_planner.input_queue_size() == 0);
    robot.storage[i].is_homed = true;
    robot.storage[i].is_calibrated = false;
    assert(!robot.update_travel_limits());
    robot.storage[i].is_calibrated = true;
  }
  robot.storage[0].servo.encoder.entries = {0, 1, 0.5f};
  assert(!robot.update_travel_limits());
  robot.storage[0].servo.encoder.entries = {0, std::numeric_limits<float>::quiet_NaN()};
  assert(!robot.update_travel_limits());
  robot.storage[0].servo.encoder = Lut();
  robot.storage[0].servo.field.lower = std::numeric_limits<float>::quiet_NaN();
  assert(!robot.update_travel_limits());
  robot.storage[0].servo.field = Lut();
  // Decreasing encoder input-to-position tables are legitimate too.
  std::reverse(robot.storage[0].servo.encoder.entries.begin(), robot.storage[0].servo.encoder.entries.end());
  assert(robot.update_travel_limits());

  // Calibration starts after its own stop clearance, so a Home ending near
  // zero cannot support a 1 mm Cartesian jog in every direction: delta
  // kinematics may need one joint to move roughly six degrees toward Home.
  // The post-Home 7-degree target provides that room without extrapolating a
  // lookup table or shortening the requested Cartesian displacement.
  for(int axis=0; axis<NUM_JOINTS; ++axis) {
    for(float direction : {-1.0f, 1.0f}) {
      Robot jog;
      float finish[NUM_JOINTS]{
        HOMING_FINISH_POSITION_DEG*Constants::DEG2RAD,
        HOMING_FINISH_POSITION_DEG*Constants::DEG2RAD,
        HOMING_FINISH_POSITION_DEG*Constants::DEG2RAD
      };
      for(int i=0; i<NUM_JOINTS; ++i) {
        jog.storage[i].servo.field.lower = 0.026827f*Constants::DEG2RAD;
        jog.joint_home_references[i].final_position = 1.0f*Constants::DEG2RAD;
        jog.joint_home_references[i].measured_clearance =
          HOMING_MEASURED_BACKOFF_ANGLE_DEG*Constants::DEG2RAD;
      }
      assert(jog.model.foreward(finish, jog.current_pose));
      Pose6DF requested = jog.current_pose;
      float& coordinate = axis==0 ? requested.translation.x :
                          axis==1 ? requested.translation.y : requested.translation.z;
      coordinate += direction;
      std::string command = "G0 X" + std::to_string(requested.translation.x) +
                            " Y" + std::to_string(requested.translation.y) +
                            " Z" + std::to_string(requested.translation.z) + " F5";
      assert(cmd.from_command_str(command.c_str()) == GCodeCommand::EParseStatus::OK);
      jog.process_motion_command(cmd, reply);
      assert(reply == "ok\n" && jog.path_planner.input_queue_size() == 1);
      assert(fabsf(jog.current_pose.translation.x-requested.translation.x) < 1e-5f);
      assert(fabsf(jog.current_pose.translation.y-requested.translation.y) < 1e-5f);
      assert(fabsf(jog.current_pose.translation.z-requested.translation.z) < 1e-5f);
    }
  }

  for(const char* command : {"G0 X10 F100 I", "G1 Y10 F100 I", "G0 Z10 F100 I",
                            "G1 X-11", "G0 Y-11", "G1 Z-11", "G0 X1000", "G0 X0.1 F0"}) {
    assert(cmd.from_command_str(command) == GCodeCommand::EParseStatus::OK);
    robot.process_motion_command(cmd, reply);
    assert(reply.find("error") == 0);
    assert(robot.path_planner.input_queue_size() == 0);
    assert(robot.current_pose.translation.sqr_length() == 0);
    assert(robot.current_feedrate.linear == 10 && robot.state == ERobotState::IDLE);
  }
  for(const char* command : {"G24 X10", "G24 Y10", "G24 Z10", "G24 X-11", "G24 Y-11", "G24 Z-11", "G24 X1000"}) {
    cmd.from_command_str(command);
    robot.process_set_pose_command(cmd, reply);
    assert(reply.find("error") == 0);
    assert(robot.current_pose.translation.sqr_length() == 0);
    for(int i=0; i<NUM_JOINTS; ++i) {
      assert(robot.shared_data.joint_target_positions[i] == float(i+1));
      assert(robot.shared_data.joint_target_velocities[i] == float(i+4));
    }
  }
  for(const char* command : {"G24 X0.1", "G24 Y0.1", "G24 Z0.1"}) {
    cmd.from_command_str(command);
    robot.process_set_pose_command(cmd, reply);
    assert(reply == "ok\n");
    assert(robot.joint_limits.contains(robot.shared_data.joint_target_positions));
  }
  cmd.from_command_str("G0 X0 Y0 Z0 F5");
  robot.process_motion_command(cmd, reply);
  assert(reply == "ok\n" && robot.current_feedrate.linear == 5);
  cmd.from_command_str("G24 X0.1");
  robot.process_set_pose_command(cmd, reply);
  assert(reply == "busy\n");
  robot.path_planner.abort();
  robot.process_set_pose_command(cmd, reply);
  assert(reply.find("error: motion limit fault") == 0);

  Robot dwell;
  // Even an invalid Cartesian pose must not make a zero-motion dwell invoke IK.
  dwell.current_pose.translation.x = std::numeric_limits<float>::quiet_NaN();
  cmd.from_command_str("G4 S0.01");
  dwell.process_dwell_command(cmd, reply);
  assert(reply == "ok\n" && dwell.path_planner.input_queue_size() == 1);
  dwell.path_planner.process(false);
  float dwell_positions[NUM_JOINTS], dwell_velocities[NUM_JOINTS], dwell_tools[NUM_TOOLS];
  bool enforce_limits = true;
  assert(dwell.motion_controller.update(0.001f, dwell_positions, dwell_velocities,
                                        dwell_tools, &enforce_limits));
  assert(!enforce_limits);
  for(int i=0; i<NUM_JOINTS; ++i)
    assert(dwell_positions[i] == dwell.planned_joint_positions[i]);

  Robot ordered_dwell;
  assert(ordered_dwell.update_travel_limits());
  cmd.from_command_str("G0 X0.1 F5");
  ordered_dwell.process_motion_command(cmd, reply);
  assert(reply == "ok\n");
  float accepted_endpoint[NUM_JOINTS];
  for(int i=0; i<NUM_JOINTS; ++i)
    accepted_endpoint[i] = ordered_dwell.planned_joint_positions[i];
  cmd.from_command_str("G4 S0.01");
  ordered_dwell.process_dwell_command(cmd, reply);
  assert(reply == "ok\n" && ordered_dwell.path_planner.input_queue_size() == 2);
  bool saw_ordered_dwell = false;
  for(int iteration=0; iteration<10000 && !ordered_dwell.path_planner.all_finished(); ++iteration) {
    ordered_dwell.path_planner.process(false);
    JointSpacePathSegment segment;
    while(ordered_dwell.path_planner.pop_js_path_segment(segment)) {
      if(!segment.requires_joint_limits()) {
        saw_ordered_dwell = true;
        for(int i=0; i<NUM_JOINTS; ++i) {
          assert(fabsf(segment.start_pos[i]-accepted_endpoint[i]) < 1e-7f);
          assert(fabsf(segment.end_pos[i]-accepted_endpoint[i]) < 1e-7f);
        }
      }
    }
  }
  assert(saw_ordered_dwell && ordered_dwell.path_planner.all_finished());

  Robot faulted_tool;
  faulted_tool.path_planner.abort();
  cmd.from_command_str("M3 T0 S0.5");
  faulted_tool.process_tool_output_command(cmd, reply);
  assert(reply.find("error: motion limit fault") == 0 &&
         faulted_tool.tool_storage[0].writes == 0 &&
         faulted_tool.current_tool_outputs[0] == 0.0f);
  cmd.from_command_str("M3 T0 S0");
  faulted_tool.process_tool_output_command(cmd, reply);
  assert(reply == "ok\n" && faulted_tool.tool_storage[0].writes == 1 &&
         faulted_tool.tool_storage[0].value == 0.0f);
  cmd.from_command_str("G4 S0.001");
  faulted_tool.process_dwell_command(cmd, reply);
  assert(reply.find("error: motion limit fault") == 0);

  std::puts("Actual Robot commands: G0/G1/G24 rejection is atomic; G4 remains non-motion before Home; faulted tool-off is immediate");
}
