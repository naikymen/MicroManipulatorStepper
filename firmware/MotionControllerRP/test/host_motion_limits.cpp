#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include "motion_control/motion_limits.h"
#include "motion_control/path_planner.h"
#include "motion_control/motion_controller.h"
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
// Existing base implementation has no definition; Delta3D overrides this.
int IKinematicModel::get_joint_count() { return NUM_JOINTS; }

static CartesianPathSegment path(const Pose6DF& start, const Pose6DF& end) {
  float tools[NUM_TOOLS]{};
  return CartesianPathSegment(start, end, LinearAngular(5.0f, 1.0f),
                              LinearAngular(500.0f, 50.0f), tools);
}

class TestModel : public IKinematicModel {
 public:
  bool fail = false, nonfinite = false, partial = false, hump = false;
  int get_joint_count() override { return NUM_JOINTS; }
  bool foreward(const float*, Pose6DF&) override { return false; }
  bool inverse(const Pose6DF& pose, float* positions) override {
    positions[0] = 0.5f;
    if(fail) return false;
    if(hump) positions[0] += 0.25f*sinf(pose.translation.x*Constants::PI_F);
    if(nonfinite) positions[0] = std::numeric_limits<float>::quiet_NaN();
    if(partial) return true;
    positions[1] = positions[2] = 0.5f;
    return true;
  }
};

int main() {
  JointTravelLimits limits;
  float angles[NUM_JOINTS]{0.5f, 0.5f, 0.5f};
  assert(!limits.contains(angles)); // empty/unconfigured fails closed
  assert(limits.describe_violation(angles).find("joint 1") != std::string::npos);
  for(int i=0; i<NUM_JOINTS; ++i) limits.set(i, 0.1f, 0.7f);
  assert(limits.contains(angles));
  JointTravelLimits tolerant;
  const float comparison_tolerance = 0.0001f*Constants::DEG2RAD;
  for(int i=0; i<NUM_JOINTS; ++i)
    tolerant.set(i, 0.5f, 0.7f, comparison_tolerance);
  float near_boundary[NUM_JOINTS]{
    0.5f-0.000003f*Constants::DEG2RAD, 0.5f, 0.5f
  };
  assert(tolerant.contains(near_boundary));
  near_boundary[0] = 0.5f-0.0002f*Constants::DEG2RAD;
  assert(!tolerant.contains(near_boundary));
  for(int i=0; i<NUM_JOINTS; ++i) {
    for(float value : {0.09f, 0.71f, std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::infinity()}) {
      angles[i] = value;
      assert(!limits.contains(angles));
      assert(limits.describe_violation(angles).find("joint " + std::to_string(i+1)) != std::string::npos);
    }
    angles[i] = 0.5f;
  }

  TestModel test;
  Pose6DF origin, end(Vec3F(1, 0, 0), QuaternionF());
  assert(cartesian_path_within_limits(test, path(origin, end), limits));
  test.hump = true;
  // Endpoints safe, middle out of range: endpoint-only checks are insufficient.
  std::string reason;
  assert(!cartesian_path_within_limits(test, path(origin, end), limits, &reason));
  assert(reason.find("joint 1 target") != std::string::npos);
  assert(reason.find("path fraction") != std::string::npos);
  assert(reason.find("outside") != std::string::npos);
  test.hump = false;
  for(bool* flag : {&test.fail, &test.nonfinite, &test.partial}) {
    *flag = true;
    assert(!checked_inverse(test, origin, angles));
    assert(!cartesian_path_within_limits(test, path(origin, end), limits));
    *flag = false;
  }
  Pose6DF bad = end;
  bad.translation.y = std::numeric_limits<float>::infinity();
  assert(!checked_inverse(test, bad, angles));
  assert(!cartesian_path_within_limits(test, path(origin, bad), limits));
  bad = end;
  bad.rotation.w = 0;
  assert(!checked_inverse(test, bad, angles));
  assert(!cartesian_path_within_limits(test, path(origin, Pose6DF(Vec3F(100,0,0), QuaternionF())), limits));

  PathPlanner planner(&test, 0.005f);
  MotionController controller(&planner);
  assert(!planner.add_cartesian_path_segment(path(origin, end)));
  assert(planner.input_queue_size() == 0);

  // A dwell is a planner-ordered tool/time event, not a motion request. It
  // must preserve the supplied targets even before travel limits exist.
  float dwell_positions[NUM_JOINTS]{1.0f, 2.0f, 3.0f};
  float dwell_tools[NUM_TOOLS]{0.25f, 0.75f};
  CartesianPathSegment dwell(origin, dwell_tools, 0.01f, dwell_positions);
  assert(!dwell.joint_motion);
  assert(planner.add_cartesian_path_segment(dwell));
  test.fail = true; // dwell must not consult IK at acceptance or generation
  planner.process(false);
  bool enforce_limits = true;
  float dwell_velocities[NUM_JOINTS], observed_tools[NUM_TOOLS];
  assert(controller.update(0.001f, angles, dwell_velocities, observed_tools,
                           &enforce_limits));
  assert(!enforce_limits);
  for(int i=0; i<NUM_JOINTS; ++i) assert(angles[i] == dwell_positions[i]);
  for(int i=0; i<NUM_TOOLS; ++i) assert(observed_tools[i] == dwell_tools[i]);
  test.fail = false;
  planner.reset();
  controller.reset();

  planner.set_joint_limits(limits);
  assert(planner.add_cartesian_path_segment(path(origin, end)));
  test.fail = true; // unexpected execution-time IK failure after preflight
  planner.process(false);
  assert(planner.has_fault());
  JointSpacePathSegment segment;
  assert(!planner.pop_js_path_segment(segment));
  float velocities[NUM_JOINTS], tools[NUM_TOOLS];
  for(float& value : angles) value = 123.0f;
  assert(!controller.update(0.001f, angles, velocities, tools));
  for(float value : angles) assert(value == 123.0f); // no partial publication
  test.fail = false;
  assert(!planner.add_cartesian_path_segment(path(origin, end))); // latched
  planner.reset();
  controller.reset();
  assert(!planner.has_fault() && planner.all_finished());
  assert(planner.add_cartesian_path_segment(path(origin, end)));
  test.hump = true; // finite execution-time range violation after acceptance
  for(int step=0; step<1000 && !planner.has_fault(); ++step) {
    planner.process(false);
    if(planner.pop_js_path_segment(segment)) {
      assert(limits.contains(segment.start_pos) && limits.contains(segment.end_pos));
    }
  }
  assert(planner.has_fault());
  test.hump = false;
  planner.reset();
  assert(planner.add_cartesian_path_segment(path(origin, end)));
  while(!planner.all_finished()) {
    planner.process(false);
    if(planner.pop_js_path_segment(segment)) {
      for(int t=0; t<=10; ++t) {
        segment.evaluate(segment.duration*t/10, angles, velocities, tools);
        assert(limits.contains(angles));
      }
    }
  }

  KinematicModel_Delta3D delta;
  JointTravelLimits calibrated;
  for(int i=0; i<NUM_JOINTS; ++i)
    calibrated.set(i, 0.5f*Constants::DEG2RAD, 82.5f*Constants::DEG2RAD);
  assert(checked_inverse(delta, origin, angles) && calibrated.contains(angles));
  auto coordinate = [](Vec3F& p, int axis) -> float& {
    if(axis == 0) return p.x;
    if(axis == 1) return p.y;
    return p.z;
  };
  for(int axis=0; axis<NUM_JOINTS; ++axis) {
    for(float direction : {-1.0f, 1.0f}) {
      Pose6DF jog = origin;
      coordinate(jog.translation, axis) = direction*0.1f;
      assert(cartesian_path_within_limits(delta, path(origin, jog), calibrated));
      Pose6DF beyond = origin;
      coordinate(beyond.translation, axis) = direction > 0 ? 10.0f : -11.0f;
      assert(!cartesian_path_within_limits(delta, path(origin, beyond), calibrated));
      assert(!checked_inverse(delta, beyond, angles) || !calibrated.contains(angles));
    }
  }
  assert(!checked_inverse(delta, Pose6DF(Vec3F(1000,1000,1000), QuaternionF()), angles));
  std::puts("Motion limits: finite IK, all axes/both ends, interior rejection, generated-segment fault and valid paths passed");
}
