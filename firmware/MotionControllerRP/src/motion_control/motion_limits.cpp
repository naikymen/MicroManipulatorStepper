#include "motion_limits.h"
#include "kinematic_models/kinematic_model_base.h"
#include <algorithm>
#include <limits>
#include "utilities/math_constants.h"

std::string JointTravelLimits::describe_violation(const float positions[NUM_JOINTS]) const {
  for(int i = 0; i < NUM_JOINTS; ++i) {
    std::string joint = "joint " + std::to_string(i+1);
    if(!configured[i]) return joint + " has no valid travel limits";
    if(!std::isfinite(positions[i])) return joint + " target is not finite";
    if(!contains(i, positions[i])) {
      return joint + " target " + std::to_string(positions[i]*Constants::RAD2DEG) +
             " deg outside " + std::to_string(minimum[i]*Constants::RAD2DEG) +
             ".." + std::to_string(maximum[i]*Constants::RAD2DEG) +
             " deg (numeric tolerance " +
             std::to_string(tolerance[i]*Constants::RAD2DEG) + " deg)";
    }
  }
  return "";
}

static bool finite_pose(const Pose6DF& pose) {
  const auto& p = pose.translation;
  const auto& q = pose.rotation;
  float norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(norm) && fabsf(norm-1.0f) < 0.001f;
}

bool checked_inverse(IKinematicModel& model, const Pose6DF& pose,
                     float positions[NUM_JOINTS]) {
  // Poison outputs so a failed or incomplete inverse cannot reuse stale targets.
  for(int i = 0; i < NUM_JOINTS; ++i)
    positions[i] = std::numeric_limits<float>::quiet_NaN();
  if(!finite_pose(pose) || !model.inverse(pose, positions)) return false;
  for(int i = 0; i < NUM_JOINTS; ++i)
    if(!std::isfinite(positions[i])) return false;
  return true;
}

bool cartesian_path_within_limits(IKinematicModel& model,
                                 const CartesianPathSegment& path,
                                 const JointTravelLimits& limits,
                                 std::string* rejection) {
  if(rejection) rejection->clear();
  if(!finite_pose(path.start_pose) || !finite_pose(path.end_pose)) {
    if(rejection) *rejection = "invalid Cartesian pose or rotation";
    return false;
  }
  float distance = (path.end_pose.translation-path.start_pose.translation).length();
  const auto& a = path.start_pose.rotation;
  const auto& b = path.end_pose.rotation;
  float dot = fabsf(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z);
  float angle = 2.0f*acosf(std::min(1.0f, dot));
  // Preflight the interior as well as endpoints, independent of feedrate and
  // look-ahead timing. Execution also checks EVERY generated joint segment:
  // this sampling alone is not a continuous-path safety proof.
  float count = ceilf(std::max(distance/0.01f, angle/0.001f));
  if(!std::isfinite(count) || count > 4096.0f) {
    if(rejection) *rejection = "path exceeds preflight sampling budget";
    return false;
  }
  int steps = std::max(1, int(count));
  for(int i = 0; i <= steps; ++i) {
    Pose6DF pose = Pose6DF::lerp(path.start_pose, path.end_pose, float(i)/steps);
    float positions[NUM_JOINTS];
    if(!checked_inverse(model, pose, positions)) {
      if(rejection) *rejection = "invalid inverse kinematics at path fraction " + std::to_string(float(i)/steps);
      return false;
    }
    if(!limits.contains(positions)) {
      if(rejection) *rejection = limits.describe_violation(positions) +
                                "; path fraction " + std::to_string(float(i)/steps);
      return false;
    }
  }
  return true;
}
