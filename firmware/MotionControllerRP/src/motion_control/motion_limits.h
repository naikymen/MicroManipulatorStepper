#pragma once

#include "path_segment.h"
#include <cmath>
#include <string>

class IKinematicModel;

// Mechanical motor radians, not Cartesian coordinates or electrical field angles.
class JointTravelLimits {
 public:
  void set(int joint, float lower, float upper, float comparison_tolerance = 0.0f) {
    minimum[joint] = lower;
    maximum[joint] = upper;
    tolerance[joint] = comparison_tolerance;
    configured[joint] = std::isfinite(lower) && std::isfinite(upper) && lower < upper &&
                        std::isfinite(comparison_tolerance) && comparison_tolerance >= 0.0f;
  }

  bool contains(int joint, float position) const {
    return configured[joint] && std::isfinite(position) &&
           position >= minimum[joint] - tolerance[joint] &&
           position <= maximum[joint] + tolerance[joint];
  }
  bool contains(const float positions[NUM_JOINTS]) const {
    for(int i = 0; i < NUM_JOINTS; ++i) {
      if(!contains(i, positions[i])) return false;
    }
    return true;
  }
  bool is_configured(int joint) const { return configured[joint]; }
  float lower(int joint) const { return minimum[joint]; }
  float upper(int joint) const { return maximum[joint]; }
  float comparison_tolerance(int joint) const { return tolerance[joint]; }
  std::string describe_violation(const float positions[NUM_JOINTS]) const;

 private:
  float minimum[NUM_JOINTS]{};
  float maximum[NUM_JOINTS]{};
  float tolerance[NUM_JOINTS]{};
  bool configured[NUM_JOINTS]{};
};

bool checked_inverse(IKinematicModel& model, const Pose6DF& pose,
                     float positions[NUM_JOINTS]);
bool cartesian_path_within_limits(IKinematicModel& model,
                                 const CartesianPathSegment& path,
                                 const JointTravelLimits& limits,
                                 std::string* rejection = nullptr);
