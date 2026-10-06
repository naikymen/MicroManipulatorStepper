// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

//*** INCLUDE ***************************************************************************

#include "path_segment.h"

//*** CLASS *****************************************************************************

class PathPlanner;

//--- MotionController ------------------------------------------------------------------

class MotionController {
  public:
    MotionController(PathPlanner* path_planner);

    // updates the motion controller and computes new joint positions and velocities
    // after dt has passed. Ouput array must hav space for 'NUM_JOINTS' entries.
    bool update(float dt, float* joint_positions, float* joint_velocities,
                float* tool_outputs, bool* enforce_joint_limits = nullptr);
    void reset() { current_path_segment = JointSpacePathSegment(); current_time = 0.0f; }
    bool is_running() const { return current_path_segment.initialized; }

  private:
    PathPlanner* path_planner;

    float current_time;
    JointSpacePathSegment current_path_segment;
    bool current_segment_finished;
};
