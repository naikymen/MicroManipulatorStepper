#!/usr/bin/env bash
set -euo pipefail
test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_tmp="$(mktemp -d)"
trap 'rm -f -- "$test_tmp/motion_limits"; rmdir -- "$test_tmp"' EXIT
g++ -std=c++17 -Wall -Wextra -include algorithm \
  -I "$test_root/test/host_stubs" -I "$test_root/src" \
  "$test_root/test/host_motion_limits.cpp" \
  "$test_root/src/motion_control/motion_limits.cpp" \
  "$test_root/src/motion_control/path_segment.cpp" \
  "$test_root/src/motion_control/path_planner.cpp" \
  "$test_root/src/motion_control/motion_controller.cpp" \
  "$test_root/src/kinematic_models/kinematic_model_delta3d.cpp" \
  -o "$test_tmp/motion_limits"
"$test_tmp/motion_limits"
{
  sed -n '1,/^int main()/p' "$test_root/test/host_motion_commands.cpp" | sed '$d'
  for method in 'bool Robot::calculate_joint_travel_limit(' 'bool Robot::update_travel_limits(' 'bool Robot::set_pose(' \
                'void Robot::process_motion_command(' 'void Robot::process_set_pose_command(' \
                'void Robot::process_dwell_command(' 'void Robot::process_tool_output_command('; do
    sed -n "/^$method/,/^}/p" "$test_root/src/robot.cpp"
  done
  sed -n '/^int main()/,$p' "$test_root/test/host_motion_commands.cpp"
} | g++ -std=c++17 -Wall -Wextra -include algorithm \
  -I "$test_root/test/host_stubs" -I "$test_root/src" -x c++ - \
  "$test_root/src/motion_control/motion_limits.cpp" \
  "$test_root/src/motion_control/path_segment.cpp" \
  "$test_root/src/motion_control/path_planner.cpp" \
  "$test_root/src/motion_control/motion_controller.cpp" \
  "$test_root/src/command_parser/command_parser.cpp" \
  "$test_root/src/kinematic_models/kinematic_model_delta3d.cpp" \
  -o "$test_tmp/motion_limits"
"$test_tmp/motion_limits"
