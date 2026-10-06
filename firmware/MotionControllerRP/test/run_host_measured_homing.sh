#!/usr/bin/env bash
set -euo pipefail
test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_tmp="$(mktemp -d)"
trap 'rm -f -- "$test_tmp/homing_check"; rmdir -- "$test_tmp"' EXIT
{
  sed -n '1,/^int main()/p' "$test_root/test/host_measured_homing.cpp" | sed '$d'
  sed '/^#include/d; /^#pragma once/d' "$test_root/src/servo_control/homing_controller.h"
  sed '/^#include/d' "$test_root/src/servo_control/homing_controller.cpp"
  sed -n '/^int main()/,$p' "$test_root/test/host_measured_homing.cpp"
} | g++ -std=c++17 -Wall -Wextra -I "$test_root/src" -x c++ - -o "$test_tmp/homing_check"
"$test_tmp/homing_check"
{
  sed -n '1,/^int main()/p' "$test_root/test/host_homing_handover.cpp" | sed '$d'
  sed -n '/^bool Robot::calculate_joint_travel_limit(/,/^}/p' "$test_root/src/robot.cpp"
  sed -n '/^bool Robot::finish_homing_handover(/,/^}/p' "$test_root/src/robot.cpp"
  sed -n '/^void Robot::update_servo_controllers(/,/^}/p' "$test_root/src/robot.cpp"
  sed -n '/^int main()/,$p' "$test_root/test/host_homing_handover.cpp"
} | g++ -std=c++17 -Wall -Wextra -I "$test_root/src" -x c++ - -o "$test_tmp/homing_check"
"$test_tmp/homing_check"
