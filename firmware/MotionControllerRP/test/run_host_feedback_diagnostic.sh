#!/usr/bin/env bash
set -euo pipefail
test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_tmp="$(mktemp -d)"
trap 'rm -f -- "$test_tmp/check"; rmdir -- "$test_tmp"' EXIT
{
  sed -n '1,/^\/\/ FIXTURE_METHODS/p' "$test_root/test/host_feedback_diagnostic.cpp" | sed '$d'
  sed '/^#include/d; /^#pragma once/d' "$test_root/src/servo_control/servo_controller.h"
  sed -n '/^void ServoController::update(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
  sed -n '/^void ServoController::record_feedback(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
  sed -n '/^void ServoController::stop_feedback_output(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
  sed -n '/^void ServoController::trip_feedback(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
  sed -n '/^void ServoController::set_motor_enabled(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
  sed -n '/^void ServoController::set_motor_update_enabled(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
  sed -n '/^void ServoController::start_homing_handover(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
  sed -n '/^\/\/ FIXTURE_METHODS/,/^\/\/ ROBOT_METHODS/p' "$test_root/test/host_feedback_diagnostic.cpp"
  sed -n '/^void Robot::update_servo_controllers(/,/^}/p' "$test_root/src/robot.cpp"
  sed -n '/^\/\/ TEST_MAIN/,$p' "$test_root/test/host_feedback_diagnostic.cpp"
} | g++ -std=c++17 -DSERVO_IDLE_DIAGNOSTIC -DHOMING_TRANSITIONAL_FIELD_HANDOVER \
      -I "$test_root/src" -x c++ - "$test_root/src/servo_control/pid.cpp" -o "$test_tmp/check"
"$test_tmp/check"
