#!/usr/bin/env bash
set -euo pipefail
test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_tmp="$(mktemp -d)"
trap 'rm -f -- "$test_tmp/servo_check"; rmdir -- "$test_tmp"' EXIT
for mode in baseline bumpless; do
  extra_flags=()
  if [[ "$mode" == bumpless ]]; then
    extra_flags+=(-DHOMING_BUMPLESS_SERVO_RESTART)
  fi
  for duration in 50000 1000000 10000000; do
    {
      sed -n '1,/^int main()/p' "$test_root/test/host_servo_restart.cpp" | sed '$d'
      sed -n '/^void ServoController::update(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
      sed -n '/^void ServoController::set_motor_update_enabled(/,/^}/p' "$test_root/src/servo_control/servo_controller.cpp"
      sed -n '/^int main()/,$p' "$test_root/test/host_servo_restart.cpp"
    } | g++ -std=c++17 "${extra_flags[@]}" -DHOMING_SERVO_PULSE_US="$duration" \
        -I "$test_root/src" -x c++ - "$test_root/src/servo_control/pid.cpp" \
        -o "$test_tmp/servo_check"
    printf '%s restart, %s us observation window\n' "$mode" "$duration"
    "$test_tmp/servo_check"
  done
done
for mode in baseline guarded; do
  extra_flags=()
  if [[ "$mode" == guarded ]]; then
    extra_flags+=(-DHOMING_RESTART_GUARD)
  fi
  {
    sed -n '1,/^int main()/p' "$test_root/test/host_restart_guard.cpp" | sed '$d'
    sed -n '/^bool Robot::enable_servo_control(/,/^}/p' "$test_root/src/robot.cpp"
    sed -n '/^int main()/,$p' "$test_root/test/host_restart_guard.cpp"
  } | g++ -std=c++17 "${extra_flags[@]}" -I "$test_root/src" -x c++ - \
      -o "$test_tmp/servo_check"
  printf '%s restart preflight\n' "$mode"
  "$test_tmp/servo_check"
done
