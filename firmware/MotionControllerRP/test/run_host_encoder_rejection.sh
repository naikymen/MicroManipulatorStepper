#!/usr/bin/env bash
set -euo pipefail
test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_tmp="$(mktemp -d)"
trap 'rm -f -- "$test_tmp/check"; rmdir -- "$test_tmp"' EXIT
{
  # Only this harness exposes private state to check that a rejected packet
  # changes neither the last accepted raw reading nor its revolution count.
  sed -n '1,/^\/\/ ENCODER_METHODS/p' "$test_root/test/host_encoder_rejection.cpp" | sed '$d'
  sed '/^#pragma once/d; /^#include/d; s/^  private:/  public:/' "$test_root/src/hardware/MT6835_encoder.h"
  sed -n '/^MT6835Encoder::AbsRawAngleType MT6835Encoder::read_abs_angle_raw()/,/^}/p' "$test_root/src/hardware/MT6835_encoder.cpp"
  sed -n '/^MT6835Encoder::AbsRawAngleType MT6835Encoder::update_abs_raw_angle(/,/^}/p' "$test_root/src/hardware/MT6835_encoder.cpp"
  sed -n '/^uint8_t MT6835Encoder::calc_crc(/,/^}/p' "$test_root/src/hardware/MT6835_encoder.cpp"
  sed -n '/^\/\/ ENCODER_METHODS/,$p' "$test_root/test/host_encoder_rejection.cpp"
} | g++ -std=c++17 -DSERVO_IDLE_DIAGNOSTIC -x c++ - -o "$test_tmp/check"
"$test_tmp/check"
printf '%s\n' 'PASS: actual encoder reader rejects bad CRC without corrupting raw history or revolution counting, including wraparound.'
