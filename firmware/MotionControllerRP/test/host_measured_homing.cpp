// Runner injects the real HomingController header and implementation, with
// deterministic clock/encoder/driver stand-ins. No device is accessed.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>
#include "hw_config.h"
#define LOG_DEBUG(...) ((void)0)
#define LOG_INFO(...) ((void)0)
#define LOG_WARNING(...) ((void)0)
#define LOG_ERROR(...) ((void)0)
uint64_t clock_us = 1000;
uint64_t time_us_64() { return clock_us; }
void sleep_ms(uint32_t ms) { clock_us += uint64_t(ms)*1000; }
constexpr float counts_to_motor = Constants::TWO_PI_F / 2097152 * ENCODER_ANGLE_TO_ROTOR_ANGLE;
struct Encoder {
  int32_t origin = 500000, raw = origin;
  uint8_t status = 0;
  bool moving_noise = false;
  float read_abs_angle() { return read_abs_angle_raw()*Constants::TWO_PI_F/2097152; }
  int32_t read_abs_angle_raw() { return raw + (moving_noise ? (clock_us/1000 % 2 ? 5000 : -5000) : 0); }
  int32_t get_last_abs_raw_angle() { return raw; }
  void reset_abs_angle_period() { raw %= 2097152; if(raw<0) raw+=2097152; origin=raw; }
  uint8_t get_status() { return status; }
  int32_t get_rawcounts_per_rev() { return 2097152; }
};
struct Lut {
  float min, max, a, b;
  bool monotonic = true;
  uint32_t size() const { return 2; }
  float get_entry(uint32_t i) const { return i ? b : a; }
  bool is_monotonic() const { return monotonic; }
  void get_intput_range(float& lo, float& hi) const { lo=min; hi=max; }
  bool in_input_range(float x) const { return x>=min && x<=max; }
};
struct Driver {
  Encoder* encoder;
  float field = 0, amplitude = .3f, physical = 0;
  float dead_field = 0, restore_shift = 0;
  bool frozen = false, reversed = false, follows_search = false;
  unsigned writes = 0, rotations = 0, restores = 0;
  float get_field_angle() { return field; }
  float get_amplitude() { return amplitude; }
  void update_raw() { encoder->raw = encoder->origin - int32_t(std::lround(physical/counts_to_motor)); }
  void set_field_angle(float next) {
    float delta = next-field;
    field = next;
    writes++;
    if(follows_search && delta < 0) physical -= delta/50;
    if(delta > 0 && !frozen) {
      float used = std::min(delta, dead_field);
      dead_field -= used;
      physical += (reversed ? -1 : 1)*(delta-used)/50;
    }
    update_raw();
  }
  void set_amplitude_smooth(float next, uint32_t ms) {
    if(next > amplitude) { restores++; physical += restore_shift; update_raw(); }
    amplitude = next;
    sleep_ms(ms);
  }
  void rotate_field(float delta, float velocity, const std::function<void()>& callback) {
    rotations++;
    set_field_angle(field+delta);
    if(callback) callback();
    sleep_ms(uint32_t(fabsf(delta/velocity)*1000));
  }
};
struct ServoController {
  Encoder encoder;
  Driver driver{&encoder};
  Lut enc_lut{}, field_lut{0, 83*Constants::DEG2RAD, 0, 83*Constants::DEG2RAD*50};
  float calibration_origin = .25f*Constants::DEG2RAD;
  ServoController() { rebuild_lut(); }
  void rebuild_lut() {
    enc_lut = Lut{encoder.origin-(calibration_origin+83*Constants::DEG2RAD)/counts_to_motor,
                  encoder.origin-calibration_origin/counts_to_motor, 83*Constants::DEG2RAD, 0};
  }
  Driver& get_motor_driver() { return driver; }
  Encoder& get_encoder() { return encoder; }
  float get_pole_pair_count() { return 50; }
  const Lut& get_enc_to_pos_lut() const { return enc_lut; }
  const Lut& get_pos_to_field_lut() const { return field_lut; }
  float encoder_angle_to_motor_pos(int32_t raw) { return (encoder.origin-raw)*counts_to_motor-calibration_origin; }
  void set_motor_enabled(bool enabled, bool) { driver.set_amplitude_smooth(enabled ? .3f : 0, 100); }
};
// The real class and methods are inserted here by the runner.
int main() {
  auto search = [](HomingController& home, ServoController& servo, bool measured=true,
                   float range=2*Constants::PI_F,
                   float clearance=HOMING_MEASURED_BACKOFF_ANGLE_DEG*Constants::DEG2RAD,
                   bool raw_encoder_backoff=false) {
    clock_us = 1000;
    home.start(&servo, -.2f, range, .15f, ENCODER_ANGLE_TO_ROTOR_ANGLE,
               clearance, false, measured, raw_encoder_backoff);
    for(unsigned i=0; !home.is_finished() && i<50000; ++i) { sleep_ms(1); home.update(); }
    assert(home.is_finished());
  };
  for(float dead_phase: {0.0f, .5f*Constants::PI_F, Constants::PI_F, 1.5f*Constants::PI_F}) {
    ServoController servo;
    servo.driver.dead_field = dead_phase;
    HomingController home;
    search(home, servo);
    assert(!home.is_successful()); // endstop alone is insufficient
    home.finalize();
    assert(home.is_successful());
    assert(servo.driver.rotations == 0); // never fixed-field rotate
    assert(servo.driver.physical >=
           (HOMING_MEASURED_BACKOFF_ANGLE_DEG-.001f)*Constants::DEG2RAD);
    assert(servo.driver.physical <
           (HOMING_MEASURED_BACKOFF_ANGLE_DEG+.02f)*Constants::DEG2RAD);
    float clearance, away_sign;
    assert(home.get_measured_clearance_at_raw(
        servo.encoder.read_abs_angle_raw(), clearance, away_sign));
    assert(clearance >=
           (HOMING_MEASURED_BACKOFF_ANGLE_DEG-.001f)*Constants::DEG2RAD &&
           away_sign == 1.0f);
    assert(servo.driver.amplitude == .3f);
    auto writes = servo.driver.writes;
    home.finalize();
    assert(servo.driver.writes == writes);
    assert(servo.enc_lut.a == 83*Constants::DEG2RAD && servo.enc_lut.b == 0);
  }
  puts("PASS: normal Home verifies clearance beyond the calibration origin across different stop phase lags.");
  {
    ServoController servo;
    servo.encoder.origin = servo.encoder.raw = 2; // cross zero while backing off
    servo.rebuild_lut();
    HomingController home;
    search(home, servo);
    home.finalize();
    assert(home.is_successful() && servo.encoder.raw < 0);
  }
  for(int fault=0; fault<6; ++fault) {
    ServoController servo;
    HomingController home;
    search(home, servo);
    if(fault==0) servo.driver.frozen=true;
    if(fault==1) servo.driver.reversed=true;
    if(fault==2) servo.encoder.status=8; // bad CRC
    if(fault==3) servo.driver.restore_shift=-.4f*Constants::DEG2RAD;
    if(fault==4) servo.driver.restore_shift=4*Constants::DEG2RAD;
    if(fault==5) servo.encoder.moving_noise=true;
    auto begin=clock_us;
    home.finalize();
    assert(!home.is_successful() && home.is_finished());
    assert(clock_us-begin < 5000000);
    auto writes = servo.driver.writes;
    home.finalize();
    assert(servo.driver.writes == writes);
  }
  puts("PASS: frozen/reversed/faulty/noisy encoder and restore displacement cannot falsely certify home.");
  {
    ServoController servo;
    // Calibration must not consume the existing LUT. Make it deliberately
    // unusable and verify raw encoder travel still establishes the origin.
    servo.enc_lut.monotonic=false;
    HomingController home;
    search(home, servo, true, 2*Constants::PI_F,
           CALIBRATION_BACKOFF_CLEARANCE_DEG*Constants::DEG2RAD, true);
    home.finalize();
    assert(home.is_successful());
    assert(servo.driver.rotations == 0);
    assert(servo.driver.physical >= 1.499f*Constants::DEG2RAD);
    assert(servo.driver.physical < 1.52f*Constants::DEG2RAD);
    assert(servo.driver.amplitude == .3f);
  }
  puts("PASS: calibration establishes its origin from measured raw travel without using a saved LUT.");
  {
    ServoController servo;
    servo.driver.follows_search=true;
    HomingController home;
    search(home, servo, true, .03f);
    assert(!home.is_successful());
    auto writes=servo.driver.writes;
    home.finalize();
    assert(servo.driver.writes == writes && servo.driver.rotations == 0);
  }
  {
    ServoController servo;
    servo.enc_lut.monotonic=false;
    HomingController home;
    search(home, servo);
    home.finalize();
    assert(!home.is_successful() && servo.driver.writes == 0);
  }
  {
    ServoController servo;
    HomingController home;
    clock_us=1000;
    home.start(&servo, -.2f, 2*Constants::PI_F, .15f, ENCODER_ANGLE_TO_ROTOR_ANGLE, 0);
    while(!home.is_finished()) { sleep_ms(1); home.update(); }
    float stop_field=servo.driver.field;
    home.finalize();
    assert(home.is_successful() && servo.driver.rotations == 1);
    assert(fabsf(servo.driver.field-stop_field-.5f*Constants::PI_F)<1e-6f);
  }
  puts("PASS: timeout is terminal; invalid G28 domains fail before motion; legacy fixed-field mode remains available.");
}
