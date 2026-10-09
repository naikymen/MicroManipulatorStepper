// The runner inserts the real encoder declaration and reader before these
// fixtures. Only the SPI transport is mocked; no hardware is accessed.
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
using uint = unsigned int;
struct spi_inst_t {};
// ENCODER_METHODS
uint8_t packet[6]{};
MT6835Encoder::MT6835Encoder(spi_inst_t* instance, int32_t chip_select)
  : spi(instance), cs_pin(chip_select) {}
MT6835Encoder::~MT6835Encoder() {}
void MT6835Encoder::spi_begin_transaction() {}
void MT6835Encoder::spi_end_transaction() {}
void MT6835Encoder::spi_transfer(uint8_t* bytes, size_t size) {
  std::memcpy(bytes, packet, size);
}

int main() {
  MT6835Encoder encoder(nullptr, 0);
  encoder.check_crc = true;
  auto sample = [&](int32_t raw, bool valid) {
    packet[2] = raw >> 13;
    packet[3] = raw >> 5;
    packet[4] = (raw & 31) << 3;
    packet[5] = encoder.calc_crc(raw, 0) ^ (valid ? 0 : 1);
    return encoder.read_abs_angle_raw();
  };

  assert(sample(100, true) == 100);
  assert(sample(1800000, false) == 100);
  assert(encoder.last_raw_angle == 100 && encoder.abs_raw_angle == 100);
  assert(encoder.last_status == MT6835_CRC_ERROR && encoder.crc_error_count == 1);
  assert(sample(110, true) == 110 && encoder.last_status == 0);
  assert(sample(MT6835_CPR - 10, true) == -10);
  assert(sample(5, false) == -10 && encoder.last_raw_angle == MT6835_CPR - 10);
  assert(sample(6, true) == 6);
}
