// Wokwi Custom Chip Stub for VL53L0X Time-of-Flight Laser Ranging Sensor (I2C 0x29)
// Behavioral stub only: optical laser propagation is not ray-traced in Wokwi.

#include "wokwi-api.h"
#include <stdint.h>
#include <stdlib.h>

typedef struct {
  pin_t pin_xshut;
  pin_t pin_int;
  uint32_t attr_range_mm;
  uint8_t reg_addr;
  uint8_t expect_reg_addr;
  uint8_t range_hi;
  uint8_t range_lo;
} vl53l0x_state_t;

static bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  vl53l0x_state_t *s = (vl53l0x_state_t *)user_data;
  if (pin_read(s->pin_xshut) == LOW) {
    return false;
  }
  if (!read) {
    s->expect_reg_addr = 1;
  }
  return address == 0x29;
}

static uint8_t on_i2c_read(void *user_data) {
  vl53l0x_state_t *s = (vl53l0x_state_t *)user_data;
  uint8_t reg = s->reg_addr++;
  uint16_t mm = (uint16_t)attr_read(s->attr_range_mm);
  if (mm < 30 || mm > 2000) {
    mm = 850;
  }
  s->range_hi = (uint8_t)((mm >> 8) & 0xFF);
  s->range_lo = (uint8_t)(mm & 0xFF);

  switch (reg) {
    case 0xC0: return 0xEE;  // IDENTIFICATION_MODEL_ID
    case 0x13: return 0x04;  // RESULT_INTERRUPT_STATUS (new sample ready)
    case 0x1E: return s->range_hi;
    case 0x1F: return s->range_lo;
    default:   return 0x00;
  }
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  vl53l0x_state_t *s = (vl53l0x_state_t *)user_data;
  if (s->expect_reg_addr) {
    s->reg_addr = data;
    s->expect_reg_addr = 0;
  } else {
    if (s->reg_addr == 0x00 && (data & 0x01)) {
      pin_write(s->pin_int, LOW);
    } else if (s->reg_addr == 0x0B) {
      pin_write(s->pin_int, HIGH);
    }
    s->reg_addr++;
  }
  return true;
}

void chip_init(void) {
  vl53l0x_state_t *s = (vl53l0x_state_t *)calloc(1, sizeof(vl53l0x_state_t));
  s->pin_xshut = pin_init("XSHUT", INPUT_PULLUP);
  s->pin_int = pin_init("INT", OUTPUT_HIGH);
  s->attr_range_mm = attr_init("range_mm", 850);

  const i2c_config_t i2c_cfg = {
    .address = 0x29,
    .scl = pin_init("SCL", INPUT_PULLUP),
    .sda = pin_init("SDA", INPUT_PULLUP),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .user_data = s,
  };
  i2c_init(&i2c_cfg);
}
