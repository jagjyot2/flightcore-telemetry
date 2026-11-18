// Wokwi Custom Chip Stub for SX1278 433 MHz LoRa Transceiver (VSPI)
// Behavioral stub only: 433 MHz RF propagation is not electromagnetically simulated.

#include "wokwi-api.h"
#include <stdint.h>
#include <stdlib.h>

typedef struct {
  pin_t pin_nss;
  pin_t pin_rst;
  pin_t pin_dio0;
  spi_dev_t spi;
  uint8_t regs[128];
  uint8_t tx_buf[1];
  uint8_t active_reg;
  uint8_t expect_addr;
  uint8_t write_mode;
} sx1278_state_t;

static void on_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  sx1278_state_t *s = (sx1278_state_t *)user_data;
  if (count == 0) {
    return;
  }
  uint8_t b = buffer[0];
  if (s->expect_addr) {
    s->write_mode = (b & 0x80) ? 1 : 0;
    s->active_reg = b & 0x7F;
    s->expect_addr = 0;
    s->tx_buf[0] = s->regs[s->active_reg];
  } else {
    if (s->write_mode) {
      s->regs[s->active_reg] = b;
      // RegOpMode (0x01) -> TX mode (0x03) triggers TxDone IRQ on DIO0
      if (s->active_reg == 0x01 && (b & 0x07) == 0x03) {
        s->regs[0x12] |= 0x08;  // RegIrqFlags: TxDone
        pin_write(s->pin_dio0, HIGH);
      } else if (s->active_reg == 0x12) {
        s->regs[0x12] &= ~b;
        pin_write(s->pin_dio0, LOW);
      }
    }
    s->active_reg = (s->active_reg + 1) & 0x7F;
    s->tx_buf[0] = s->regs[s->active_reg];
  }
  if (pin_read(s->pin_nss) == LOW) {
    spi_start(s->spi, s->tx_buf, 1);
  }
}

static void on_nss_change(void *user_data, pin_t pin, uint32_t value) {
  sx1278_state_t *s = (sx1278_state_t *)user_data;
  (void)pin;
  if (value == LOW) {
    s->expect_addr = 1;
    s->tx_buf[0] = 0x00;
    spi_start(s->spi, s->tx_buf, 1);
  } else {
    spi_stop(s->spi);
  }
}

void chip_init(void) {
  sx1278_state_t *s = (sx1278_state_t *)calloc(1, sizeof(sx1278_state_t));
  s->regs[0x42] = 0x12;  // SX1278 RegVersion silicon ID
  s->pin_nss = pin_init("NSS", INPUT_PULLUP);
  s->pin_rst = pin_init("RST", INPUT_PULLUP);
  s->pin_dio0 = pin_init("DIO0", OUTPUT_LOW);

  const pin_watch_config_t nss_watch = {
    .edge = BOTH,
    .pin_change = on_nss_change,
    .user_data = s,
  };
  pin_watch(s->pin_nss, &nss_watch);

  const spi_config_t spi_cfg = {
    .sck = pin_init("SCK", INPUT),
    .mosi = pin_init("MOSI", INPUT),
    .miso = pin_init("MISO", OUTPUT),
    .mode = 0,
    .done = on_spi_done,
    .user_data = s,
  };
  s->spi = spi_init(&spi_cfg);
}
