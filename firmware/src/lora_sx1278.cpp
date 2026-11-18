#include "lora_sx1278.h"
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#endif

namespace flightcore {

LoraSx1278::LoraSx1278(ISx1278Transport* transport)
    : transport_(transport),
      tx_state_(LoraTxState::IDLE),
      tx_len_(0),
      tx_start_ms_(0),
      retry_at_ms_(0),
      current_retry_(0) {
    memset(tx_buf_, 0, sizeof(tx_buf_));
    diag_ = LoraDiagnostics{};
}

void LoraSx1278::set_frequency(uint32_t freq_hz) {
    // Frf = (Freq_Hz * 2^19) / 32,000,000 Hz
    const uint64_t frf = (static_cast<uint64_t>(freq_hz) << 19) / 32000000ULL;
    transport_->write_register(REG_FRF_MSB, static_cast<uint8_t>((frf >> 16) & 0xFFU));
    transport_->write_register(REG_FRF_MID, static_cast<uint8_t>((frf >> 8) & 0xFFU));
    transport_->write_register(REG_FRF_LSB, static_cast<uint8_t>(frf & 0xFFU));
}

void LoraSx1278::set_tx_power(uint8_t dbm) {
    if (dbm < 2) dbm = 2;
    if (dbm > 17) dbm = 17;  // Safe PA_BOOST linear max without +20dBm high-current mode
    // PA_BOOST (bit 7 = 1), MaxPower = 7 (0x70), OutputPower = dbm - 2
    const uint8_t pa_cfg = static_cast<uint8_t>(0x80U | 0x70U | ((dbm - 2U) & 0x0FU));
    transport_->write_register(REG_PA_CONFIG, pa_cfg);
}

void LoraSx1278::configure_modem(uint8_t sf, uint32_t bw_hz, uint8_t cr) {
    if (sf < 6) sf = 6;
    if (sf > 12) sf = 12;
    if (cr < 5) cr = 5;
    if (cr > 8) cr = 8;

    uint8_t bw_bits = 0x07;  // 125 kHz default
    if (bw_hz <= 7800) bw_bits = 0x00;
    else if (bw_hz <= 10400) bw_bits = 0x01;
    else if (bw_hz <= 15600) bw_bits = 0x02;
    else if (bw_hz <= 20800) bw_bits = 0x03;
    else if (bw_hz <= 31250) bw_bits = 0x04;
    else if (bw_hz <= 41700) bw_bits = 0x05;
    else if (bw_hz <= 62500) bw_bits = 0x06;
    else if (bw_hz <= 125000) bw_bits = 0x07;
    else if (bw_hz <= 250000) bw_bits = 0x08;
    else bw_bits = 0x09;

    const uint8_t cr_bits = static_cast<uint8_t>(cr - 4U);
    const uint8_t mc1 = static_cast<uint8_t>((bw_bits << 4) | (cr_bits << 1));
    // Enable RxPayloadCrcOn (bit 2 = 0x04)
    const uint8_t mc2 = static_cast<uint8_t>((sf << 4) | 0x04U);
    // AgcAutoOn (bit 2 = 0x04); enable LowDataRateOptimize if symbol time >= 16ms
    uint8_t mc3 = 0x04;
    if (sf >= 11 && bw_hz <= 125000) {
        mc3 |= 0x08U;
    }

    transport_->write_register(REG_MODEM_CONFIG_1, mc1);
    transport_->write_register(REG_MODEM_CONFIG_2, mc2);
    transport_->write_register(REG_MODEM_CONFIG_3, mc3);
}

void LoraSx1278::enter_rx_continuous() {
    transport_->write_register(REG_DIO_MAPPING_1, 0x00);  // DIO0 -> RxDone
    transport_->write_register(REG_FIFO_ADDR_PTR, 0x00);
    transport_->write_register(REG_OP_MODE, static_cast<uint8_t>(MODE_LONG_RANGE | MODE_RX_CONTINUOUS));
}

bool LoraSx1278::begin(uint32_t freq_hz,
                       uint8_t tx_power_dbm,
                       uint8_t sf,
                       uint32_t bw_hz,
                       uint8_t cr,
                       uint8_t sync_word) {
    diag_ = LoraDiagnostics{};
    tx_state_ = LoraTxState::IDLE;

    if (!transport_ || !transport_->begin()) {
        return false;
    }

    transport_->reset_pulse();
    const uint8_t version = transport_->read_register(REG_VERSION);
    diag_.chip_version = version;
    if (version != EXPECTED_VERSION) {
        ++diag_.spi_faults;
        return false;
    }

    // Enter LoRa sleep mode (MODE_SLEEP = 0x00) to enable LoRa bit (bit 7)
    transport_->write_register(REG_OP_MODE, MODE_LONG_RANGE);
    transport_->write_register(REG_OP_MODE, static_cast<uint8_t>(MODE_LONG_RANGE | MODE_STDBY));

    set_frequency(freq_hz);
    transport_->write_register(REG_FIFO_TX_BASE_ADDR, 0x00);
    transport_->write_register(REG_FIFO_RX_BASE_ADDR, 0x00);
    configure_modem(sf, bw_hz, cr);
    set_tx_power(tx_power_dbm);
    transport_->write_register(REG_SYNC_WORD, sync_word);
    transport_->write_register(REG_IRQ_FLAGS, 0xFF);  // Clear all pending IRQs

    enter_rx_continuous();
    diag_.initialized = true;
    return true;
}

void LoraSx1278::dispatch_fifo_tx(uint32_t now_ms) {
    transport_->write_register(REG_OP_MODE, static_cast<uint8_t>(MODE_LONG_RANGE | MODE_STDBY));
    transport_->write_register(REG_IRQ_FLAGS, 0xFF);
    transport_->write_register(REG_FIFO_ADDR_PTR, 0x00);
    transport_->write_fifo(tx_buf_, tx_len_);
    transport_->write_register(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(tx_len_));
    transport_->write_register(REG_DIO_MAPPING_1, 0x40);  // DIO0 -> TxDone
    transport_->write_register(REG_OP_MODE, static_cast<uint8_t>(MODE_LONG_RANGE | MODE_TX));

    tx_state_ = LoraTxState::TRANSMITTING;
    tx_start_ms_ = now_ms;
    diag_.tx_busy = true;
}

bool LoraSx1278::start_transmit(const uint8_t* data, size_t len, uint32_t now_ms) {
    if (!diag_.initialized || !transport_ || !data || len == 0 || len > sizeof(tx_buf_)) {
        return false;
    }
    if (tx_state_ != LoraTxState::IDLE) {
        return false;
    }

    memcpy(tx_buf_, data, len);
    tx_len_ = len;
    current_retry_ = 0;
    ++diag_.tx_attempts;
    dispatch_fifo_tx(now_ms);
    return true;
}

void LoraSx1278::poll(uint32_t now_ms) {
    if (!diag_.initialized || !transport_) {
        return;
    }

    if (tx_state_ == LoraTxState::TRANSMITTING) {
        const uint8_t irq = transport_->read_register(REG_IRQ_FLAGS);
        if ((irq & IRQ_TX_DONE_MASK) != 0U || transport_->read_dio0()) {
            transport_->write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
            ++diag_.tx_success;
            diag_.last_tx_retries = current_retry_;
            diag_.tx_busy = false;
            tx_state_ = LoraTxState::IDLE;
            enter_rx_continuous();
            return;
        }

        if ((now_ms - tx_start_ms_) >= LORA_TX_TIMEOUT_MS) {
            ++diag_.tx_timeouts;
            if (current_retry_ < LORA_MAX_TX_RETRIES) {
                ++current_retry_;
                ++diag_.tx_retries_total;
                tx_state_ = LoraTxState::RETRY_BACKOFF;
                retry_at_ms_ = now_ms + 25U * current_retry_;
            } else {
                diag_.last_tx_retries = current_retry_;
                diag_.tx_busy = false;
                tx_state_ = LoraTxState::IDLE;
                ++diag_.spi_faults;
                enter_rx_continuous();
            }
        }
    } else if (tx_state_ == LoraTxState::RETRY_BACKOFF) {
        if (static_cast<int32_t>(now_ms - retry_at_ms_) >= 0) {
            dispatch_fifo_tx(now_ms);
        }
    }
}

size_t LoraSx1278::read_received_packet(uint8_t* out_buf, size_t max_len) {
    if (!diag_.initialized || !transport_ || tx_state_ != LoraTxState::IDLE || !out_buf || max_len == 0) {
        return 0;
    }

    const uint8_t irq = transport_->read_register(REG_IRQ_FLAGS);
    if ((irq & IRQ_RX_DONE_MASK) == 0U) {
        return 0;
    }

    if ((irq & IRQ_PAYLOAD_CRC_ERROR_MASK) != 0U) {
        transport_->write_register(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK | IRQ_PAYLOAD_CRC_ERROR_MASK);
        return 0;
    }

    const uint8_t rx_bytes = transport_->read_register(REG_RX_NB_BYTES);
    const uint8_t curr_addr = transport_->read_register(REG_FIFO_RX_CURRENT_ADDR);
    transport_->write_register(REG_FIFO_ADDR_PTR, curr_addr);

    const size_t to_read = (rx_bytes < max_len) ? rx_bytes : max_len;
    transport_->read_fifo(out_buf, to_read);

    // SX1278 433 MHz LF band RSSI offset = -164 dBm
    const uint8_t raw_rssi = transport_->read_register(REG_PKT_RSSI_VALUE);
    const int8_t raw_snr = static_cast<int8_t>(transport_->read_register(REG_PKT_SNR_VALUE));
    diag_.last_snr_db = static_cast<float>(raw_snr) / 4.0f;
    int16_t rssi = static_cast<int16_t>(raw_rssi) - 164;
    if (rssi < -127) rssi = -127;
    if (rssi > 0) rssi = 0;
    diag_.last_rssi_dbm = static_cast<int8_t>(rssi);

    ++diag_.rx_packets;
    transport_->write_register(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);
    return to_read;
}

// ---------------------------------------------------------------------------
// Hardware ESP32 VSPI Transport
// ---------------------------------------------------------------------------
#ifdef ARDUINO

namespace {
SPISettings g_lora_spi_settings(4000000UL, MSBFIRST, SPI_MODE0);
}

Esp32Sx1278Transport::Esp32Sx1278Transport(uint8_t sck,
                                           uint8_t miso,
                                           uint8_t mosi,
                                           uint8_t cs,
                                           uint8_t rst,
                                           uint8_t dio0)
    : sck_(sck), miso_(miso), mosi_(mosi), cs_(cs), rst_(rst), dio0_(dio0) {}

bool Esp32Sx1278Transport::begin() {
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);
    pinMode(rst_, OUTPUT);
    digitalWrite(rst_, HIGH);
    pinMode(dio0_, INPUT);
    SPI.begin(sck_, miso_, mosi_, cs_);
    return true;
}

void Esp32Sx1278Transport::reset_pulse() {
    digitalWrite(rst_, LOW);
    delay(10);
    digitalWrite(rst_, HIGH);
    delay(10);
}

uint8_t Esp32Sx1278Transport::read_register(uint8_t reg) {
    SPI.beginTransaction(g_lora_spi_settings);
    digitalWrite(cs_, LOW);
    SPI.transfer(static_cast<uint8_t>(reg & 0x7FU));
    const uint8_t val = SPI.transfer(0x00);
    digitalWrite(cs_, HIGH);
    SPI.endTransaction();
    return val;
}

void Esp32Sx1278Transport::write_register(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(g_lora_spi_settings);
    digitalWrite(cs_, LOW);
    SPI.transfer(static_cast<uint8_t>(reg | 0x80U));
    SPI.transfer(value);
    digitalWrite(cs_, HIGH);
    SPI.endTransaction();
}

void Esp32Sx1278Transport::write_fifo(const uint8_t* data, size_t len) {
    SPI.beginTransaction(g_lora_spi_settings);
    digitalWrite(cs_, LOW);
    // SX1278 SPI write uses MSB=1; REG_FIFO is 0x00
    SPI.transfer(0x80U);
    for (size_t i = 0; i < len; ++i) {
        SPI.transfer(data[i]);
    }
    digitalWrite(cs_, HIGH);
    SPI.endTransaction();
}

void Esp32Sx1278Transport::read_fifo(uint8_t* out, size_t len) {
    SPI.beginTransaction(g_lora_spi_settings);
    digitalWrite(cs_, LOW);
    SPI.transfer(static_cast<uint8_t>(LoraSx1278::REG_FIFO & 0x7FU));
    for (size_t i = 0; i < len; ++i) {
        out[i] = SPI.transfer(0x00);
    }
    digitalWrite(cs_, HIGH);
    SPI.endTransaction();
}

bool Esp32Sx1278Transport::read_dio0() const {
    return digitalRead(dio0_) == HIGH;
}

#endif  // ARDUINO

}  // namespace flightcore
