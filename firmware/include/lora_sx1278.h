#pragma once

#include <stddef.h>
#include <stdint.h>
#include "config.h"

namespace flightcore {

/**
 * Abstract SPI + GPIO transport for SX1278 LoRa chip, allowing hardware SPI
 * on ESP32 and deterministic mock register testing in native C++ unit tests.
 */
class ISx1278Transport {
public:
    virtual ~ISx1278Transport() = default;
    virtual bool begin() = 0;
    virtual void reset_pulse() = 0;
    virtual uint8_t read_register(uint8_t reg) = 0;
    virtual void write_register(uint8_t reg, uint8_t value) = 0;
    virtual void write_fifo(const uint8_t* data, size_t len) = 0;
    virtual void read_fifo(uint8_t* out, size_t len) = 0;
    virtual bool read_dio0() const = 0;
};

struct LoraDiagnostics {
    uint8_t chip_version = 0;
    bool initialized = false;
    bool tx_busy = false;
    uint32_t tx_attempts = 0;
    uint32_t tx_success = 0;
    uint32_t tx_timeouts = 0;
    uint32_t tx_retries_total = 0;
    uint8_t last_tx_retries = 0;
    uint32_t rx_packets = 0;
    int8_t last_rssi_dbm = 0;
    float last_snr_db = 0.0f;
    uint32_t spi_faults = 0;
};

enum class LoraTxState : uint8_t {
    IDLE = 0,
    TRANSMITTING,
    RETRY_BACKOFF
};

/**
 * Register-level SX1278 433 MHz LoRa Transceiver Driver with non-blocking TX
 * completion monitoring, retry handling, RSSI/SNR link telemetry, and RX polling.
 */
class LoraSx1278 {
public:
    explicit LoraSx1278(ISx1278Transport* transport = nullptr);

    void set_transport(ISx1278Transport* transport) { transport_ = transport; }

    bool begin(uint32_t freq_hz = LORA_FREQ_HZ,
               uint8_t tx_power_dbm = LORA_TX_POWER_DBM,
               uint8_t sf = LORA_SPREADING_FACTOR,
               uint32_t bw_hz = LORA_BANDWIDTH_HZ,
               uint8_t cr = LORA_CODING_RATE,
               uint8_t sync_word = LORA_SYNC_WORD);

    /**
     * Queues a packet (up to 255 bytes) for non-blocking LoRa transmission.
     * Returns false if radio is uninitialized, busy, or payload is invalid.
     */
    bool start_transmit(const uint8_t* data, size_t len, uint32_t now_ms);

    /**
     * Services TX completion, timeout detection, automatic retries, and RX mode return.
     * Call frequently from the main loop.
     */
    void poll(uint32_t now_ms);

    /**
     * Checks if an incoming LoRa packet (e.g. ground station ACK/command) arrived.
     * Returns byte count read into out_buf (0 if none).
     */
    size_t read_received_packet(uint8_t* out_buf, size_t max_len);

    bool is_healthy() const { return diag_.initialized && diag_.spi_faults < 5; }
    bool is_busy() const { return tx_state_ != LoraTxState::IDLE; }
    const LoraDiagnostics& diagnostics() const { return diag_; }

    // Exposed SX1278 Register Constants for verification & tests
    static constexpr uint8_t REG_FIFO = 0x00;
    static constexpr uint8_t REG_OP_MODE = 0x01;
    static constexpr uint8_t REG_FRF_MSB = 0x06;
    static constexpr uint8_t REG_FRF_MID = 0x07;
    static constexpr uint8_t REG_FRF_LSB = 0x08;
    static constexpr uint8_t REG_PA_CONFIG = 0x09;
    static constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
    static constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
    static constexpr uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
    static constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
    static constexpr uint8_t REG_IRQ_FLAGS = 0x12;
    static constexpr uint8_t REG_RX_NB_BYTES = 0x13;
    static constexpr uint8_t REG_PKT_SNR_VALUE = 0x19;
    static constexpr uint8_t REG_PKT_RSSI_VALUE = 0x1A;
    static constexpr uint8_t REG_MODEM_CONFIG_1 = 0x1D;
    static constexpr uint8_t REG_MODEM_CONFIG_2 = 0x1E;
    static constexpr uint8_t REG_PAYLOAD_LENGTH = 0x22;
    static constexpr uint8_t REG_MODEM_CONFIG_3 = 0x26;
    static constexpr uint8_t REG_SYNC_WORD = 0x39;
    static constexpr uint8_t REG_DIO_MAPPING_1 = 0x40;
    static constexpr uint8_t REG_VERSION = 0x42;
    static constexpr uint8_t EXPECTED_VERSION = 0x12;

    static constexpr uint8_t MODE_LONG_RANGE = 0x80;
    static constexpr uint8_t MODE_SLEEP = 0x00;
    static constexpr uint8_t MODE_STDBY = 0x01;
    static constexpr uint8_t MODE_TX = 0x03;
    static constexpr uint8_t MODE_RX_CONTINUOUS = 0x05;

    static constexpr uint8_t IRQ_TX_DONE_MASK = 0x08;
    static constexpr uint8_t IRQ_RX_DONE_MASK = 0x40;
    static constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20;

private:
    ISx1278Transport* transport_;
    LoraDiagnostics diag_;
    LoraTxState tx_state_;
    uint8_t tx_buf_[255];
    size_t tx_len_;
    uint32_t tx_start_ms_;
    uint32_t retry_at_ms_;
    uint8_t current_retry_;

    void set_frequency(uint32_t freq_hz);
    void set_tx_power(uint8_t dbm);
    void configure_modem(uint8_t sf, uint32_t bw_hz, uint8_t cr);
    void enter_rx_continuous();
    void dispatch_fifo_tx(uint32_t now_ms);
};

#ifdef ARDUINO
/**
 * Default ESP32 VSPI transport for SX1278.
 */
class Esp32Sx1278Transport : public ISx1278Transport {
public:
    explicit Esp32Sx1278Transport(uint8_t sck = PIN_LORA_SCK,
                                  uint8_t miso = PIN_LORA_MISO,
                                  uint8_t mosi = PIN_LORA_MOSI,
                                  uint8_t cs = PIN_LORA_CS,
                                  uint8_t rst = PIN_LORA_RST,
                                  uint8_t dio0 = PIN_LORA_DIO0);

    bool begin() override;
    void reset_pulse() override;
    uint8_t read_register(uint8_t reg) override;
    void write_register(uint8_t reg, uint8_t value) override;
    void write_fifo(const uint8_t* data, size_t len) override;
    void read_fifo(uint8_t* out, size_t len) override;
    bool read_dio0() const override;

private:
    uint8_t sck_, miso_, mosi_, cs_, rst_, dio0_;
};
#endif

}  // namespace flightcore
