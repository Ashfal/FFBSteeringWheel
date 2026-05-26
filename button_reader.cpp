// =========================================================================
// Button Reader — SPI + DMA, 3-Read Debounce
// =========================================================================
// Reads 16 buttons from 2x HCF4021B shift registers via hardware SPI.
// Latch pulse → SPI DMA read 2 bytes → rolling buffer → debounce.
// Pull-ups: 0 = pressed, so we invert the result.
// =========================================================================

#include "button_reader.h"
#include "config.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

void ButtonReader::init() {
    // Initialize SPI0
    spi_init(spi0, SPI_FREQ_HZ);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_RX,  GPIO_FUNC_SPI);

    // Latch pin — manual GPIO
    gpio_init(PIN_SPI_LATCH);
    gpio_set_dir(PIN_SPI_LATCH, GPIO_OUT);
    gpio_put(PIN_SPI_LATCH, 0);

    // Claim DMA channels
    dma_tx_chan_ = dma_claim_unused_channel(true);
    dma_rx_chan_ = dma_claim_unused_channel(true);
    dma_channel_config tx_cfg = dma_channel_get_default_config(dma_tx_chan_);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&tx_cfg, spi_get_dreq(spi0, true));
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);

    dma_channel_config rx_cfg = dma_channel_get_default_config(dma_rx_chan_);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&rx_cfg, spi_get_dreq(spi0, false));
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);

    dma_channel_configure(dma_rx_chan_, &rx_cfg,
                          rx_buf_,                        // dest
                          &spi_get_hw(spi0)->dr,          // src
                          2,                               // count
                          false);                          // don't start yet

    dma_channel_configure(dma_tx_chan_, &tx_cfg,
                          &spi_get_hw(spi0)->dr,          // dest
                          tx_dummy_,                       // src
                          2,                               // count
                          false);                          // don't start yet
}

void ButtonReader::update() {
    if (dma_active_) {
        // Check if finished
        if (dma_channel_is_busy(dma_rx_chan_)) {
            return; // Still running, do nothing this frame
        }

        dma_active_ = false;

        // 3. Assemble 16-bit value and invert (pull-up: 0 = pressed → 1 = pressed)
        uint16_t raw = static_cast<uint16_t>((rx_buf_[0] << 8) | rx_buf_[1]);
        raw = ~raw;  // Invert: 0=pressed becomes 1=pressed

        // 4. Store in rolling buffer
        history_[history_idx_] = raw;
        history_idx_ = (history_idx_ + 1) % DEBOUNCE_READS;

        // 5. Debounce: require 3 consecutive identical reads to change state in either direction.
        debounced_ = (debounced_ & (history_[0] | history_[1] | history_[2])) | 
                     (history_[0] & history_[1] & history_[2]);
    }

    if (!dma_active_) {
        // 1. Latch: pulse HIGH then LOW to capture parallel button states
        gpio_put(PIN_SPI_LATCH, 1);
        // Brief delay — HCF4021 needs ~100ns, busy_wait gives us at least 1µs
        busy_wait_us_32(1);
        gpio_put(PIN_SPI_LATCH, 0);

        // 2. Restart DMA channels
        dma_channel_set_read_addr(dma_rx_chan_, &spi_get_hw(spi0)->dr, false);
        dma_channel_set_write_addr(dma_rx_chan_, rx_buf_, false);
        dma_channel_set_trans_count(dma_rx_chan_, 2, false);

        dma_channel_set_read_addr(dma_tx_chan_, tx_dummy_, false);
        dma_channel_set_write_addr(dma_tx_chan_, &spi_get_hw(spi0)->dr, false);
        dma_channel_set_trans_count(dma_tx_chan_, 2, false);

        // Start both channels simultaneously
        dma_start_channel_mask((1u << dma_tx_chan_) | (1u << dma_rx_chan_));
        dma_active_ = true;
    }
}
