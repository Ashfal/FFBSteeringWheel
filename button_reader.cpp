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
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_RX,  GPIO_FUNC_SPI);

    // Latch pin — manual GPIO
    gpio_init(PIN_SPI_LATCH);
    gpio_set_dir(PIN_SPI_LATCH, GPIO_OUT);
    gpio_put(PIN_SPI_LATCH, 0);

    // Claim DMA channels
    dma_tx_chan_ = dma_claim_unused_channel(true);
    dma_rx_chan_ = dma_claim_unused_channel(true);
}

void ButtonReader::update() {
    // 1. Latch: pulse HIGH then LOW to capture parallel button states
    gpio_put(PIN_SPI_LATCH, 1);
    // Brief delay — HCF4021 needs ~100ns, busy_wait gives us at least 1µs
    busy_wait_us_32(1);
    gpio_put(PIN_SPI_LATCH, 0);

    // 2. SPI DMA read 2 bytes (16 bits = 2 shift registers)
    // TX: send dummy bytes to generate clock
    // RX: receive button data
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

    // Start both channels simultaneously
    dma_start_channel_mask((1u << dma_tx_chan_) | (1u << dma_rx_chan_));
    dma_channel_wait_for_finish_blocking(dma_rx_chan_);

    // 3. Assemble 16-bit value and invert (pull-up: 0 = pressed → 1 = pressed)
    uint16_t raw = static_cast<uint16_t>((rx_buf_[0] << 8) | rx_buf_[1]);
    raw = ~raw;  // Invert: 0=pressed becomes 1=pressed

    // 4. Store in rolling buffer
    history_[history_idx_] = raw;
    history_idx_ = (history_idx_ + 1) % DEBOUNCE_READS;

    // 5. Debounce: only report a button if all 3 reads agree
    debounced_ = history_[0] & history_[1] & history_[2];
}
