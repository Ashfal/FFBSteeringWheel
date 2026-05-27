// =========================================================================
// I2C DMA Driver for AS5600
// =========================================================================
// RP2040 I2C DMA is complex because the I2C controller doesn't have 
// a simple block-read DMA interface. We must feed the DATA_CMD register
// with specific command words.
// =========================================================================

#include "i2c_dma.h"
#include "config.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

// I2C DATA_CMD register flags
#define I2C_CMD_RESTART 0x400
#define I2C_CMD_STOP    0x200
#define I2C_CMD_READ    0x100

void I2CDMA::init() {
    i2c_init(i2c0, I2C_FREQ_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    dma_tx_chan_ = dma_claim_unused_channel(true);
    dma_rx_chan_ = dma_claim_unused_channel(true);

    // Prepare the command buffer for reading 3 bytes starting from STATUS register (0x0B)
    tx_cmd_buf_[0] = AS5600_REG_STATUS;                                  // Write register address
    tx_cmd_buf_[1] = I2C_CMD_RESTART | I2C_CMD_READ;                     // Restart and read byte 1 (Status)
    tx_cmd_buf_[2] = I2C_CMD_READ;                                       // Read byte 2 (Angle High)
    tx_cmd_buf_[3] = I2C_CMD_STOP | I2C_CMD_READ;                        // Read byte 3 (Angle Low) and Stop

    // Configure TX DMA (writes commands to DATA_CMD)
    tx_cfg_ = dma_channel_get_default_config(dma_tx_chan_);
    channel_config_set_transfer_data_size(&tx_cfg_, DMA_SIZE_16);
    channel_config_set_dreq(&tx_cfg_, i2c_get_dreq(i2c0, true));
    channel_config_set_read_increment(&tx_cfg_, true);
    channel_config_set_write_increment(&tx_cfg_, false);

    // Configure RX DMA (reads data from DATA_CMD)
    // We only want to capture the 3 read bytes. The first command is a write, 
    // which doesn't produce RX data. 
    rx_cfg_ = dma_channel_get_default_config(dma_rx_chan_);
    channel_config_set_transfer_data_size(&rx_cfg_, DMA_SIZE_8); // We only care about the lower 8 bits
    channel_config_set_dreq(&rx_cfg_, i2c_get_dreq(i2c0, false));
    channel_config_set_read_increment(&rx_cfg_, false);
    channel_config_set_write_increment(&rx_cfg_, true);

    // Don't start yet
}

void I2CDMA::start_read() {
    // Set target address
    i2c_get_hw(i2c0)->enable = 0;
    i2c_get_hw(i2c0)->tar = AS5600_I2C_ADDR;
    i2c_get_hw(i2c0)->enable = 1;

    // We must abort any pending DMA and clear FIFOs
    dma_channel_abort(dma_tx_chan_);
    dma_channel_abort(dma_rx_chan_);
    while (i2c_get_hw(i2c0)->status & I2C_IC_STATUS_RFNE_BITS) {
        (void)i2c_get_hw(i2c0)->data_cmd;
    }

    dma_channel_configure(dma_rx_chan_, &rx_cfg_,
                          rx_buf_,                        // dest
                          &i2c_get_hw(i2c0)->data_cmd,    // src
                          3,                              // read 3 bytes
                          false);

    dma_channel_configure(dma_tx_chan_, &tx_cfg_,
                          &i2c_get_hw(i2c0)->data_cmd,    // dest
                          tx_cmd_buf_,                    // src
                          4,                              // 4 commands total
                          false);

    // Enable DMA IRQ for RX completion
    dma_channel_set_irq0_enabled(dma_rx_chan_, true);

    // Start both
    dma_start_channel_mask((1u << dma_tx_chan_) | (1u << dma_rx_chan_));
}

bool I2CDMA::handle_isr() {
    if (dma_channel_get_irq0_status(dma_rx_chan_)) {
        dma_channel_acknowledge_irq0(dma_rx_chan_);
        return true;
    }
    return false;
}

void I2CDMA::reset_bus() {
    // 1. Abort any hanging DMA
    dma_channel_abort(dma_tx_chan_);
    dma_channel_abort(dma_rx_chan_);

    // 2. Disable I2C peripheral to take manual control of pins
    i2c_deinit(i2c0);

    // 3. Configure pins as raw GPIO outputs to bit-bang a recovery
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_SIO);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_I2C_SCL, GPIO_OUT);
    gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
    gpio_pull_up(PIN_I2C_SDA); // Need pull up to read it

    // 4. Toggle SCL up to 9 times to force the stuck slave to release SDA
    for (int i = 0; i < 9; i++) {
        // If SDA goes high, the bus is free!
        if (gpio_get(PIN_I2C_SDA)) {
            break;
        }
        gpio_put(PIN_I2C_SCL, 0);
        sleep_us(5);
        gpio_put(PIN_I2C_SCL, 1);
        sleep_us(5);
    }

    // 5. Generate a hardware STOP condition (SDA low to high while SCL is high)
    gpio_set_dir(PIN_I2C_SDA, GPIO_OUT);
    gpio_put(PIN_I2C_SDA, 0);
    sleep_us(5);
    gpio_put(PIN_I2C_SCL, 1);
    sleep_us(5);
    gpio_put(PIN_I2C_SDA, 1);
    sleep_us(5);

    // 6. Re-initialize I2C peripheral to let the hardware take back control
    i2c_init(i2c0, I2C_FREQ_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}
