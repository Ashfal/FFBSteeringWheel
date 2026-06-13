// =========================================================================
// Flash Storage — Persistent Calibration Data
// =========================================================================
// Saves the wheel center offset and pedal ADC ranges to the RP2040 flash.
// Uses the last sector of flash memory.
// flash_safe_execute is used to ensure Core 1 isn't accessing XIP memory
// while the flash is being erased/written.
// =========================================================================

#include "flash_storage.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include <cstring>

// Standard Pico flash starts at 0x10000000.
// Typical size is 2MB. We use the very last 4KB sector.
#define FLASH_TARGET_OFFSET (2048 * 1024 - FLASH_SECTOR_SIZE)
#define MAGIC_NUMBER        0xFEEDFACE
#define CURRENT_VERSION     1

// Pointer to the memory-mapped flash location
const FlashCalibrationData* flash_data_ptr = (const FlashCalibrationData*)(XIP_BASE + FLASH_TARGET_OFFSET);

// Wrapper for flash erase/write to be run by flash_safe_execute
struct FlashCmdArgs {
    uint32_t offset;
    const uint8_t* data;
    size_t length;
};

static void __not_in_flash_func(flash_write_wrapper)(void* param) {
    auto* args = static_cast<FlashCmdArgs*>(param);
    flash_range_erase(args->offset, FLASH_SECTOR_SIZE);
    flash_range_program(args->offset, args->data, FLASH_PAGE_SIZE);
}

bool FlashStorage::load(FlashCalibrationData& out_data) {
    // Read directly from memory-mapped flash
    memcpy(&out_data, flash_data_ptr, sizeof(FlashCalibrationData));

    // Validate
    if (out_data.magic != MAGIC_NUMBER) {
        return false;
    }
    if (out_data.version != CURRENT_VERSION) {
        // Handle migration if needed in the future
        return false;
    }
    if (out_data.crc32 != calculate_crc(out_data)) {
        return false; // Corrupted
    }

    return true;
}

bool FlashStorage::save(const FlashCalibrationData& data) {
    // We need a page-aligned buffer to write to flash
    uint8_t page_buf[FLASH_PAGE_SIZE] = {0};
    
    // Copy data into buffer and update CRC
    FlashCalibrationData* p = reinterpret_cast<FlashCalibrationData*>(page_buf);
    memcpy(p, &data, sizeof(FlashCalibrationData));
    
    p->magic = MAGIC_NUMBER;
    p->version = CURRENT_VERSION;
    p->crc32 = calculate_crc(*p);

    FlashCmdArgs args;
    args.offset = FLASH_TARGET_OFFSET;
    args.data = page_buf;
    args.length = FLASH_PAGE_SIZE;

    // Use flash_safe_execute to pause Core 1 and safely write to flash
    int result = flash_safe_execute(flash_write_wrapper, &args, 50);
    
    return (result == PICO_OK);
}

// Simple CRC32 implementation
uint32_t FlashStorage::calculate_crc(const FlashCalibrationData& data) const {
    static_assert(offsetof(FlashCalibrationData, crc32) == sizeof(FlashCalibrationData) - sizeof(uint32_t),
                  "crc32 must be the last field in FlashCalibrationData");
    uint32_t crc = 0xFFFFFFFF;
    
    // Calculate CRC over everything EXCEPT the crc32 field itself
    size_t len = sizeof(FlashCalibrationData) - sizeof(uint32_t);
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&data);
    
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }
    
    return ~crc;
}
