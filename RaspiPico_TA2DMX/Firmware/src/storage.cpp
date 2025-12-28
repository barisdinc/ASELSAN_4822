#include "storage.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

// Flash'ın sonundan 4KB öncesi
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

void storage_init() {
    // Gerekirse varsayılanları kontrol et
}

void save_channel(ChannelConfig *cfg) {
    uint8_t buffer[FLASH_PAGE_SIZE];
    memset(buffer, 0, FLASH_PAGE_SIZE);
    memcpy(buffer, cfg, sizeof(ChannelConfig));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

void load_channel(ChannelConfig *cfg) {
    memcpy(cfg, flash_target_contents, sizeof(ChannelConfig));
}
