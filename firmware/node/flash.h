#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

void flash_erase_page(void *addr);
uint32_t flash_erase_pages(uint32_t start_addr, uint64_t size);
int flash_verify_block(void *adr, uint8_t *data, uint16_t len);
void flash_write_block(uint32_t addr, const void *buf, uint16_t count);

#endif /* FLASH_H */