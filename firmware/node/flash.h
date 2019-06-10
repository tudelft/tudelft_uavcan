#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

void flash_erase_page(void *addr);
void* flash_erase_pages(void *start_addr, uint64_t size);
int flash_verify_block(void *adr, uint8_t *data, uint16_t len);
void flash_write_block(void *adr, uint8_t *data, uint16_t len);

#endif /* FLASH_H */