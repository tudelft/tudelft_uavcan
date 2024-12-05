#include "flash.h"

#include <ch.h>
#include <hal.h>

#if defined(STM32F10X_CL)
#define FLASH_PAGE_SIZE         2048
#endif

/* Some compiler options will convert short loads and stores into byte loads
 * and stores.  We don't want this to happen for IO reads and writes!
 */
/* # define getreg16(a)       (*(volatile uint16_t *)(a)) */
static inline uint16_t getreg16(unsigned int addr)
{
    uint16_t retval;
    __asm__ __volatile__("\tldrh %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
    return retval;
}

/* define putreg16(v,a)       (*(volatile uint16_t *)(a) = (v)) */
static inline void putreg16(uint16_t val, unsigned int addr)
{
    __asm__ __volatile__("\tstrh %0, [%1]\n\t": : "r"(val), "r"(addr));
}

/* # define getreg32(a)       (*(volatile uint32_t *)(a)) */
static inline uint32_t getreg32(unsigned int addr)
{
    uint32_t retval;
    __asm__ __volatile__("\tldr %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
    return retval;
}

/* define putreg32(v,a)        */
static inline void putreg32(uint32_t val, unsigned int addr)
{
    *(volatile uint32_t *)(addr) = val;
}

static inline void flash_wait_nb(void) {
  __DSB();
  while (FLASH->SR & FLASH_SR_BSY) {
    // nop
  }
}

/* Unlock the flash */
static void flash_unlock(void)
{
  __DSB();
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xcdef89ab;
  }
}

/* Erase the page containing this address */
void flash_erase_page(void *addr) {
  chSysLock();
  addr = addr  - ((uint32_t)addr % FLASH_PAGE_SIZE);
  // uint16_t i = 0;
  // uint32_t *p = (uint32_t *)(addr);
  // for (i = 0; i < FLASH_PAGE_SIZE-1; ++i)
  // {
  //   if(getreg32((uint32_t)&p[i]) != 0xFFFFFFFF)
  //   {
  //     break;
  //   }
  // }
  // if(i == FLASH_PAGE_SIZE) {
  //   /* already empty - no erase necessary */
  //   chSysUnlock();
  //   return;
  // }

  flash_unlock();
  flash_wait_nb();
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = (uint32_t)addr;
  FLASH->CR |= FLASH_CR_STRT;
  flash_wait_nb();
  FLASH->CR &= ~FLASH_CR_PER;
  FLASH->CR |= FLASH_CR_LOCK;
  chSysUnlock();
}

/* Erase pages starting from the address until a certain size and return the last erased address */
uint32_t flash_erase_pages(uint32_t start_addr, uint64_t size) {
  uint32_t addr = start_addr - (start_addr % FLASH_PAGE_SIZE);
  size += (start_addr % FLASH_PAGE_SIZE);
  do {
    flash_erase_page((void *)addr);
    addr += FLASH_PAGE_SIZE;
  } while(addr <= start_addr+size);

  return (addr - 1);
}

/* compare flash content with given array.
 * returns 1 on match, 0 on dismatch.
 * adr and data need to be 16-bit aligned,
 * len should be an even number of bytes!
 */
int flash_verify_block(void *adr, uint8_t *data, uint16_t len)
{
  uint16_t  i = 0;
  uint16_t *flash = (uint16_t *)adr;
  uint16_t *v = (uint16_t *)data;
  len = (len + 1) / 2;
  for (i = 0; i < len; ++i)
  {
    if (getreg16((uint32_t)&flash[i]) != v[i])
    {
      return 0;
    }
  }
  return 1;
}

/* writes len bytes of data to adr.
 * data and addr need to be half-word aligned!
 * len should be an even number of bytes!
 */
#pragma GCC push_options
#pragma GCC optimize ("O0")
void flash_write_block(uint32_t addr, const void *buf, uint16_t count)
{
  uint8_t *b = (uint8_t *)buf;
  if(count & 1) {
    count += 1;
  }

  chSysLock();
  flash_unlock();
  flash_wait_nb();
  while (count >= 2)
  {
    FLASH->CR = FLASH_CR_PG;

    putreg16(*(uint16_t *)b, addr);
    flash_wait_nb();
    FLASH->CR = 0;

    count -= 2;
    b += 2;
    addr += 2;
  }
  flash_wait_nb();
  FLASH->CR |= FLASH_CR_LOCK;
  chSysUnlock();
}
#pragma GCC pop_options