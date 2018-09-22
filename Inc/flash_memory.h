#ifndef _FLASH_MEMORY_H
#define _FLASH_MEMORY_H
#include "main.h"
#include "stm32f4xx_hal.h"
void FlashInit(void);
void WriteFlash(uint32_t addr, uint16_t DATA_16);
void PrintFlash(void);
uint16_t ReadFlash(uint32_t addr);

#endif
