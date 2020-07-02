#ifndef _MEMORY_H
#define _MEMORY_H

#include "stm32f4xx.h"

#define NBR_DATA_SECTOR_SEVEN 0x08060000

void Unlock_Memory_Flash(void);
void Lock_Memory_Flash(void);
void Erase_Sector_Seven(void);
void Write_Sector_Seven(uint32_t, uint32_t);
uint32_t Read_Sector_Seven(uint32_t);

#endif
