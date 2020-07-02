#include "memory.h"

void Unlock_Memory_Flash(void){
	//Flash key register (FLASH_KEYR)
	//page 60 (STM32F401 Reference Manual (pours les périphériques))
	FLASH->KEYR=0x45670123;
	FLASH->KEYR=0xCDEF89AB;
}

void Lock_Memory_Flash(void){
	//Setting the LOCK bit of Flash control register (FLASH_CR)
	//page 62 (STM32F401 Reference Manual (pours les périphériques))
	FLASH->CR |= (1<<31);
}

void Erase_Sector_Seven(void){
	Unlock_Memory_Flash();

	//Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register
	//page 61 (STM32F401 Reference Manual (pours les périphériques))
	while(FLASH->SR & (1<<16));

	//Set the SER bit and select the sector out of the 5 sectors (for STM32F401xB/C) and 
	//out of 7 (for STM32F401xD/E) in the main memory block) you wish to erase (SNB) in the FLASH_CR register
	FLASH->CR |= (1<<1);
	FLASH->CR &= ~(15<<3);
	FLASH->CR |= (7<<3);
	
	//Set the STRT bit in the FLASH_CR register
	FLASH->CR |= (1<<16);

	//Wait for the BSY bit to be cleared
	while(FLASH->SR & (1<<16));

	Lock_Memory_Flash();		
}

void Write_Sector_Seven(uint32_t add, uint32_t val){
	Unlock_Memory_Flash();
	//Check that no main Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register.
	while(FLASH->SR & (1<<16));

	//Set the PG bit in the FLASH_CR register
	FLASH->CR |= (1<<0);

	// Perform the data write operation(s) to the desired memory address (inside main memory block or OTP area):
	//–Word access in case of x32 parallelism
	FLASH->CR &= ~(3<<8);
	FLASH->CR |= (2<<8);
	*(__IO uint32_t*)add = (uint32_t) val;

	//Wait for the BSY bit to be cleared
	while(FLASH->SR & (1<<16));
	Lock_Memory_Flash();
}


uint32_t Read_Sector_Seven(uint32_t add)
{
	return *(uint32_t*)add;
}
