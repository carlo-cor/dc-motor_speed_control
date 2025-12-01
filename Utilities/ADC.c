#include "ADC.h"

// NOTE: Rewire and check mapping
// ------------------------------
    /*
    - PB2-6 to pins 13-9 (B3 - B7)  | 
    - PE2-1 to pins 15-16 (B2 - B1) | 
    - PB7 to pins 17 (B0)           | 
    */

void ADC_Init(void)
{
	SYSCTL->RCGCGPIO |= 0x12; // Port E and Port B
	while ((SYSCTL->PRGPIO & 0x12) == 0){}

    // PB2-7 are inputs
	GPIOB->DIR &= ~0xFC; 
	GPIOB->DEN |= 0xFC;
	GPIOB->AMSEL &= ~0xFC;
	GPIOB->AFSEL &= ~0xFC;

    // PE1-2 are inputs
	GPIOE->DIR &= ~0x06; 
	GPIOE->DEN |= 0x06;
	GPIOE->AMSEL &= ~0x06;
	GPIOE->AFSEL &= ~0x06;

    // PE4-Busy (Pin 21)
	GPIOE->DIR &= ~0x10; 
	GPIOE->DEN |= 0x10;

    // PE5-BYTE (Pin 24)
	GPIOE->DIR |= 0x20; 
	GPIOE->DEN |= 0x20;
	GPIOE->DATA |= 0x20;
}

// NOTE: Check and configure
uint8_t ADC_Read(void){
    uint8_t ADC_DATA = ((GPIOB->DATA & 0x7C) << 1) | (GPIOE->DATA & 0x06) | ((GPIOB->DATA & 0x80) >> 7);
    return ADC_DATA;
}

