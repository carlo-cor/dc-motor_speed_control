#include "tm4c123gh6pm.h"
#include "ADC.h"

// ADS7806 Parallel MSB mode
// Data bus mapping:
//   PB5 = D7 (MSB) ADC-B11 Pin9
//   PB4 = D6	ADC-B10 Pin10
//   PB3 = D5	ADC-B9 Pin11
//   PB2 = D4 ADC-B8 Pin12
//   PC5 = D3 ADC-B7 Pin13
//   PC4 = D2	ADC-B6 Pin15
//   PE2 = D1	ADC-B5 Pin16
//   PE1 = D0 (LSB of MSB byte) need to change to different pin (maybe?) ADC-B4 Pin17
//
// Control pins:
//   PE3 = R/C (output)  
//   PE4 = BUSY (input)
//   PE5 = BYTE (output) 
//

static void DelayCycles(volatile uint32_t n){
    while (n--) {
        __asm(" NOP");
    }
}

static void DelayUs(uint32_t us){
    // crude approximate delay: us * 20 cycles
    while (us--) {
        DelayCycles(20);
    }
}

// Start a conversion: R/C falling edge of =40 ns
static inline void ADC_StartConversion(void)
{
    GPIOE->DATA &= ~0x08;      // R/C LOW ? start conversion
		DelayUs(30);
		GPIOE->DATA  |= 0x08;    // BUSY = 0 to take in reading
}

void ADC_Init(void)
{
    //-------------------------------------------------------
    // Enable Ports B, C, E,  clocks
    //-------------------------------------------------------
    SYSCTL->RCGCGPIO |= 0x16; // B, E, D
    while ((SYSCTL->PRGPIO & 0x16) == 0){};	

    //=================================================================
    // DATA BUS CONFIGURATION (Inputs)
    //=================================================================

    //-------------------------------------------------------
    // PB2–PB5 = D4–D7
    //-------------------------------------------------------
    GPIOB->DIR   &= ~(0x3C);     // PB2-5 input (0011_1100)
    GPIOB->DEN   |=  (0x3C);
    GPIOB->AFSEL &= ~(0x3C);
    GPIOB->AMSEL &= ~(0x3C);
    GPIOB->PCTL  &= ~(0xFFFF0F00);  // Clear PB2–PB5 fields

    //-------------------------------------------------------
    // PC4–PC5 = D2–D3
    //-------------------------------------------------------
    GPIOC->DIR   &= ~(0x30);     // PC4-5 input (0011_0000)
    GPIOC->DEN   |=  (0x30);
    GPIOC->AFSEL &= ~(0x30);
    GPIOC->AMSEL &= ~(0x30);
    GPIOC->PCTL  &= ~(0x00FF0000);  // Clear PC4–PC5 fields

    //-------------------------------------------------------
    // PE1–PE2 = D0–D1
    //-------------------------------------------------------
    GPIOE->DIR   &= ~(0x06);     // PE1-2 input
    GPIOE->DEN   |=  (0x06);
    GPIOE->AFSEL &= ~(0x06);
    GPIOE->AMSEL &= ~(0x06);
    GPIOE->PCTL  &= ~(0x00000FF0); // Clear PE1/PE2 fields

    //=================================================================
    // CONTROL PINS
    //=================================================================

    //-------------------------------------------------------
    // R/C = PE3 (output)
    //-------------------------------------------------------
    GPIOE->DIR   |=  0x08;   // PE3 output
    GPIOE->DEN   |=  0x08;
    GPIOE->AFSEL &= ~0x08;
    GPIOE->AMSEL &= ~0x08;
    GPIOE->DATA  |=  0x08;   // idle high

    //-------------------------------------------------------
    // BUSY = PE4 (input — ADS7806 drives it)
    //-------------------------------------------------------
    GPIOE->DIR   &= ~0x10;   // PE4 input
    GPIOE->DEN   |=  0x10;
    GPIOE->AFSEL &= ~0x10;
    GPIOE->AMSEL &= ~0x10;

    //-------------------------------------------------------
    // BYTE = PE5 (output, 0 = MSB byte)
    //-------------------------------------------------------
    GPIOE->DIR   |=  0x20;   // PE5 output
    GPIOE->DEN   |=  0x20;
    GPIOE->AFSEL &= ~0x20;
    GPIOE->AMSEL &= ~0x20;
    GPIOE->DATA  &= ~0x20;   // BYTE = 0 (read MSB byte)
}

// Read 8-bit MSB result using fixed delay instead of BUSY
int8_t ADC_Read(void)
{
    int8_t value = 0;

    ADC_StartConversion();

    // Wait for BUSY (PE4) = 1 (conversion finished)
    while ((GPIOE->DATA & 0x10) == 0) {}

    // Capture GPIO values
    uint32_t pb = GPIOB->DATA;
    uint32_t pc = GPIOC->DATA;
    uint32_t pe = GPIOE->DATA;

    // PB5–PB2 ? D7–D4
    value |= (pb & (1<<5)) ? (1<<7) : 0;   // PB5 -> bit7
    value |= (pb & (1<<4)) ? (1<<6) : 0;   // PB4 -> bit6
    value |= (pb & (1<<3)) ? (1<<5) : 0;   // PB3 -> bit5
    value |= (pb & (1<<2)) ? (1<<4) : 0;   // PB2 -> bit4

    // PC5–PC4 ? D3–D2
    value |= (pc & (1<<5)) ? (1<<3) : 0;   // PC5 -> bit3
    value |= (pc & (1<<4)) ? (1<<2) : 0;   // PC4 -> bit2

    // PE2–PE1 ? D1–D0
    value |= (pe & (1<<2)) ? (1<<1) : 0;   // PE2 -> bit1
    value |= (pe & (1<<1)) ? (1<<0) : 0;   // PE1 -> bit0
		
		value = (value * 10000)/ 0x7F;
    return value;
}
