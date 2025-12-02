#include "tm4c123gh6pm.h"
#include "ADC.h"

// ADS7806 Parallel MSB mode
// Data bus mapping (your wiring):
//   PB2 = D7 (MSB)
//   PB3 = D6
//   PB4 = D5
//   PB5 = D4
//   PB6 = D3
//   PE2 = D2
//   PE1 = D1
//   PB7 = D0 (LSB of MSB byte)
//
// Control pins:
//   PD1 = R/C (output)  ? ADS7806 R/C
//   PE5 = BYTE (output) ? ADS7806 BYTE (0 = MSB byte)
//
// We will NOT use BUSY in this version, just a fixed delay.

static void DelayCycles(volatile uint32_t n)
{
    while (n--) {
        __asm(" NOP");
    }
}

// Adjust this for your clock. For 16 MHz, ~16 cycles ˜ 1 µs.
// So 30 µs ˜ 30 * 16 = 480 cycles. We’ll overkill a bit.
static void DelayUs(uint32_t us)
{
    // crude approximate delay: us * 20 cycles
    while (us--) {
        DelayCycles(20);
    }
}

void ADC_Init(void)
{
    //-------------------------------------------------------
    // Enable Ports B, E, D clocks
    //-------------------------------------------------------
    SYSCTL->RCGCGPIO |= (1U<<1) | (1U<<4) | (1U<<3); // B, E, D
    while ((SYSCTL->PRGPIO & ((1U<<1)|(1U<<4)|(1U<<3))) == 0U) {}

    //-------------------------------------------------------
    // DATA PINS (PB2–PB7) = inputs
    //-------------------------------------------------------
    GPIOB->DIR   &= ~0xFC;    // 11111100b ? PB2-7 inputs
    GPIOB->DEN   |=  0xFC;
    GPIOB->AFSEL &= ~0xFC;
    GPIOB->AMSEL &= ~0xFC;
    GPIOB->PCTL  &= ~0xFFFF0000;

    //-------------------------------------------------------
    // DATA PINS (PE1–PE2) = inputs
    //-------------------------------------------------------
    GPIOE->DIR   &= ~0x06;    // 00000110b ? PE1-2 inputs
    GPIOE->DEN   |=  0x06;
    GPIOE->AFSEL &= ~0x06;
    GPIOE->AMSEL &= ~0x06;
    GPIOE->PCTL  &= ~0x00000FF0;

    //-------------------------------------------------------
    // BYTE = PE5 (output, LOW = MSB byte)
    //-------------------------------------------------------
    GPIOE->DIR   |=  0x20;    // PE5 output
    GPIOE->DEN   |=  0x20;
    GPIOE->AFSEL &= ~0x20;
    GPIOE->AMSEL &= ~0x20;
    GPIOE->DATA  &= ~0x20;    // BYTE = 0 ? MSB byte on D7..D0

    //-------------------------------------------------------
    // R/C = PD1 (output)
    //-------------------------------------------------------
    GPIO_PORTD_LOCK_R = 0x4C4F434B;  // unlock PD
    GPIO_PORTD_CR_R  |= 0x02;        // allow PD1 changes

    GPIOD->DIR   |=  0x02;           // PD1 output
    GPIOD->DEN   |=  0x02;
    GPIOD->AFSEL &= ~0x02;
    GPIOD->AMSEL &= ~0x02;
    GPIOD->PCTL  &= ~0x000000F0;

    // Idle state: R/C = HIGH
    GPIOD->DATA  |= 0x02;
}

// Start a conversion: R/C falling edge of =40 ns
static inline void ADC_StartConversion(void)
{
    GPIOD->DATA &= ~0x02;      // R/C LOW ? start conversion
    __asm(" NOP; NOP; NOP; NOP; ");
    GPIOD->DATA |=  0x02;      // R/C HIGH ? back to idle
}

// Read 8-bit MSB result using fixed delay instead of BUSY
uint8_t ADC_Read(void)
{
    ADC_StartConversion();

    // Wait long enough for conversion to finish
    // Datasheet: 20 µs max conversion, 25 µs full cycle.
    // We'll wait ~30 µs to be safe.
    DelayUs(30);

    // Now read parallel data bus
    uint8_t  b  = 0;
    uint32_t pb = GPIOB->DATA;
    uint32_t pe = GPIOE->DATA;

    b |= ((pb >> 2) & 0x1F) << 3;   // PB2–PB6 ? bits 7–3
    b |= ((pe >> 1) & 0x03) << 1;   // PE1–PE2 ? bits 2–1
    b |= ((pb >> 7) & 0x01);        // PB7     ? bit 0

    return b;
}
