#ifndef ADC_H
#define ADC_H

#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>
#include <stdint.h>

void ADC_Init(void);
uint8_t ADC_Read(void);

#endif
