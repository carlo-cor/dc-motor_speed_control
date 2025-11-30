#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>

// DC Motor Drivers

uint32_t inputSpeed(uint8_t speed){
    return speed;
}

uint32_t setMotorDirection(){

}

// change duty cycle of PF2
// duty is number of PWM clock cycles output is high
// (2<=duty<=period-1)
void PWM1C_Duty(uint16_t duty)
{
	PWM1_3_CMPA_R = duty - 1;
	// count value when output rises
}