#include "motor.h"

void PWM_Config(uint32_t period, uint16_t high)
{
	// PF2 Output Initialization
	
	SYSCTL->RCGCPWM |= 0x02;  // Set clock to use PWM1
	SYSCTL->RCGCGPIO |= 0x22; // Activate Port B and F
	while ((SYSCTL->PRGPIO & 0x22) == 0){};		// Allow time for clock Port B and F to setup
		
	// PWM Generator Initialization
	SYSCTL->RCC |=0x1E0000;   // Use system clock for PWM
	PWM1->_3_CTL = 0x00;								      // Disable PWM1_3 during configuration
	PWM1->_3_GENA = 0xC8;								      // Output low for load, high for match
	PWM1->_3_LOAD = period - 1;								// Period is 2500
	PWM1->_3_CMPA = high - 1;								  // Duty Cycle @ 99% | high = 2474
	PWM1->ENABLE |= 0x40;									    // Enable PWM1
	PWM1->_3_CTL |= 0x1;										  // Enable PWM1_3
	
	// PF2 Initialization
	GPIOF->AFSEL |= 0x04;									  // Activate PF2
	GPIOF->PCTL &= ~0x00000F00; 					  // Configure PF2 as PWM1
	GPIOF->PCTL |= 0x00000500;						  // Write to program control register
	GPIOF->DIR |= 0x04; 									  // Make PF2 output
	GPIOF->DEN |= 0x04;										  // Enable digital I/O on PF2
	
	// PB0-1 Initialization
	GPIOB->DIR |= 0x03;											// Set default direction
	GPIOB->DEN |= 0x03;  								    // Set pins 0-1 as output 
}

void setMotorDirectionFwd(void){
	GPIOB->DATA = (GPIOB->DATA & ~0x03) | 0x02; // Set motor direction forward
}

void setMotorDirectionBckwrd(void){
	GPIOB->DATA = (GPIOB->DATA & ~0x03) | 0x01; // Set motor direction backward
}

void setMotorSpeed(uint16_t duty){
	PWM1->_3_CMPA = duty - 1;					// Set speed via duty cycle value (percentage)
}

