#include "motor.h"

void PWM_Config(uint16_t period, uint16_t high)
{
	// PF2 Output Initialization
	
	SYSCTL->RCGCPWM |= 0x02;  // Set clock to use PWM1
	SYSCTL->RCGCGPIO |= 0x20; // Activate Port F
	while ((SYSCTL->PRGPIO & 0x20) == 0){};					  // Allow time for clock Port F to setup
	// NOTE: Maybe not enough time? Add Delay?
	GPIOF->AFSEL |= 0x04;									  // Activate PF2
	GPIOF->PCTL &= ~0x00000F00; 							  // Configure PF2 as PWM1
	GPIOF->PCTL |= 0x00000500;								  // Write to program control register
	GPIOF->AMSEL &= ~0x04;									  // Disable analog functionality on PF2
	GPIOF->DIR |= 0x04; 									  // Make PF2 output
	GPIOF->DEN |= 0x04;										  // Enable digital I/O on PF2

	// PWM Generator Initialization
	SYSCTL->RCC = 0x001E0000 | (SYSCTL->RCC & (~0x00E70000)); // NOTE: (FLAGGED! Check w/ Alex's for reference) Use system clock for PWM
	PWM1->_3_CTL = 0x00;								      // Disable PWM1_3 during configuration
	PWM1->_3_GENA = 0xC8;								      // Output low for load, high for match
	PWM1->_3_LOAD = period - 1;								  // Period is 2500
	PWM1->_3_CMPA = high - 1;								  // Duty Cycle @ 99% | high = 2474
	PWM1->_3_CTL |= 1;										  // Enable PWM1_3
	PWM1->ENABLE |= 0x40;									  // Enable PWM1

	/*
	// Alex's PWM Code
	SYSCTL->RCGCGPIO |= 0x02; 

    Delay1ms(10);

	SYSCTL->RCC &= ~0x00100000; 
    PWM1->_3_CTL = 0x00; //Disable PWM 
    PWM1->_3_GENA = 0xC8; 
    PWM1->_3_LOAD = period - 1; //Set period
    PWM1->_3_CMPA = high - 1;   //Set duty cycle
    PWM1->_3_CTL |= 0x1; //Enable PWM 
    PWM1->ENABLE |= 0x40; //Enable PWM output PF2
    
    GPIOF->AFSEL |= 0x04; //Enable alternate function
    GPIOF->PCTL &= ~0x00000F00; //Clear PF2 
    GPIOF->PCTL |= 0x00000500; //Set PF2
	GPIOF->DIR |= 0x04; 
    GPIOF->DEN |= 0x04; 
	*/
}

// DC Motor Drivers
void DCMotor_Init(void){
	SYSCTL->RCGCGPIO |= 0x02;				  				// Activate Port B
	while ((SYSCTL->RCGCGPIO & 0x02) == 0){}; // Allow time for clock Port B to setup
	GPIOB->DIR |= 0x03; // Make PB0-1 input
	GPIOB->DEN |= 0x03; // Enable digital I/O on PB0-1
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

