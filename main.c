#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>

// Threads
void LED_Display(void);
void LCD_Display(void);
void GetInput(void);

// OS Functions
void OS_Init(void);
int OS_AddThreads(void (*task0)(void), void (*task1)(void), void (*task2)(void));
void OS_Launch(uint32_t theTimeSlice);

// OS Semaphore Functions
void OS_Signal(int32_t *s);
void OS_Sleep(uint32_t SleepCtr);
void OS_Suspend(void);
void OS_Wait(int32_t *s);

// PWM Config
void TIMER0A_Handler(uint32_t reload);
void PWM_Config(uint16_t period, uint16_t high);

int length = 0;
uint32_t SW5;
int32_t SW1;

int speed = 0;
int dir = 0;

// PWM variables
uint16_t period = 2500;
uint16_t high = 2474;

uint32_t reload = 0;

uint8_t Key_ASCII;

int *startPt;
int *currentPt;

// Timer Variables
uint32_t Time;	 // Time in 0.1 msec
int32_t X;		 // Estimated speed in 0.1 RPM, 0 to 1000
int32_t Xstar;	 // Desired speed in 0.1 RPM, 0 to 1000
int32_t E;		 // Speed error in 0.1 RPM, -1000 to +1000
int32_t U, I, P; // Actuator duty cycle, 100 to 19900 cycles
uint32_t Cnt;	 // incremented every 0.1 msec

int32_t X, X_err, err; // speed, fixed-point
int32_t actual_speed;

// Keypad Initialization and Utility Functions
extern void(Init_Keypad(void));
extern void Scan_Keypad(void);
extern uint8_t Key_ASCII;

// LCD Initialization and Utility Functions
extern void Init_LCD_Ports(void);
extern void Init_LCD(void);
extern void Display_Msg(char *s);
extern void Display_Char(char c);
extern void Set_Position(uint32_t pos);
extern void Set_Blink_ON(uint32_t pos);
extern void Set_Blink_OFF(void);
extern void Delay1ms(uint32_t n);

// Motor Functions
extern uint32_t inputSpeed(uint8_t speed);
extern uint32_t setMotorDirection(void);
extern void PWM1C_Duty(uint16_t duty);

int main(void)
{
	OS_Init();				   // initialize, disable interrupts, 16 MHz
	SYSCTL_RCGCGPIO_R |= 0x3F; // activate clock for all ports
	while ((SYSCTL_RCGCGPIO_R & 0x3F) == 0){}; // allow time for clock to stabilize
	GPIOB->DIR |= 0x03; // make PB0-1 input
	GPIOB->DEN |= 0x03; // Enable digital I/O on PB0-1
	GPIOF->DIR |= 0x04; // Make PF2 output
	GPIOF->DEN |= 0x04; // enable digital I/O on PF2
	Init_LCD_Ports();
	Init_LCD();
	TIMER0A_Handler(reload);
	PWM_Config(period, 1500);

	// LCD Display
	char tbuf[6], cbuf[6], tpad[6], cpad[6];
	Set_Position(0x00);
	Display_Msg("Input: ");
	Display_Msg("INS Speed");
	Set_Position(0x40);
	Display_Msg("T: ");
	Display_Msg("INS ");
	Display_Msg("C: ");
	Display_Msg("INS");

	while (1)
	{
		Scan_Keypad(); // Blocks until a key is pressed
		uint8_t k = Key_ASCII;

		// Now you can use k in your logic
		if (k >= '0' && k <= '9')
		{
			// handle digits
		}
		else if (k == '#')
		{
			// confirm input
		}
		else if (k == 'C')
		{
			// clear input
		}

		OS_Sleep(20); // small debounce + yield
	}

	// OS_AddThreads(&GetInput, &LCD_Display, &LED_display );
	// OS_Launch(32000);
	return 0;
}

void PWM_Config(uint16_t period, uint16_t high)
{
	SYSCTL->RCGCPWM |= 0x02;  // Set clock to use PWM1
	SYSCTL->RCGCGPIO |= 0x20; // Activate PortF
	while ((SYSCTL->PRGPIO & 0x20) == 0){};
	GPIOF->AFSEL |= 0x04;									  // Activate PF2
	GPIOF->PCTL &= ~0x00000F00; 							  // Configure PF2 as PWM1
	GPIOF->PCTL |= 0x00000500;								  // Write to program control register
	GPIOF->AMSEL &= ~0x04;									  // Disable analog functionality on PF2
	GPIOF->DEN |= 0x04;										  // Enable digital I/O on PF2
	SYSCTL->RCC = 0x001E0000 | (SYSCTL->RCC & (~0x00E70000)); // Use system clock for PWM
	PWM1->_3_CTL = 0;										  // Disable PWM1_3 during configuration
	PWM1->_3_GENA = 0x00000C8;								  // Output low for load, high for match
	PWM1->_3_LOAD = period - 1;								  // Period is 2500
	PWM1->_3_CMPA = high - 1;								  // Duty Cycle @ 99% | high = 2474
	PWM1->ENABLE |= 0x40;									  // Enable PWM1
	PWM1->_3_CTL |= 1;										  // Enable PWM1_3
}

void ADC_Init(void)
{
	SYSCTL->RCGCGPIO |= 0x12; // Port E and Port B
	while ((SYSCTL->PRGPIO & 0x12) == 0)
	{
	}

	GPIOB->DIR &= ~0xFC; // PB2-7 are inputs
	GPIOB->DEN |= 0xFC;
	GPIOB->AMSEL &= ~0xFC;
	GPIOB->AFSEL &= ~0xFC;

	GPIOE->DIR &= ~0x06; // PE1-2 are inputs
	GPIOE->DEN |= 0x06;
	GPIOE->AMSEL &= ~0x06;
	GPIOE->AFSEL &= ~0x06;

	GPIOE->DIR &= ~0x10; // PE4-Busy (pin 21)
	GPIOE->DEN |= 0x10;

	GPIOE->DIR |= 0x20; // PE5-BYTE (pin 24)
	GPIOE->DEN |= 0x20;
	GPIOE->DATA |= 0x20;
}

void Timer0A_Handler(void)
{
	X = inputSpeed(k);	   // estimated speed
	err = X_err - X; // error
	if (err < -10)
		actual_speed--; // decrease if too fast
	else if (err > 10)
		actual_speed++; // increase if too slow
	// leave as is if close enough
	if (actual_speed < 2)
		actual_speed = 2; // underflow (minimum PWM)
	if (actual_speed > 249)
		actual_speed = 249;   // overflow (maximum PWM)
	PWM1C_Duty(actual_speed); // output to actuator
	TIMER0_ICR_R = 0x01;
	// acknowledge timer0A periodic timer
}

