#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include "ADC.h"
#include "motor.h"

// NOTE: Fix #includes, and make header files for .c

#define TIMESLICE 32000		 // Timeslice of 2 ms
#define PWM_PERIOD 2500		 // PWM period
#define PWM_DUTY_CYCLE 1500	 // PWM duty cycle
#define TIMER_RELOAD_VALUE 0 // Timer reload value

#define DEFAULT_MOTOR_SPEED 400 // Default motor speed

// Threads
void retrieveInput(void);
void updateLCD(void);
void motorPIDControlLoop(void);

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

uint8_t Key_ASCII;

/*
// NOTE: Not sure what these are for again? Double check later.
int length = 0;
uint32_t SW5;
int32_t SW1;
*/

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

void retrieveInput(void)
{
	// Keypad Input
	while (1)
	{
		Scan_Keypad();								  // Blocks until a key is pressed
		uint8_t k = Key_ASCII;						  // Takes in value from keypad press
		unsigned char key_value = (unsigned char)(k); // Cast value into readable character

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
}

void updateLCD(void)
{
	/*
	char tbuf[6], cbuf[6], tpad[6], cpad[6];

	// Row 1 Line
	Set_Position(0x00);
	Display_Msg("Input: ");
	Display_Msg("INS Speed");

	// Row 2 Line
	Set_Position(0x40);
	Display_Msg("T: ");
	Display_Msg("INS ");
	Display_Msg("C: ");
	Display_Msg("INS");
	*/
}

void motorPIDControlLoop(void)
{
	uint32_t kP;
	uint32_t kI;
	uint32_t kD;

	/*
	// NOTE: Tadrous' PID implementation variables from slides, clean up and look into more
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
	*/


}

int main(void)
{
	// Initialization Functions
	OS_Init();
	Init_LCD_Ports();
	Init_LCD();
	PWM_Config(PWM_PERIOD, PWM_DUTY_CYCLE);
	DCMotor_Init();
	ADC_Init();

	// Default Motor Speed
	setMotorDirectionFwd();
	setMotorSpeed(DEFAULT_MOTOR_SPEED);

	// RTOS Thread Control
	/*
	OS_AddThreads(&retrieveInput, &updateLCD, &motorPIDControlLoop);
	OS_Launch(TIMESLICE);
	*/

	return 0;
}
