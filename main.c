#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include "ADC.h"
#include "motor.h"

#define TIMESLICE 32000		 // Timeslice of 2 ms
#define CPU_HZ 16000			 // CPU Clock Hz
#define PWM_PERIOD 4000		 // PWM period
#define PWM_DUTY_CYCLE 2000	 // PWM duty cycle
#define TIMER_RELOAD_VALUE 0 // Timer reload value

#define DEFAULT_MOTOR_SPEED 2000 // Default motor speed

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

void TIMER0A_Handler(uint32_t reload);

// Key Input Variables
uint8_t Key_ASCII;
int32_t sampledADC_value;

// PID Timer Handler Variables
int32_t estRPM, targetRPM, error; // Speed variables
int32_t P, I, D;    // PID variables
int32_t kP, kI, kD; // PID constants
int32_t actuatorRPM; // Speed error adjustment
uint32_t time;		   // Time in 0.1 ms
uint32_t count;		   // Counter for time

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

extern int32_t Current_speed(int32_t Avg_volt);

static inline void Delay100us(void)
{
    const uint32_t cycles = (CPU_HZ / 10000u); // 100us worth of cycles
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

void retrieveInput(void)
{
	// ADC Implementation
	uint8_t sum = 0;
	uint8_t avgVal = 0;
	
	// Attain a sample per 100us and sum it together
	for(uint32_t i = 0; i < 100; i++){
		sum += i;
		Delay100us();
	}
	
	sampledADC_value = sum/100;					  // Averaged sampled ADC value
	targetRPM = Current_speed(sampledADC_value);  // Get target RPM speed using voltage conversion function
	
	// If sampled ADC value is below zero for whatever reason, no RPM
	if(targetRPM < 0){
		targetRPM = 0;
	}
	
	// continue..
	
	// Keypad Input
	while (1)
	{
		Scan_Keypad();								  // Blocks until a key is pressed
		uint8_t k = Key_ASCII;						  // Takes in value from keypad press
		// unsigned char key_value = (unsigned char)(k); // Cast value into readable character

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
	
}

void motorPIDControlLoop(void)
{
	
}

void TIMER0A_Handler(uint32_t reload)
{
	time++;
	if((count++) == 4000){
		count = 0;
		error = estRPM - targetRPM; // Calculated error of current speed versus desired

		// PID variable calculations
		P = (kP * error) / 20;
		I = I + (kI * error) / 640;
		D = kD * error;

		if(I < -500) I = -500;
		if(I > 4000) I = 4000;
		
		actuatorRPM = P + I;
		if(actuatorRPM < 100) U = 100;
		if(actuatorRPM > 19900) U = 19900;
		setMotorSpeed(actuatorRPM);
	}
	TIMER0->ICR = 0x01;
}

int main(void)
{
	// Initialization Functions
	OS_Init();
	Init_LCD_Ports();
	Init_LCD();
	PWM_Config(PWM_PERIOD, PWM_DUTY_CYCLE);
	ADC_Init();

	// Default Motor Speed Test (Working)
	// setMotorDirectionFwd();
	// setMotorSpeed(DEFAULT_MOTOR_SPEED);
	
	// RTOS Thread Control
	/*
	OS_AddThreads(&retrieveInput, &updateLCD, &motorPIDControlLoop);
	OS_Launch(TIMESLICE);
	*/

	return 0;
}
