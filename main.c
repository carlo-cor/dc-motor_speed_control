#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include "ADC.h"
#include "motor.h"

#define TIMESLICE 32000        // Timeslice of 2 ms
#define CPU_HZ 16000                 // CPU Clock Hz
#define PWM_PERIOD 4000        // PWM period
#define PWM_DUTY_CYCLE 2000    // PWM duty cycle
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
int32_t Keypress_Buffer;

// PID Timer Handler Variables
int32_t estRPM, targetRPM, error; // Speed variables
int32_t P, I, D;    // PID variables

// PID Constants
int32_t kP = 1;
int32_t kI = 1;
int32_t kD = 1; 

int32_t actuatorRPM; 			 // Speed error adjustment
uint32_t time;             // Time in 0.1 ms
uint32_t count;            // Counter for time

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
extern void(Hex2ASCII(char *s,uint32_t n ));
extern int32_t Current_speed(int32_t Avg_volt);

static inline void Delay100us(void)
{
    const uint32_t cycles = (CPU_HZ / 10000u); // 100us worth of cycles
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

// Globals
int32_t Keypress_Buffer = -1;      // -1 means empty
uint8_t sampledADC_value = 0;
int32_t targetRPM = 0;
char keypadBuffer[17];             // fits one LCD line (16 chars + null)
int keypadIndex = 0;

// Helper Functions

uint8_t getKeypadChar(void)
{
    Scan_Keypad(); // blocks until a key is pressed
    return (uint8_t)(*(volatile uint32_t *)&Key_ASCII);
}

uint8_t getADCavg(void)
{
    uint32_t sum = 0;

    for(int i = 0; i < 100; i++)
    {
        sum += ADC_Read();      // <--- correct ADC read
        Delay100us();
    }

    return (uint8_t)(sum / 100);
}

// Thread Functions

void retrieveInput(void)
{
    uint8_t k = getKeypadChar();   // Waits until a key is pressed

    // Clear buffer if user presses 'C'
    if(k == 'C')
    {
        keypadIndex = 0;
        keypadBuffer[0] = '\0';
        return;
    }

    // Append character if space available
    if(keypadIndex < 16)
    {
        keypadBuffer[keypadIndex++] = k;
        keypadBuffer[keypadIndex]   = '\0'; // null terminate
    }
}

void updateLCD(void)
{
    // Row 1
    Set_Position(0x00);
    Display_Msg("Keys: ");

    Set_Position(0x06);      // print starting at column 6
    Display_Msg(keypadBuffer);
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
            if(actuatorRPM < 100) actuatorRPM = 100;
            if(actuatorRPM > 19900) actuatorRPM = 19900;
            setMotorSpeed(actuatorRPM);
      }
      TIMER0->ICR = 0x01;
}

int main(void)
{
    Init_LCD_Ports();
    Init_LCD();
    Init_Keypad();

		/*
    keypadBuffer[0] = '\0'; // initialize empty buffer

    while(1)
    {
        retrieveInput();   // Read one key and store it
        updateLCD();       // Show buffer on LCD
        Delay1ms(50);
    }
		*/
}