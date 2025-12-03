/**
 * Motor Control System with PID and RTOS
 * 
 * This program implements a closed-loop motor speed controller using a PID algorithm.
 * It runs on a TM4C123 microcontroller with a simple RTOS for task scheduling.
 */

#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include "ADC.h"
#include "motor.h"

// System Configuration Constants
#define TIMESLICE          32000      // RTOS time slice of 2 ms (16 MHz clock)
#define CPU_HZ             16000      // CPU frequency in kHz (16 MHz)
#define PWM_PERIOD         5000       // PWM period in ticks
#define PWM_DUTY_CYCLE     2000       // Initial PWM duty cycle

// PID Controller Gains (tuning parameters)
#define kP                 1
#define kI                 1
#define kD                 1

#define DEFAULT_MOTOR_SPEED 2000      // Default motor speed for testing

// Thread Function Declarations
void retrieveInput(void);             // Handles keypad input
void updateLCD(void);                 // Updates LCD display
void motorPIDControlLoop(void);       // PID control thread (not implemented)

// RTOS Function Declarations
void OS_Init(void);
int OS_AddThreads(void (*task0)(void), void (*task1)(void));
void OS_Launch(uint32_t theTimeSlice);

// RTOS Semaphore Functions
void OS_Signal(int32_t *s);
void OS_Sleep(uint32_t SleepCtr);
void OS_Suspend(void);
void OS_Wait(int32_t *s);

// Keypad Input Variables
uint8_t Key_ASCII;                    // Last pressed key in ASCII format
int32_t Keypress_Buffer;              // Buffer for numeric keypad input

// PID Controller Variables
int32_t estRPM, targetRPM, error;     // Speed tracking and error
int32_t P, I, D;                      // PID components
int32_t actuatorRPM;                  // Calculated motor speed adjustment
int32_t sum;                          // ADC sample accumulator
int time;                             // Time counter in 0.1 ms units
int count;                            // Sample counter for averaging

// External Hardware Interface Functions
extern void Init_Keypad(void);        // Keypad initialization
extern void Scan_Keypad(void);        // Keypad scanning routine
extern void Init_LCD_Ports(void);     // LCD port configuration
extern void Init_LCD(void);           // LCD initialization
extern void Display_Msg(char *s);     // Display string on LCD
extern void Display_Char(char c);     // Display single character
extern void Set_Position(uint32_t pos); // Set LCD cursor position
extern void Set_Blink_ON(uint32_t pos); // Enable cursor blinking
extern void Set_Blink_OFF(void);      // Disable cursor blinking
extern void Delay1ms(uint32_t n);     // Millisecond delay
extern void Hex2ASCII(char *s, uint32_t n); // Convert number to ASCII string
extern int32_t Current_speed(int32_t Avg_volt); // Convert voltage to RPM

// Timer and Initialization Functions
void TIMER0A_Handler(void);           // Timer interrupt handler for PID
void PI_Timer_Init(void);             // Timer initialization

// ADC Data
uint32_t sampledADC_value = 0;        // Latest sampled ADC value

// Keypad Buffer Management
int32_t Keypress_Buffer = -1;         // -1 indicates empty buffer
char keypadBuffer[17];                // Buffer for LCD display (16 chars + null)
int keypadIndex = 0;                  // Buffer index

/**
 * @brief  Creates a precise 100탎 delay using the CPU cycle counter
 * @note   Uses DWT cycle counter for accurate timing
 */
static inline void Delay100us(void)
{
    const uint32_t cycles = (CPU_HZ / 10000u); // Calculate cycles for 100탎
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {}
}

/**
 * @brief   Reads a character from the keypad
 * @return  ASCII value of pressed key
 * @note    Blocks until a key is pressed
 */
uint8_t getKeypadChar(void)
{
    Scan_Keypad(); // Blocking call until key press
    return (uint8_t)(*(volatile uint32_t *)&Key_ASCII);
}

// Thread Implementations

/**
 * @brief   Keypad input processing thread
 * 
 * Handles numeric input, entry confirmation (#), and clearing (C).
 * Updates the target RPM based on user input.
 */
void retrieveInput(void)
{
    while (1)
    {
        uint8_t k = getKeypadChar(); // Get keypad input
        
        if (k >= '0' && k <= '9') // Numeric key handling
        {
            if (Keypress_Buffer < -1) // Buffer is empty
            {
                Keypress_Buffer = Key_ASCII - 48; // Convert ASCII to digit
            }
            else if (Keypress_Buffer < 1000) // Limit to 3 digits
            {
                Keypress_Buffer = Keypress_Buffer * 10 + Key_ASCII - 48;
            }
            else // Buffer full, set as target RPM
            {
                targetRPM = Keypress_Buffer;
                Keypress_Buffer = -1; // Clear buffer
            }
        }
        else if (k == '#') // Confirm entry
        {
            if (Keypress_Buffer == -1)
            {
                targetRPM = 0; // Default to 0 if buffer empty
            }
            targetRPM = Keypress_Buffer;
            Keypress_Buffer = -1; // Clear buffer after confirmation
        }
        else if (k == 'C') // Clear input
        {
            Keypress_Buffer = -1; // Reset buffer
        }
        
        OS_Sleep(80); // Debounce delay and yield to scheduler
    }
}

/**
 * @brief   LCD update thread
 * 
 * Continuously updates the LCD display with current system status:
 * - Line 1: Current keypad input
 * - Line 2: Target RPM and estimated current RPM
 */
void updateLCD(void)
{
    while (1)
    {
        char inbuf[4], tbuf[4], cbuf[4]; // Buffers for number conversions
        
        // First display line: current input
        Set_Position(0x00);
        Display_Msg("Input: ");
        if (Keypress_Buffer > -1) // Display input if present
        {
            Hex2ASCII(inbuf, Keypress_Buffer);
            Display_Msg(inbuf);
        }
        else
        {
            Display_Msg("       "); // Clear area
        }
        
        // Second display line: target and current RPM
        Set_Position(0x40);
        Display_Msg("T: ");
        Hex2ASCII(tbuf, targetRPM);
        Display_Msg(tbuf);
        
        Display_Msg("  C: ");
        Hex2ASCII(cbuf, estRPM);
        Display_Msg(cbuf);
    }
}

/**
 * @brief Timer 0A Interrupt Handler - PID Control Loop
 * 
 * Executes PID calculations at 100탎 intervals (10 kHz).
 * Averages 100 samples (10ms) for RPM estimation.
 */
void TIMER0A_Handler(void)
{
    time++;
    
    // Accumulate ADC samples for averaging
    sum += ADC_Read();
    count++;
    
    if (count == 100) // Every 10ms (100 samples at 100탎 intervals)
    {
        sampledADC_value = sum * 10; // Convert to mV and compute average
        estRPM = Current_speed(sampledADC_value); // Convert voltage to RPM
        
        error = estRPM - targetRPM; // Calculate speed error
        
        // PID Calculations with scaling
        P = (kP * error) / 20;
        I = I + (kI * error) / 640;
        D = kD * error;
        
        // Integrator anti-windup limits
        if (I < -500) I = -500;
        if (I > 4000) I = 4000;
        
        // Compute actuator command (PI only in current implementation)
        actuatorRPM = P + I;
        
        // Output saturation limits
        if (actuatorRPM < 100) actuatorRPM = 100;
        if (actuatorRPM > 19900) actuatorRPM = 19900;
        
        setMotorSpeed(actuatorRPM); // Apply control output
        sum = 0; // Reset accumulator for next averaging period
        count = 0;
    }
    
    TIMER0->ICR = 0x01; // Clear timer interrupt flag
}

/**
 * @brief   Main application entry point
 * @return  Exit code (never returns due to RTOS)
 * 
 * Initializes all hardware components and starts the RTOS scheduler.
 */
int main(void)
{
    // System Initialization
    OS_Init();                     // Initialize RTOS
    PI_Timer_Init();               // Set up PID control timer
    Init_LCD_Ports();              // Configure LCD GPIO
    Init_LCD();                    // Initialize LCD module
    Init_Keypad();                 // Initialize keypad
    PWM_Config(PWM_PERIOD, PWM_DUTY_CYCLE); // Configure PWM for motor
    ADC_Init();                    // Initialize ADC for speed sensing
    
    // Uncomment for motor testing without PID
    // setMotorDirectionFwd();
    // setMotorSpeed(DEFAULT_MOTOR_SPEED);
    
    // RTOS Setup and Launch
    OS_AddThreads(&retrieveInput, &updateLCD);
    OS_Launch(TIMESLICE); // Start scheduler (never returns)
    
    return 0; // Unreachable
}