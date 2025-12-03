#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include "ADC.h"
#include "motor.h"

#define TIMESLICE 32000		 // Timeslice of 2 ms
#define CPU_HZ 16000			 // CPU Clock Hz
#define PWM_PERIOD 4000		 // PWM period
#define PWM_DUTY_CYCLE 2000	 // PWM duty cycle
#define TIMER_RELOAD_VALUE 0 // Timer reload value

<<<<<<< Updated upstream
#define DEFAULT_MOTOR_SPEED 2000 // Default motor speed

// Threads
void retrieveInput(void);
void updateLCD(void);
void motorPIDControlLoop(void);

// OS Functions
=======
// PID Controller Gains (tuning parameters - MUST TUNE THESE!)
#define KP                 1.0f       // Proportional gain (start small)
#define KI                 0.1f       // Integral gain (start very small)
#define KD                 0.01f      // Derivative gain (optional)

#define SAMPLE_TIME_MS     10         // PID update period in milliseconds
#define DEFAULT_MOTOR_SPEED 2000      // Default motor speed for testing

// Motor physical limits (adjust based on your hardware)
#define MOTOR_RPM_MIN      100
#define MOTOR_RPM_MAX      19900

// Thread Function Declarations
void retrieveInput(void);             // Handles keypad input
void updateLCD(void);                 // Updates LCD display

// RTOS Function Declarations
>>>>>>> Stashed changes
void OS_Init(void);
int OS_AddThreads(void (*task0)(void), void (*task1)(void), void (*task2)(void));
void OS_Launch(uint32_t theTimeSlice);

// OS Semaphore Functions
void OS_Signal(int32_t *s);
void OS_Sleep(uint32_t SleepCtr);
void OS_Suspend(void);
void OS_Wait(int32_t *s);

void TIMER0A_Handler(uint32_t reload);

<<<<<<< Updated upstream
// Key Input Variables
uint8_t Key_ASCII;
int32_t sampledADC_value;
=======
// PID Controller Variables
volatile int32_t estRPM, targetRPM;   // Speed tracking
volatile float error;                 // PID error (use float for precision)
volatile float P_term, I_term, D_term; // PID components
volatile float actuatorRPM;           // Calculated motor speed adjustment
volatile int32_t sum;                 // ADC sample accumulator
volatile int time;                    // Time counter in 0.1 ms units
volatile int count;                   // Sample counter for averaging

// PID state variables
volatile float integral = 0.0f;       // Integral accumulator
volatile float prev_error = 0.0f;     // Previous error for derivative
volatile float prev_estRPM = 0.0f;    // Previous RPM for derivative on measurement
volatile uint32_t last_pid_time = 0;  // Last PID update time
>>>>>>> Stashed changes

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

<<<<<<< Updated upstream
// Keypad Initialization and Utility Functions
extern void(Init_Keypad(void));
extern void Scan_Keypad(void);
extern uint8_t Key_ASCII;
=======
// ADC Data
volatile uint32_t sampledADC_value = 0; // Latest sampled ADC value
>>>>>>> Stashed changes

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

<<<<<<< Updated upstream
static inline void Delay100us(void)
{
    const uint32_t cycles = (CPU_HZ / 10000u); // 100us worth of cycles
=======
/**
 * @brief  Creates a precise 100µs delay using the CPU cycle counter
 * @note   Uses DWT cycle counter for accurate timing
 */
static inline void Delay100us(void)
{
    const uint32_t cycles = (CPU_HZ / 10000u); // Calculate cycles for 100µs
>>>>>>> Stashed changes
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

<<<<<<< Updated upstream
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
=======
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

/**
 * @brief   Clamp a value between minimum and maximum limits
 */
static inline float clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
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
            if (Keypress_Buffer < 0) // Buffer is empty
            {
                Keypress_Buffer = Key_ASCII - 48; // Convert ASCII to digit
            }
            else if (Keypress_Buffer < 1000) // Limit to 3 digits
            {
                Keypress_Buffer = Keypress_Buffer * 10 + Key_ASCII - 48;
            }
            else // Buffer full, set as target RPM
            {
                targetRPM = clamp(Keypress_Buffer, MOTOR_RPM_MIN, MOTOR_RPM_MAX);
                Keypress_Buffer = -1; // Clear buffer
            }
        }
        else if (k == '#') // Confirm entry
        {
            if (Keypress_Buffer < 0)
            {
                targetRPM = DEFAULT_MOTOR_SPEED; // Default if buffer empty
            }
            else
            {
                targetRPM = clamp(Keypress_Buffer, MOTOR_RPM_MIN, MOTOR_RPM_MAX);
            }
            Keypress_Buffer = -1; // Clear buffer after confirmation
        }
        else if (k == 'C') // Clear input
        {
            Keypress_Buffer = -1; // Reset buffer
        }
        
        OS_Sleep(80); // Debounce delay and yield to scheduler
    }
>>>>>>> Stashed changes
}

void updateLCD(void)
{
<<<<<<< Updated upstream
	
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
=======
    while (1)
    {
        char inbuf[4], tbuf[4], cbuf[4]; // Buffers for number conversions
        
        // First display line: current input
        Set_Position(0x00);
        Display_Msg("Input: ");
        if (Keypress_Buffer >= 0) // Display input if present
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
        
        // Optional: Display error or PID terms for debugging
        Set_Position(0x10);
        Display_Msg("Err: ");
        char errbuf[6];
        Hex2ASCII(errbuf, (int32_t)error);
        Display_Msg(errbuf);
        
        OS_Sleep(200); // Update display every 200ms
    }
}

/**
 * @brief Timer 0A Interrupt Handler - PID Control Loop
 * 
 * Executes PID calculations at 100µs intervals (10 kHz).
 * Averages 100 samples (10ms) for RPM estimation.
 */
void TIMER0A_Handler(void)
{
    static uint32_t sample_count = 0;
    
    time++;
    
    // Accumulate ADC samples for averaging
    sum += ADC_Read();
    count++;
    
    // Every SAMPLE_TIME_MS (10ms = 100 samples at 100µs intervals)
    if (count >= 100) 
    {
        // Calculate average ADC value
        sampledADC_value = sum * 10;
        
        // Convert voltage to RPM
        estRPM = Current_speed(sampledADC_value);
        
        // --- FIXED PID CALCULATIONS ---
        
        // Calculate error (convention: target - actual)
        error = (float)targetRPM - (float)estRPM;
        
        // Calculate sample time in seconds (10ms = 0.01s)
        float sample_time = 0.01f;
        
        // PROPORTIONAL TERM
        P_term = KP * error;
        
        // INTEGRAL TERM with anti-windup
        float new_integral = integral + (KI * error * sample_time);
        
        // Anti-windup: limit integral contribution
        float max_integral = 0.5f * (float)MOTOR_RPM_MAX; // Integral limited to 50% of max RPM
        if (new_integral > max_integral) new_integral = max_integral;
        if (new_integral < -max_integral) new_integral = -max_integral;
        integral = new_integral;
        
        I_term = integral;
        
        // DERIVATIVE TERM (on measurement to avoid derivative kick)
        float rpm_derivative = 0.0f;
        if (sample_time > 0) {
            // Derivative of measurement (not error) to avoid spikes on setpoint changes
            rpm_derivative = ((float)prev_estRPM - (float)estRPM) / sample_time;
        }
        D_term = KD * rpm_derivative;
        
        // Update previous values for next iteration
        prev_error = error;
        prev_estRPM = estRPM;
        
        // Calculate output
        actuatorRPM = P_term + I_term + D_term;
        
        // Clamp output to physical motor limits
        if (actuatorRPM < MOTOR_RPM_MIN) actuatorRPM = MOTOR_RPM_MIN;
        if (actuatorRPM > MOTOR_RPM_MAX) actuatorRPM = MOTOR_RPM_MAX;
        
        // Apply control output
        setMotorSpeed((int32_t)actuatorRPM);
        
        // Reset for next averaging period
        sum = 0;
        count = 0;
        sample_count++;
    }
    
    TIMER0->ICR = 0x01; // Clear timer interrupt flag
}

/**
 * @brief Initialize Timer0A for PID control loop
 */
void PI_Timer_Init(void)
{
    SYSCTL->RCGCTIMER |= 0x01; // Enable Timer0 clock
    
    // Ensure timer is disabled before configuration
    TIMER0->CTL &= ~0x01;
    
    // Configure as 32-bit periodic timer
    TIMER0->CFG = 0x00000000; // 32-bit timer configuration
    TIMER0->TAMR = 0x00000002; // Periodic mode
    
    // Set reload value for 100µs interrupts (10 kHz)
    // Assuming 16 MHz system clock: 16,000,000 Hz / 10,000 Hz = 1600 ticks
    TIMER0->TAILR = 1600 - 1; // 100µs period
    
    // Enable timer interrupt
    TIMER0->IMR |= 0x01; // Enable timeout interrupt
    
    // Clear any pending interrupts
    TIMER0->ICR = 0x01;
    
    // Configure NVIC for Timer0A
    NVIC->ISER[0] |= 1 << 19; // Enable Timer0A interrupt (IRQ19)
    NVIC->IP[19] = 0x20; // Set priority (adjust as needed)
    
    // Enable timer
    TIMER0->CTL |= 0x01;
    
    // Initialize PID state
    integral = 0.0f;
    prev_error = 0.0f;
    prev_estRPM = 0.0f;
    error = 0.0f;
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
    
    // Initialize target RPM
    targetRPM = DEFAULT_MOTOR_SPEED;
    
    // RTOS Setup and Launch
    OS_AddThreads(&retrieveInput, &updateLCD);
    OS_Launch(TIMESLICE); // Start scheduler (never returns)
    
    return 0; // Unreachable
}
>>>>>>> Stashed changes
