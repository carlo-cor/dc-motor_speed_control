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
int32_t sampledADC_value;

// PID Timer Handler Variables
int32_t estRPM, targetRPM, error; // Speed variables
int32_t P, I, D;    // PID variables
int32_t kP = 1;
int32_t kI = 1;
int32_t kD = 1; // PID constants
int32_t actuatorRPM; // Speed error adjustment
uint32_t time;             // Time in 0.1 ms
uint32_t count;            // Counter for time

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
extern void(Hex2ASCII(char *s,uint32_t n ));
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
      
      sampledADC_value = sum/100;                             // Averaged sampled ADC value
      targetRPM = Current_speed(sampledADC_value);  // Get target RPM speed using voltage conversion function
      
      // If sampled ADC value is below zero for whatever reason, no RPM
      if(targetRPM < 0){
            targetRPM = 0;
      }
      
      // continue..
      
      // Keypad Input
      while (1)
      {
            Scan_Keypad();                                                // Blocks until a key is pressed
            uint8_t k = Key_ASCII;                                  // Takes in value from keypad press
            // unsigned char key_value = (unsigned char)(k); // Cast value into readable character
            
            if (k >= '0' && k <= '9')
            {
                  // handle digits
                  if(Keypress_Buffer <-1)// -1 means the buffer is empty
                  {
                        Keypress_Buffer = Key_ASCII-48;
                        
                  }
                  else if(Keypress_Buffer <1000){
                  Keypress_Buffer = Keypress_Buffer* 10 + Key_ASCII-48;
                        
                  }
                  else{
                        sampledADC_value  = Keypress_Buffer;
                        Keypress_Buffer = -1;
                        
                  }
            }
            else if (k == '#')
            {
                  
                  sampledADC_value  = Keypress_Buffer;
                  // confirm input
                  
            }
            else if (k == 'C')
            {
                  // clear input
                  Keypress_Buffer = -1; // -1 means the buffer is empty
            }

            OS_Sleep(20); // small debounce + yield
      }
}


extern uint32_t Key_ASCII;

uint8_t getKeypadChar(void)
{
    Scan_Keypad();   // assembly ? blocks until a valid key
    return (uint8_t)(*(volatile uint32_t*)&Key_ASCII);
}
int32_t Keypress_Buffer = -1;

void processKey(uint8_t k)
{
    if(k >= '0' && k <= '9')
    {
        if(Keypress_Buffer < 0)
        {
            Keypress_Buffer = k - '0';
        }
        else
        {
            Keypress_Buffer = Keypress_Buffer * 10 + (k - '0');
        }
    }
    else if(k == '#')
    {
        sampledADC_value = Keypress_Buffer;
        Keypress_Buffer = -1;
    }
    else if(k == 'C')
    {
        Keypress_Buffer = -1;
    }
}
void updateLCD(void)
{
    char inbuf[8], tbuf[8], cbuf[8];

    // Row 1
    Set_Position(0x00);
    Display_Msg("Input: ");

    if(Keypress_Buffer >= 0)
    {
        Hex2ASCII(inbuf, Keypress_Buffer);
        Display_Msg(inbuf);
    }

    // Row 2
    Set_Position(0x40);

    Display_Msg("T:");
    Hex2ASCII(tbuf, targetRPM);
    Display_Msg(tbuf);

    Display_Msg(" C:");
    Hex2ASCII(cbuf, estRPM);
    Display_Msg(cbuf);
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
       while(1)
    {
        // ---- Read ADC and convert ----
        uint8_t adc = getADCavg();
        targetRPM = Current_speed(adc);
        if(targetRPM < 0) targetRPM = 0;

        // ---- Read keypad ----
        uint8_t key = getKeypadChar();  // blocks until pressed
        processKey(key);

        // ---- Update LCD ----
        updateLCD();
    }
      // OS_AddThreads(&retrieveInput, &updateLCD);
      // OS_Launch(TIMESLICE);

      return 0;
}