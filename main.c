
#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>

void Init_LCD_Ports(void);
void Init_LCD(void); 
void Set_Position(uint32_t POS);
void Display_Msg(char *Str);
void OS_Sleep(uint32_t SleepCtr);
int length = 0;
uint32_t SW5;
int32_t SW1;

int speed = 0;
int dir = 0;

void OS_Init(void);
int OS_AddThreads(void(*task0)(void),void(*task1)(void),void(*task2)(void));
void OS_Launch(uint32_t theTimeSlice);  
void LED_display(void);
void LCD_Display(void);
void GetInput(void);
void OS_Signal(int32_t *s);
void OS_Suspend(void);
void OS_Wait(int32_t *s);

void TIMER0A_Handler(uint32_t reload);

void PWM_Config(uint16_t period, uint16_t high);

//PWM variables
uint16_t period = 2500;
uint16_t high = 2474;

uint32_t reload = 0;

uint8_t Key_ASCII;

int *startPt;
int *currentPt;



//initialize keypad and scan 
	extern void (Init_Keypad(void));
	extern void Scan_Keypad(void);
	extern uint8_t Key_ASCII;
		
	//LCD initialize
	extern void Init_LCD_Ports(void);
	extern void Init_LCD(void);
	extern void Display_Msg(char *s);
	extern void Display_Char(char c);
	extern void Set_Position(uint32_t pos);
	extern void Set_Blink_ON(uint32_t pos);
	extern void Set_Blink_OFF(void);
	extern void Delay1ms(uint32_t n);
	
int main(void){
 
	OS_Init();           // initialize, disable interrupts, 16 MHz
  SYSCTL_RCGCGPIO_R |= 0x3F;            // activate clock for all ports
  while((SYSCTL_RCGCGPIO_R&0x3F) == 0){} // allow time for clock to stabilize
  GPIOB->DIR |= 0x03;             // make PB1-0 input
  GPIOB->DEN |= 0x03;             // enable digital I/O on PB1-0
	GPIOF->DIR |= 0x04;								// make PF2 output
	GPIOF->DEN |= 0x04;              // enable digital I/O on PF2
	Init_LCD_Ports();
	Init_LCD();
	TIMER0A_Handler(reload);
	PWM_Config(period, 1500);
			
		
	//LCD display
	char tbuf[6], cbuf[6], tpad[6], cpad[6];
	Set_Position(0x00);
	Display_Msg("Input: ");
	Display_Msg("INS Speed");
	Set_Position(0x40);
	Display_Msg("T: ");
	Display_Msg("INS ");
	Display_Msg("C: ");
	Display_Msg("INS");
		
	while (1) {
        Scan_Keypad();     // blocks until a key is pressed
        uint8_t k = Key_ASCII;

        // Now you can use k in your logic
        if (k >= '0' && k <= '9') {
            // handle digits
        }
        else if (k == '#') {
            // confirm input
        }
        else if (k == 'C') {
            // clear input
        }

        OS_Sleep(20);   // small debounce + yield
    }
	
  //OS_AddThreads(&GetInput, &LCD_Display, &LED_display );
  //OS_Launch(32000); 
	return 0;
}


void PWM_Config(uint16_t period, uint16_t high){ 
	SYSCTL->RCGCPWM |= 0x02; //set bit 1
	SYSCTL->RCGCGPIO |= 0x20; //activate PortF
	while((SYSCTL->PRGPIO & 0x20)==0){};
	GPIOF->AFSEL |= 0x04; //activate PF2 
	GPIOF->PCTL &= ~0x00000F00; //confiure PF2 as PWM1
	GPIOF->PCTL |= 0x00000500;
	GPIOF->AMSEL &= ~0x04;	//disable analog functionality on PF2
	GPIOF->DEN |= 0x04;	//Enable digital I/O on PF2
	SYSCTL->RCC = 0x001E0000 | (SYSCTL->RCC & (~0x00E70000)); // use system clock for PWM
	PWM1->_3_CTL = 0; // disable PWM1_3 during configuration
	PWM1->_3_GENA = 0x00000C8; // output low for load, high for match
	PWM1->_3_LOAD = period-1; //period is 2500
	PWM1->_3_CMPA = high-1; //high =2474 99%duty cycle
	PWM1->ENABLE |= 0x40; // enable PWM1
	PWM1->_3_CTL |= 1; // enable PWM1_3
}


void ADC_Init(void){
		SYSCTL->RCGCGPIO |= 0x12; //Port E and Port B
		while((SYSCTL->PRGPIO & 0x12) == 0) {}
			
		GPIOB->DIR &= ~0xFC; //PB2-7 are inputs
		GPIOB->DEN |= 0xFC; 
		GPIOB->AMSEL &= ~0xFC;
		GPIOB->AFSEL &= ~0xFC;
		
		GPIOE->DIR &= ~0x06; //PE1-2 are inputs
		GPIOE->DEN |= 0x06; 
		GPIOE->AMSEL &= ~0x06;
		GPIOE->AFSEL &= ~0x06;
			
		GPIOE->DIR &= ~0x10; //PE4-Busy (pin 21)
		GPIOE->DEN |= 0x10; 
			
		GPIOE->DIR |= 0x20; //PE5-BYTE (pin 24)
		GPIOE->DEN |= 0x20; 
		GPIOE->DATA |= 0x20;
	
	
}


void TIMER0A_Handler(uint32_t reload){
}

