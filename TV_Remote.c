#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "UART.h"

#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOB			0x00000002	// port B Clock Gating Control
#define SYSCTL_TCGC2_FPIOA			0x00000001	// port A Clock Gating Control
#define CR   0x0D
#define LF   0x0A
#define BS   0x08

/************************************************************************************************
	FUNCTION CALLS & VARIABLE(S)
************************************************************************************************/
char read[5];
int i = 0;
long device_number = 0;

/************************************************************************************************
	PORT B INIT
************************************************************************************************/
void PortB_Init(void) { volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000002;     // B clock
	delay = SYSCTL_RCGC2_R;           // delay
	GPIO_PORTB_CR_R = 0xFF;						// allow changes to PB7-PB0
	GPIO_PORTB_AMSEL_R = 0x00;			  // disable analog function
	GPIO_PORTB_PCTL_R = 0x00000000;   // GPIO clear bit PCTL
	GPIO_PORTB_DIR_R = 0xFF;					// PB7-PB0 Output
	GPIO_PORTB_AFSEL_R = 0x00;				// no alt function
	GPIO_PORTB_DEN_R = 0xFF;					// enable digital pins PB7-PB0
}

/************************************************************************************************
	PORT F INIT
************************************************************************************************/
void PortF_Init(void){      
	
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0   
  GPIO_PORTF_IS_R &= ~0x10;     		// (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    		//     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    		//     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      		// (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      		// (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      		// (h) enable interrupt 30 in NVIC
}

void ColorChange(long n) {
	switch(n) {
		case 0:
			GPIO_PORTF_DATA_R = 0x02;
			break;
		case 1:
			GPIO_PORTF_DATA_R = 0x08;
			break;
		case 2:
			GPIO_PORTF_DATA_R = 0x04;
			break;
		case 3:
			GPIO_PORTF_DATA_R = 0x0E;
			break;
		
		default:
			break;
	}
}

void GPIOPortF_Handler(void){
	if ((GPIO_PORTF_RIS_R & 0x10)){ // sw1 is pressed
		GPIO_PORTF_ICR_R = 0x10;
		if(device_number == 3) {
			device_number = 0;
		}
		else {
			device_number++;
		}
		ColorChange(device_number);
	}
}

void CRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

/************************************************************************************************
	SYSTICK
************************************************************************************************/
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}

void SysTick_Wait(unsigned long delay){
  volatile unsigned long elapsedTime;
  unsigned long startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}

void SysTick_Wait10ms(unsigned long delay){
  unsigned long j;
  for(j=0; j<delay; j++){
    SysTick_Wait(500000);  // wait 10ms (assumes 50 MHz clock)
  }
}

/************************************************************************************************
	INFRARED MODULATION
************************************************************************************************/
void HighSignal(void) {
	GPIO_PORTB_DATA_R = 0xFF;
	SysTick_Wait(600);
	GPIO_PORTB_DATA_R = 0x00;
	SysTick_Wait(600);
}

void LowSignal(void) {
	GPIO_PORTB_DATA_R = 0x00;
	SysTick_Wait(600);
	GPIO_PORTB_DATA_R = 0x00;
	SysTick_Wait(600);
}

// 900us high = 34 pulses
// 450us low = 17 pulses
void Logic1(void)	{ 
	for(i = 0; i < 34; i++) {
		HighSignal();
	}
	
	for(i = 0; i < 17; i++) {
		LowSignal();
	}
}

// 450us high = 17 pulses
// 450us low = 17 pulses
void Logic0(void)	{ 
	for(i = 0; i < 17; i++) {
		HighSignal();
	}
	
	for(i = 0; i < 17; i++) {
		LowSignal();
	}
}

// 1ms high = 38 pulses
// 800us low = 30 pulses
void StartPulse(void)	{ 
	for(i = 0; i < 38; i++) {
		HighSignal();
	}
	
	for(i = 0; i < 30; i++) {
		LowSignal();
	}
}

void ModulateAddress(long n) {
	switch(n) {
		case 0:
			Logic0();
			Logic0();
			break;
		case 1:
			Logic0();
			Logic1();
			break;
		case 2:
			Logic1();
			Logic0();
			break;
		case 3:
			Logic1();
			Logic1();
			break;
		
		default:
			break;
	}
}

void ModulateCommand(char c) {
	switch(c) {
		case '0':
			Logic0();
			Logic0();
			Logic0();
			CRLF();
			break;
		case '1':
			Logic0();
			Logic0();
			Logic1();
			CRLF();
			break;
		case '2':
			Logic0();
			Logic1();
			Logic0();	
			CRLF();
			break;
		case '3':
			Logic0();
			Logic1();
			Logic1();
			CRLF();
			break;
		case '4':
			Logic1();
			Logic0();
			Logic0();
			CRLF();
			break;
		case '5':
			Logic1();
			Logic0();
			Logic1();
			CRLF();
			break;
		case '6':
			Logic1();
			Logic1();
			Logic0();
			CRLF();
			break;
		case '7':
			Logic1();
			Logic1();
			Logic1();
			CRLF();
			break;
		
		default:
			break;
	}
}

void test(long n) {
	switch(n) {
		case 0:
			read[0] = '0';
			break;
		case 1:
			read[0] = '1';
			break;
		case 2:
			read[0] = '2';
			break;
		case 3:
			read[0] = '3';
			break;
		case 4:
			read[0] = '4';
			break;
		case 5:
			read[0] = '5';
			break;
		case 6:
			read[0] = '6';
			break;
		case 7:
			read[0] = '7';
			break;
		
		default:
			break;
	}
}

/************************************************************************************************
	MAIN LOOP
************************************************************************************************/
int main(void){
	PLL_Init();
	PortB_Init();
	PortF_Init();
	UART_Init();
	SysTick_Init();
	GPIO_PORTF_DATA_R = 0x02;
	UART_OutString("\r\nEnter a command:\r\n");
	UART_InString(read, 5);
	CRLF();
	while(1){
		StartPulse();
		ModulateAddress(device_number);
		ModulateCommand(read[0]);
		SysTick_Wait(60000);
	}
}

// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06
