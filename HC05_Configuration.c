#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"

#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW1       0x01                      // on the left side of the Launchpad board
#define SW2       0x10                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOB			0x00000002	// port B Clock Gating Control
#define SYSCTL_TCGC2_FPIOA			0x00000001	// port A Clock Gating Control
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define YELLOW    0x0A
#define SKYBLUE   0x0C
#define WHITE     0x0E
#define CR   0x0D
#define LF   0x0A
#define BS   0x08

/************************************************************************************************
	FUNCTION CALLS & VARIABLE(S)
************************************************************************************************/
unsigned char n = '0';

/************************************************************************************************
	UNIVERSAL ASYCHRONOUS RECEIVE TRANSMIT - SPEAKING TO BLUETOOTH
************************************************************************************************/
void UART5_Init(void){
	unsigned volatile delay;
	SYSCTL_RCGCUART_R |= 0x0020; 			// activate UART5
	SYSCTL_RCGCGPIO_R |= 0x0010;			// activate GPIOE
	delay = SYSCTL_RCGCGPIO_R;
	UART5_CTL_R &= ~0x0001;						// disable UART5
	UART5_IBRD_R = 81;								// IBRD, 50MHz, 9600 baud rate (50MHz / (16 * 9600))
	UART5_FBRD_R = 24;							  // FBRD, (0.83333 * 64 + .05)
	UART5_LCRH_R = 0x0070;						// 8 bit (no parity, one stop, no FIFOs)
	UART5_CTL_R |= 0x0001; 						// enable UART5
	GPIO_PORTE_AFSEL_R |= 0x30;				// enable alt funct on PE5
	GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & ~0x00FF0000) | 0x00110000; 
	GPIO_PORTE_AMSEL_R &= ~0x30;			// disable analog funct on PE5
	GPIO_PORTE_DEN_R |= 0x30;					// enable digital I/O on PE5
}

unsigned char UART5_InChar(void){
  while((UART5_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART5_DR_R&0xFF));
}

void UART5_OutChar(unsigned char data){
  while((UART5_FR_R&UART_FR_TXFF) != 0);
  UART5_DR_R = data;
}

/************************************************************************************************
	UNIVERSAL ASYCHRONOUS RECEIVE TRANSMIT - Used with USB
************************************************************************************************/

#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define UART0_DR_R              (*((volatile unsigned long *)0x4000C000))
#define UART0_FR_R              (*((volatile unsigned long *)0x4000C018))
#define UART0_IBRD_R            (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R            (*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH_R            (*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL_R             (*((volatile unsigned long *)0x4000C030))
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8        0x00000060  // 8 bit word length
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_CTL_UARTEN         0x00000001  // UART Enable
#define SYSCTL_RCGC1_R          (*((volatile unsigned long *)0x400FE104))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC1_UART0      0x00000001  // UART0 Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control

void UART0_Init(void){
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 27;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
  UART0_FBRD_R = 8;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= UART_CTL_UARTEN;       // enable UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1-0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1-0
                                        // configure PA1-0 as UART
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA
}

unsigned char UART0_InChar(void){
  while((UART0_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART0_DR_R&0xFF));
}

void UART0_OutChar(unsigned char data){
  while((UART0_FR_R&UART_FR_TXFF) != 0);
  UART0_DR_R = data;
}

void respondFromBT(void) {
	n = UART5_InChar();
	UART0_OutChar(n);
	while(n != '\n') {
		n = UART5_InChar();
		UART0_OutChar(n);
	}
}

void sendCommandBT(char* string) {
	while(*string)
		UART5_OutChar(*(string++));
}

void UART_transmit_String(char* string){
    while(*string)
			UART0_OutChar(*(string++));
}

/************************************************************************************************
	DELAY
************************************************************************************************/
void Delay(void){unsigned long volatile time;
  time = 727240*200/91;  // 0.1sec
  while(time){
    time--;
  }
}

/************************************************************************************************
	MAIN LOOP
************************************************************************************************/
int main(void){
	PLL_Init();
	UART5_Init();
	UART0_Init();
	Delay();

	UART_transmit_String("Lab bluetooth configuration\r\n");
	UART_transmit_String("AT+NAME=Vince\r\n");
	sendCommandBT("AT+NAME=Vince\r\n");
	respondFromBT();
	Delay();
	
	UART_transmit_String("AT+UART=57600,0,2\r\n");
	sendCommandBT("AT+UART=57600,0,2\r\n");
	respondFromBT();
	Delay();
	
	UART_transmit_String("AT+PSWD=1130\r\n");
	sendCommandBT("AT+PSWD=1130\r\n");
	respondFromBT();
	Delay();
	
	UART_transmit_String("AT+ROLE=0\r\n");
	sendCommandBT("AT+ROLE=0\r\n");
	respondFromBT();
	Delay();
	
	while(1){
		Delay();
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
