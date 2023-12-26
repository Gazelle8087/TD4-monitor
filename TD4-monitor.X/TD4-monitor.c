/*
 * Copyright (c) 2023 @Gazelle8087
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
/*  PIC18F47Q43 TD4 ROM enmulator
 *  This single source file contains all code
 *
 *  written by Gazelle8087  https://github.com/Gazelle8087/TD4-monitor
 *  2023.12.26 not released yet
 */

// CONFIG1
#pragma config FEXTOSC = OFF	// External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF	// Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON		// PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON		// Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON		// Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#ifndef _18F47Q43
#pragma config JTAGEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF
#endif

// CONFIG3
#pragma config MCLRE = EXTMCLR	// MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON		// Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON		// IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF	// Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS	// Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9	// Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF		// ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF	// PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON		// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON			// Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF		// Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF		// WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC		// WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF		// Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF		// Storage Area Flash enable bit (SAF disabled)
#ifdef _18F47Q43
#pragma config DEBUG = OFF		// Background Debugger (Background Debugger disabled)	
#endif

// CONFIG8
#pragma config WRTB = OFF		// Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF		// Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF		// Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF		// SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF		// Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF		 // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

#ifndef _18F47Q43
// CONFIG9
#pragma config BOOTPINSEL = RC5 // CRC on boot output pin selection (CRC on boot output pin is RC5)
#pragma config BPEN = OFF       // CRC on boot output pin enable bit (CRC on boot output pin disabled)
#pragma config ODCON = OFF      // CRC on boot output pin open drain bit (Pin drives both high-going and low-going signals)

// CONFIG11
#pragma config BOOTSCEN = OFF   // CRC on boot scan enable for boot area (CRC on boot will not include the boot area of program memory in its calculation)
#pragma config BOOTCOE = HALT   // CRC on boot Continue on Error for boot areas bit (CRC on boot will stop device if error is detected in boot areas)
#pragma config APPSCEN = OFF    // CRC on boot application code scan enable (CRC on boot will not include the application area of program memory in its calculation)
#pragma config SAFSCEN = OFF    // CRC on boot SAF area scan enable (CRC on boot will not include the SAF area of program memory in its calculation)
#pragma config DATASCEN = OFF   // CRC on boot Data EEPROM scan enable (CRC on boot will not include data EEPROM in its calculation)
#pragma config CFGSCEN = OFF    // CRC on boot Config fuses scan enable (CRC on boot will not include the configuration fuses in its calculation)
#pragma config COE = HALT       // CRC on boot Continue on Error for non-boot areas bit (CRC on boot will stop device if error is detected in non-boot areas)
#pragma config BOOTPOR = OFF    // Boot on CRC Enable bit (CRC on boot will not run)

// CONFIG12
#pragma config BCRCPOLT = hFF   // Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of BCRCPOL are 0xFF)

// CONFIG13
#pragma config BCRCPOLU = hFF   // Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of BCRCPOL are 0xFF)

// CONFIG14
#pragma config BCRCPOLH = hFF   // Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of BCRCPOL are 0xFF)

// CONFIG15
#pragma config BCRCPOLL = hFF   // Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of BCRCPOL are 0xFF)

// CONFIG16
#pragma config BCRCSEEDT = hFF  // Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of BCRCSEED are 0xFF)

// CONFIG17
#pragma config BCRCSEEDU = hFF  // Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of BCRCSEED are 0xFF)

// CONFIG18
#pragma config BCRCSEEDH = hFF  // Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of BCRCSEED are 0xFF)

// CONFIG19
#pragma config BCRCSEEDL = hFF  // Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of BCRCSEED are 0xFF)

// CONFIG20
#pragma config BCRCEREST = hFF  // Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of BCRCERES are 0xFF)

// CONFIG21
#pragma config BCRCERESU = hFF  // Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of BCRCERES are 0xFF)

// CONFIG22
#pragma config BCRCERESH = hFF  // Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of BCRCERES are 0xFF)

// CONFIG23
#pragma config BCRCERESL = hFF  // Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of BCRCERES are 0xFF)

// CONFIG24
#pragma config CRCPOLT = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of CRCPOL are 0xFF)

// CONFIG25
#pragma config CRCPOLU = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of CRCPOL are 0xFF)

// CONFIG26
#pragma config CRCPOLH = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of CRCPOL are 0xFF)

// CONFIG27
#pragma config CRCPOLL = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of CRCPOL are 0xFF)

// CONFIG28
#pragma config CRCSEEDT = hFF   // Non-Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of CRCSEED are 0xFF)

// CONFIG29
#pragma config CRCSEEDU = hFF   // Non-Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of CRCSEED are 0xFF)

// CONFIG30
#pragma config CRCSEEDH = hFF   // Non-Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of CRCSEED are 0xFF)

// CONFIG31
#pragma config CRCSEEDL = hFF   // Non-Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of CRCSEED are 0xFF)

// CONFIG32
#pragma config CRCEREST = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of CRCERES are 0xFF)

// CONFIG33
#pragma config CRCERESU = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of CRCERES are 0xFF)

// CONFIG34
#pragma config CRCERESH = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of CRCERES are 0xFF)

// CONFIG35
#pragma config CRCERESL = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of CRCERES are 0xFF)
#endif

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>

#define _XTAL_FREQ 64000000UL

//	unsigned char TD4_ROM[16] ={
//	0xb7,0x01,0xe1,0x01,0xe3,0xb6,0x01,0xe6,	// Ramen timer
//	0x01,0xe8,0xb0,0xb4,0x01,0xea,0xb8,0xff,	//
//	};

	unsigned char TD4_ROM[16] ={
	0xb3,0xb6,0xbc,0xb8,0xb8,0xbc,0xb6,0xb3,	// Knight rider
	0xb1,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,	//
	};

unsigned char	TD4_PC;
unsigned char	rom_pointer;
unsigned char	rom_pointer2;
unsigned char	scan_pointer;
unsigned char	scan_pointer2;
//signed char	LED_CTL[32] = {0};	// 0xf000 - 0xf01f

// 7 segment hex font
const unsigned char led_font[16]={
	0b00011101, // 0
	0b00110000, // 1
	0b01101101, // 2
	0b01111001, // 3
	0b00110011, // 4
	0b01011011, // 5
	0b01011111, // 6
	0b01110000, // 7
	0b01111111, // 8
	0b01111011, // 9
	0b01110111, // A
	0b00011111, // b
	0b01001110, // C
	0b00111101, // d
	0b01001111, // E
	0b01000111  // F
};

// UART3 Transmit
void putch(char c) {
	while(!U3TXIF);		// Wait or Tx interrupt flag set
	U3TXB = c;			// Write data
}

// UART3 Recive
char getch(void) {
	while(!U3RXIF);		// Wait for Rx interrupt flag set
	return U3RXB;		// Read data
}

void led_matrix_write(unsigned char r1,unsigned char c1,unsigned char r2,unsigned char c2) {
	while(SPI1CON2bits.BUSY);		// BUSY?
	SPI1TCNT = 4;
	while(SPI1CON2bits.BUSY);		// BUSY?
	SPI1TXB = r2;
	while(SPI1CON2bits.BUSY);		// BUSY?
	SPI1TXB = c2;
	while(SPI1CON2bits.BUSY);		// BUSY?
	SPI1TXB = r1;
	while(SPI1CON2bits.BUSY);		// BUSY?
	SPI1TXB = c1;
}

void led_7seg_write(char r, char c) {

//	while(!SPI2STATUSbits.TXBE);	// TX buffer empty?
	while(SPI2CON2bits.BUSY);		// BUSY?
	SPI2TCNT = 2;
	while(SPI2CON2bits.BUSY);		// BUSY?
	SPI2TXB = r;
	while(SPI2CON2bits.BUSY);		// BUSY?
	SPI2TXB = c;
}

// Never called, logically
void __interrupt(irq(default),base(8)) Default_ISR(){}

/*=========================================================
	ROM data shown in LED matrix
	currently selected ROM data highlight(full duty lit)
	other ROM data half duty
*/
void __interrupt(irq(TMR0),base(8)) TMR0_ISR(){

	#define OFF_DELAY 3

	unsigned char	romdata1,romdata2,rom_a,a0,a1,a2;

	romdata1 = TD4_ROM[scan_pointer];
	romdata2 = TD4_ROM[scan_pointer+8];
	rom_a = (PORTA & 0x0f);

	a0 = scan_pointer+1;
	a1 = scan_pointer2+1;
	a2 = a1;
	
	if(rom_a == scan_pointer2){
		a1=0;									// force to NOP
	}else if (rom_a == scan_pointer2+8){
		a2=0;									// force to NOP
	}

	led_matrix_write(a0,romdata1,a0,romdata2);	// ON
	led_matrix_write(a1,0,a2,0);				// OFF after OFF_DELAY

	scan_pointer2=((scan_pointer - OFF_DELAY)&0x07);
	++scan_pointer;
	scan_pointer &= 0x07;

	TMR0IF = 0;
}
/*=========================================================
 * Clock rising edge interrupt
 
 */
void __interrupt(irq(IOC),base(8)) IOC_ISR(){

	TD4_PC = PORTA & 0x0f;
	LATD = TD4_ROM[TD4_PC];
	led_7seg_write(5,led_font[TD4_PC]);
	led_7seg_write(2,led_font[LATD>>4]);
	led_7seg_write(1,led_font[LATD&0x0f]);
	
	IOCIF = 0;
	IOCCF5 = 0;
}
//=================================================================
// main routine
void main(void) {

// System initialize
	OSCFRQ = 0x08;		// 64MHz internal OSC

// Ext_ROM_SELECT(RE0)
	ANSELE0 = 0;		// Disable analog function
	LATE0 = 1;			// Ext ROM on
//	WPUE0 = 0;			// weak pull up
//	ODCE0 = 1;			// open drain
	TRISE0 = 0;			// Set as output

// TD4 RESET(RE1)
	ANSELE1 = 0;		// Disable analog function
	LATE1 = 0;			// Reset = L
//	WPUE1 = 0;			// weak pull up
//	ODCE1 = 1;			// open drain
	TRISE1 = 1;			// Set as input (No use)

// TD4 clock(RE2)
	ANSELE2 = 0;		// Disable analog function
	LATE2 = 0;			// RESET = 0active
//	WPUE2 = 0;
	TRISE2 = 0;			// Set as output

// PORTB
	ANSELB = 0x00;		// Disable analog function
	WPUB = 0xff;		// Week pull up
	LATB = 0x00;		//
	TRISB = 0x0f;		// RC all input

// PORTC
	ANSELC = 0x00;		// Disable analog function
	WPUC = 0xdf;		// Week pull up
//	LATC = 0x00;		//
	TRISC = 0xff;		// RC all input

// PORTD(ROM data out)
	ANSELD = 0x00;		// Disable analog function
//	WPUD = 0xff;		// Week pull up
	LATD = 0x00;		//
	TRISD = 0x00;		// RD all out

// TD4-Address(RA0-3)
	ANSELA = 0;		// Disable analog function
	WPUA = 0xff;
	TRISA = 0x8f;		// Set as input

// CS1(RA4)
	ANSELA4 = 0;		// Disable analog function
	LATA4 = 0;			// Default level
	TRISA4 = 0;			// Set as out

// CS2(RA5)
	ANSELA5 = 0;		// Disable analog function
	LATA5 = 0;
	TRISA5 = 0;			// Set as intput

//==== TD4 clock(RE2) by NCO3 FDC mode=========
	RE2PPS = 0x41;		// RE2 asign NCO3
	ANSELE2 = 0;		// Disable analog function
	WPUE2 = 0;
	TRISE2 = 0;			// NCO output pin
//	NCO3INC = 67;		// 1Hz
	NCO3INC = 671;		// 10Hz
//	NCO3INC = 6711;		// 100Hz
	NCO3CLK = 0x04;		// Clock source MFINTOSC(31.25kHz)
	NCO3PFM = 0;		// FDC mode
	NCO3OUT = 1;		// NCO output enable
	NCO3EN = 1;			// NCO enable

//======= UART setup =====================================
// UART3 initialize
	U3BRG = 416;		// 9600bps @ 64MHz
	U3RXEN = 1;			// Receiver enable
	U3TXEN = 1;			// Transmitter enable

// UART3 Receiver
	ANSELA7 = 0;		// Disable analog function
	WPUA7 = 1;			// weak pull up
	TRISA7 = 1;			// RX set as input
	U3RXPPS = 0x07;		//RA7->UART3:RX3;

// UART3 Transmitter
	ANSELA6 = 0;		// Disable analog function
	LATA6 = 1;			// Default level
	TRISA6 = 0;			// TX set as output
	RA6PPS = 0x26;		//RA6->UART3:TX3;

	U3ON = 1;			// Serial port enable

//======= SPI setup =======================================
	SPI1CON0 = 0x03;		// EN=0, LSBF=0; MST=1, BMODE=1
	SPI1CON1 = 0xd4;		// SMP=1, CKE=1, CKP=0, FST=1, SSP=1, SDOP=0, SDIO=0
	SPI1CON2 = 0x02;		// SSET=0, TXR=1, RXR=0
	SPI1CLK = 0;			// Clock source Fosc
	SPI1BAUD = 3;			// bauid rate 8MHz 64MHz/(2(SPIBAUD+1))	
	SPI1TWIDTH = 0;			// 8bit
	SPI1CON0bits.EN = 1;	// EN=1	
	RB4PPS = 0x31;			// SPI1SCK
	RB6PPS = 0x32;			// SPI1SDO
	RA4PPS = 0x33;			// SPI1SS

	SPI2CON0 = 0x03;		// EN=0, LSBF=0; MST=1, BMODE=1
	SPI2CON1 = 0xd4;		// SMP=1, CKE=1, CKP=0, FST=1, SSP=1, SDOP=0, SDIO=0
	SPI2CON2 = 0x02;		// SSET=0, TXR=1, RXR=0
	SPI2CLK = 0;			// Clock source Fosc
	SPI2BAUD = 3;			// bauid rate 8MHz 64MHz/(2(SPIBAUD+1))	
	SPI2TWIDTH = 0;			// 8bit
	SPI2CON0bits.EN = 1;	// EN=1
	RB5PPS = 0x34;			// SPI2SCK
	RB7PPS = 0x35;			// SPI2SDO
	RA5PPS = 0x36;			// SPI2SS

//========= TMT0 setup =====================================
	T0CON1 = 0x90;			// clk=LFINTOSC(31kHz), Async, no prescaler
	TMR0H = 60;				// approx. 500Hz interval
	T0CON0 = 0x80;			// EN, 8bit, 1:1 postscaler

//========= IOCC5 setup =====================================
	IOCCP5 = 1;				// RC5(TD4 clock) rising edge

//=========================================================

	printf("\r\nTD4 ROM emulator and monitor %2.2fHz\r\n",NCO3INC/67.1089);

// Unlock IVT
	IVTLOCK = 0x55;
	IVTLOCK = 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x00;

// Default IVT base address
	IVTBASE = 0x000008;

// Lock IVT
	IVTLOCK = 0x55;
	IVTLOCK = 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x01;

	led_matrix_write(0x0f, 0x00, 0x0f, 0x00);	// display test , normal operation
	led_matrix_write(0x0a, 0x01, 0x0a, 0x01);	// Intensity , Duty Cycle 0x00=1/32 .. 0x0F=31/32
	led_matrix_write(0x0b, 0x07, 0x0b, 0x07);	// scan limit , display 0 to 7
	led_matrix_write(0x0c, 0x01, 0x0c, 0x01);	// shutdown mode register , normal operation
	led_matrix_write(0x09, 0x00, 0x09, 0x00);	// No decode for digits 0-7
//	led_off();

	led_7seg_write(0x0f, 0x00);	// display test , normal operation
	led_7seg_write(0x0a, 0x00);	// Intensity , Duty Cycle 0x00=1/32 .. 0x0F=31/32
	led_7seg_write(0x0b, 0x07);	// scan limit , display 0 to 7
	led_7seg_write(0x0c, 0x01);	// shutdown mode register , normal operation
	led_7seg_write(0x09, 0x00);	// No decode for digits 0-7

	led_7seg_write(0x01,0);
	led_7seg_write(0x02,0);
	led_7seg_write(0x03,0);
	led_7seg_write(0x04,0);
	led_7seg_write(0x05,0);
	led_7seg_write(0x06,0);
	led_7seg_write(0x07,0);
	led_7seg_write(0x08,0);

// CLC VI enable

	GIE = 1;		// Global interrupt enable
	U3RXIE = 1;
	TMR0IE = 1;
	IOCIE = 1;

//	LATE1 = 1;		// Release reset TD4 start

	while(1);       // invoke /INT to 8085
}
