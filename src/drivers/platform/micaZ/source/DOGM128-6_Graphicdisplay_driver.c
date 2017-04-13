/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2011 Technische Universität München (www.vmi.ei.tum.de)
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
*  Driver for Electronic Assembly DOGM128-6 Full Graphic Display
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/

/*


Author: 	P. Diener
Target: 	AtMega128(L)

The display has to be connected to usart1 spi
chipselect is
A0 is

example of usage: 	    LCD_print("Peter Diener", 5*6, 0);

***************************************

copyright 2008 - 2009

*/

#define F_CPU 7372800
#include "avr/io.h"
#include "util/delay.h"
#include <avr/pgmspace.h>

#include "DOGM128-6_Graphicdisplay_driver.h"

#include <font5x7.h>
//Always read this font with pgm_read_byte(&(font5x7[character][i]))


#define DisA0low		do {PORTC &= ~(1<<0);} while(0)
#define DisA0high		do {PORTC |=  (1<<0);} while(0)
#define DisCSlow		do {PORTC &= ~(1<<2);} while(0)
#define DisCShigh		do {PORTC |=  (1<<2);} while(0)
#define DisRESETlow		do {PORTC &= ~(1<<1);} while(0)
#define DisRESEThigh	do {PORTC |=  (1<<1);} while(0)
#define DisBLIGHTon		do {PORTB |=  (1<<4);} while(0)
#define DisBLIGHToff	do {PORTB &= ~(1<<4);} while(0)

#define DisSCLlow		do {PORTD &= ~(1<<5);} while(0)
#define DisSCLhigh		do {PORTD |=  (1<<5);} while(0)
#define DisSOlow		do {PORTD &= ~(1<<3);} while(0)
#define DisSOhigh		do {PORTD |=  (1<<3);} while(0)



/* We are using software spi at the moment
void usart1_init(void)
{
	//crystal oscillator @ 7.3728 MHz
	#define FOSC 7372800
  	// Set the baud rate registers
  	UBRR1H = 0;
  	UBRR1L = 0;
  	// Enable receiver and transmitter
  	UCSR1B = ((1<<RXEN) | (1<<TXEN));
  	// Set synchronous operation, set frame format: 8data bits, clock polarity: data is changed at the falling clock edge and sampled at the rising edge
  	UCSR1C = ((1<<UMSEL1) | (3<<UCSZ0) | (1<<UCPOL));
  	UBRR1H = 0;
  	UBRR1L = 8;

}
*/


void DOGM128_6_usart1_sendByte(unsigned char ch)
{
/*	volatile unsigned char garb;
	DisCSlow;
	//clear TXC1 here by writing a 1 to it? Is that necessary? yes it is!
	//UCSR1A |= (1<<TXC1);
	UDR1 = ch;
	while ((UCSR1A & (1<<TXC1)) == 0);	//wait for complete transmition
	garb = UDR1;
	DisCShigh;
*/


	DisCSlow;
	DisSCLlow;
	if (ch & 0x80) DisSOhigh; else DisSOlow;
	DisSCLhigh;
	DisSCLlow;
	if (ch & 0x40) DisSOhigh; else DisSOlow;
	DisSCLhigh;
	DisSCLlow;
	if (ch & 0x20) DisSOhigh; else DisSOlow;
	DisSCLhigh;
	DisSCLlow;
	if (ch & 0x10) DisSOhigh; else DisSOlow;
	DisSCLhigh;
	DisSCLlow;
	if (ch & 0x08) DisSOhigh; else DisSOlow;
	DisSCLhigh;
	DisSCLlow;
	if (ch & 0x04) DisSOhigh; else DisSOlow;
	DisSCLhigh;
	DisSCLlow;
	if (ch & 0x02) DisSOhigh; else DisSOlow;
	DisSCLhigh;
	DisSCLlow;
	if (ch & 0x01) DisSOhigh; else DisSOlow;
	DisSCLhigh;
	DisSCLhigh;	//short delay (repeating the last command)

	DisCShigh;
}



void DOGM128_6_clear_display(void)
{
  volatile int i;
  volatile char j;

  DisA0high;

  for(j=0; j<8; j++)
  {
	DisA0low;
    DOGM128_6_usart1_sendByte(0xB0 + j);						//Set Page Address
    DOGM128_6_usart1_sendByte(0x10);							//Set Column Address upper nibble to 0
    DOGM128_6_usart1_sendByte(0x00);							//Set Column Address lower nibble to 0
    DisA0high;

    for(i=0; i<128; i++)
    {
      DOGM128_6_usart1_sendByte(0x00);							//Write white bytes to the display
    }
  }
}



void DOGM128_6_display_init(void)
{

  	PORTB &= b11101111;		//LED backlight at Port B4
  	DDRB  |= b00010000;

  	PORTC |= b00000111;		//A0, Chip select and Reset at Port C0..C2
  	DDRC  |= b00000111;

  	DDRD  |= b00101000;		//PD5 is XCK output, PD3 is serial data output
  	PORTD |= b00001000;

	PORTC = b01111000;		//enable pullups for push-buttons
	DDRC &= b10000111;		//pins to push-buttons are inputs

//usart doesn't work, so we use an SPI in software
//	usart1_init();

  	DisA0low;

  	DisRESETlow;
  	_delay_ms(100);
  	DisRESEThigh;
  	_delay_ms(100);

  	DOGM128_6_usart1_sendByte (0x40);
  	DOGM128_6_usart1_sendByte (0xA1);
  	DOGM128_6_usart1_sendByte (0xC0);
  	DOGM128_6_usart1_sendByte (0xA6);
  	DOGM128_6_usart1_sendByte (0xA2);
  	DOGM128_6_usart1_sendByte (0x2F);
	DOGM128_6_usart1_sendByte (0xF8);
	DOGM128_6_usart1_sendByte (0x00);
	DOGM128_6_usart1_sendByte (0x27);
	DOGM128_6_usart1_sendByte (0x81);							//Display contrast
	DOGM128_6_usart1_sendByte (0x10);//was 16
	DOGM128_6_usart1_sendByte (0xAC);
	DOGM128_6_usart1_sendByte (0x00);
	DOGM128_6_usart1_sendByte (0xAF);

	DOGM128_6_clear_display();
}



void DOGM128_6_display_backlight(type_ON_OFF backlight)
{
	if (backlight == ON) DisBLIGHTon;
	else DisBLIGHToff;
}



void DOGM128_6_LCD_print(char text[], unsigned char x, unsigned char y)
{
	int charnumber = 0, pixelcolumn;

	DisA0low;

	DOGM128_6_usart1_sendByte (0xB0 + y);					//Set Page Address
	DOGM128_6_usart1_sendByte (0x10 | (x >> 4));		//Set Column Address upper nibble to 0
	DOGM128_6_usart1_sendByte (x & 0x0F);					//Set Column Address lower nibble to 0

	DisA0high;

  	while (text[charnumber])
  	{
    	for (pixelcolumn = 0; pixelcolumn < 5 ; pixelcolumn++)
	  	{
      		DOGM128_6_usart1_sendByte ( pgm_read_byte(&(font5x7 [(unsigned char)text[charnumber]] [pixelcolumn] )) );	//Write to the display
	  	}
    	DOGM128_6_usart1_sendByte (0x00);														//Write white space pixels between chars to the display
		charnumber++;
  	}
}
