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
*  I²C driver for ATmega128 as a Master
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/



#include <avr/io.h>
#include <util/twi.h>
#include "i2c_driver.h"



// inits to ~300 kHz bus frequency
void i2c_init()
{
	// Set up clock prescaler
	TWSR |= (0 << TWPS1) | (0 << TWPS0);	// Prescaler = 1
	// Set up clock divider
	TWBR = 1;
	// Set up module
	TWCR = (1 << TWEN);	// I2C on, no interrupts
}



void i2c_waitForInterruptFlag()
{
	while (! (TWCR & (1 << TWINT)));
}



void i2c_actionStart()
{
	TWCR |= (1 << TWINT);		// clearing the interrupt flag will start the next action
}



void i2c_start()
{
	TWCR |= (1 << TWSTA);			// Set start condition flag
	i2c_actionStart();				// Send START
	i2c_waitForInterruptFlag();	// Wait until START is sent
	TWCR &= ~(1 << TWSTA);			// Clear start condition flag
}



void i2c_stop()
{
	TWCR |= (1 << TWSTO);			// Set stop condition flag - this will be cleared automatically
	i2c_actionStart();				// Send STOP
	while (TWCR & (1 << TWSTO));	// Wait until STOP is sent
}



// Returns 0 if ACK has been received, else 1
unsigned char i2c_send(unsigned char data)
{
	TWDR = data;
	i2c_actionStart();
	i2c_waitForInterruptFlag();
	if ((TWSR & 0xF8) == TW_MT_DATA_ACK) return 0;			// Data byte ACK
	else if ((TWSR & 0xF8) == TW_MT_SLA_ACK) return 0;	// Address byte ACK
	else return 1;
}



// if ack == 0, no-ACK will be sent
unsigned char i2c_receive(unsigned int ack)
{
	if (ack) TWCR |= (1 << TWEA);	// ACK
	else 		TWCR &= ~(1 << TWEA);	// no ACK
	i2c_actionStart();					// Start receiving
	i2c_waitForInterruptFlag();		// Wait until byte is received
	return TWDR;
}



// This initiates an rx transfer with START + SLA + R
void i2c_controlByte_RX(unsigned char address)
{
	i2c_start();
	i2c_send((address << 1) | 0x01);
}



// This initiates an tx transfer with START + SLA
void i2c_controlByte_TX(unsigned char address)
{
	i2c_start();
	i2c_send(address << 1);
}
