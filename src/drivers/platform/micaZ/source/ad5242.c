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
*  AD5242 digital potentiometer driver
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/

#include "ad5242.h"
#include "i2c_driver.h"



// Initialize the interface
void ad5242_init()
{
	i2c_init();
}


// hwAddress is defined by the Pin settings [AD1 AD0]
// channel is 1 or 2
// value is the resistor divider value
void ad5242_set(unsigned char hwAddress, unsigned char channel, unsigned char value)
{
	unsigned char address = 0b0101100 | (hwAddress & 0b11);
	
	i2c_controlByte_TX(address);
	
	// Instruction byte
	if (channel == 1)
		i2c_send(0x00);		// Channel A (1)
	else
		i2c_send(0x80);		// Channel B (2)
		
	// Resistor divider value
	i2c_send(value);
		
	i2c_stop();
}
