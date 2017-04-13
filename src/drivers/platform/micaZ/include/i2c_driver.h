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



/// @brief Inits to ~300 kHz bus frequency
// This must be called before calling any other function of this driver
void i2c_init();

/// @brief Internal function, do not call this from userspace
// This function waits until a current i2c hardware action has finished
void i2c_waitForInterruptFlag();

/// @brief Internal function, do not call this from userspace
// This function starts an hardware action that has been configured previously
void i2c_actionStart();

/// @brief Internal function, do not call this from userspace
//	This function sends a Startcondition on the bus
// As a user call i2c_controlByte_RX or i2c_controlByte_TX instead which will include the transfer of the controlbyte
void i2c_start();

/// @brief Send a Stopcondition on the bus
// Call this after your data transfer has finished
void i2c_stop();

/// @brief Send a byte to the slave
/// @param data: the data payload
/// @return 0 if Acknowledge has been received on the bus, else 1
unsigned char i2c_send(unsigned char data);

/// @brief Receive a byte from the slave
/// @param ack: if ack == 0, no-ACK will be sent, else ACK will be sent
unsigned char i2c_receive(unsigned int ack);

/// @brief This initiates an rx transfer with Startcondition + SlaveAddressByte + ReadBit set
void i2c_controlByte_RX(unsigned char address);

/// @brief This initiates an tx transfer with Startcondition + SlaveAddressByte + ReadBit cleared
void i2c_controlByte_TX(unsigned char address);
