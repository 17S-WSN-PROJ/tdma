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
*  Serial ID Driver for Maxim DS2401.h
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/


#include "DS2401.h"
#include <avr/io.h>
#include <util/delay.h>



// DataPin_DriveLow forces a low state to the data pin
#define DataPin_DriveLow   do {OUTPUT &= ~(1 << PIN); DIRECTION |= (1 << PIN);} while(0)
// DataPin_PullUp switches the data pin to input and enables the pullup resistor
#define DataPin_PullUp     do {DIRECTION &= ~(1 << PIN); OUTPUT |= (1 << PIN);} while(0)
// DataPin_State returns 1 if the data pin is high, else 0
#define DataPin_State      ((INPUT & (1 << PIN)) >> PIN)

// The Maxim 1Wire interface is designed for a slot time of 100µs

// Applies a reset pulse and checks for the presence pulse of a slave device
// Returns 0 if slave is present, else -1
inline signed char Maxim_1Wire_ResetPulse(void)
{
    unsigned char delaytime = 0;
    _delay_us(500);
    DataPin_DriveLow;
    _delay_us(500);   // datasheet value: 480 µs
    DataPin_PullUp;

    // Wait for data line to go high
    while (DataPin_State == 0);

    // Wait for presence pulse
    while (DataPin_State == 1)
    {
        _delay_us(20);
        delaytime ++;
        if (delaytime > 30) return -1;  // Timeout 600 µs: no presence pulse detected
    }

    // Presence pulse detected, wait until bus is free
    _delay_us(500);   // Datasheet: Trsth

    return 0;
}


// Write will take a 100 µs time slot and write one or zero
// Wrtie One will pull low for 10 µs
// Write Zero will pull down for 80 µs
inline void Maxim_1Wire_WriteBit(unsigned char databit)
{
   if (databit == 0) // Write Zero
   {
      DataPin_DriveLow;
      _delay_us(80);    //80
      DataPin_PullUp;
      _delay_us(20);    //20
   }
   else              // Write One
   {
      DataPin_DriveLow;
      _delay_us(10);    //10
      DataPin_PullUp;
      _delay_us(90);    //90
   }
}



// Reads a bit
inline unsigned char Maxim_1Wire_ReadBit(void)
{
   unsigned char bit;

   DataPin_DriveLow;
   _delay_us(1);    //1
   DataPin_PullUp;
   _delay_us(9);   //9
   bit = DataPin_State;
   _delay_us(80);   //80

   return bit;
}


inline void Maxim_1Wire_WriteByte(unsigned char data)
{
   // Data order is lsb first
   unsigned char currentBit;

   for (currentBit = 0; currentBit < 8; currentBit++)
   {
      Maxim_1Wire_WriteBit(data & (1 << currentBit));
   }
}


inline unsigned char Maxim_1Wire_ReadByte(void)
{
   // Data order is lsb first
   unsigned char data = 0;
   unsigned char currentBit;

   for (currentBit = 0; currentBit < 8; currentBit++)
   {
      data |= (Maxim_1Wire_ReadBit() << currentBit);
   }

   return data;
}



inline void Maxim_1Wire_CommandReadRom(void)
{
   Maxim_1Wire_WriteByte(0x0f);
}



inline void DS2401_init(void)
{
    SFIOR &= ~(1 << PUD);
    DataPin_PullUp;
    _delay_us(100);   // One time slot for powerup
}


// Reads the 32 bit world wide unique serial number
// valid is set to 0 in case of success, -2 in case of family code mismatch, -1 in case of CRC error (tdb)
inline uint32_t DS2401_getSerialNumber(int8_t *valid) {
    uint32_t serialNumber = 0;
    uint8_t byte;
    signed char returnVal;
    returnVal = Maxim_1Wire_ResetPulse();
    if (returnVal != 0) return returnVal;
    Maxim_1Wire_CommandReadRom();
    if (Maxim_1Wire_ReadByte() != 0x01) *valid = -2;      //  Read device code

    *(uint8_t*)&serialNumber = Maxim_1Wire_ReadByte();          //  Read ID Byte 0 - last significant

    *((uint8_t*)&serialNumber + 1) = Maxim_1Wire_ReadByte();    //  Read ID Byte 1

    *((uint8_t*)&serialNumber + 2) = Maxim_1Wire_ReadByte();    //  Read ID Byte 2

    *((uint8_t*)&serialNumber + 3) = Maxim_1Wire_ReadByte();    //  Read ID Byte 3

    Maxim_1Wire_ReadByte();                                     //  Read ID Byte 4 = 0
    Maxim_1Wire_ReadByte();                                     //  Read ID Byte 5 = 0 - most significant
    Maxim_1Wire_ReadByte();                                     //  Read CRC, tbd.
    *valid = 0;
    return serialNumber;
}

