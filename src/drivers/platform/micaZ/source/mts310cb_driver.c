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
*  mts310cb sensor board driver
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/


#include "mts310cb_driver.h"
#include "adc_driver.h"
#include "ad5242.h"
#include <avr/io.h>


volatile unsigned char mts310cb_initDone = 0;

#define CheckIfInitDone() do{ if (mts310cb_initDone == 0) mts310cb_init(); }while(0)


// Do not forget to init everything before using it
void mts310cb_init()
{
	adc_init();
	ad5242_init();
	mts310cb_initDone = 1;
}



void mts310cb_Powersave()
{
	CheckIfInitDone();
	adc_Powersave();
}



// Power control line PW4
void mts310cb_Accelerometer_Power(T_Power powerstatus)
{
	CheckIfInitDone();
	if (powerstatus == POWER_ON)	PORTC |=  (1 << 4);
	else									PORTC &= ~(1 << 4);
}



// Power control line PW5
void mts310cb_Magnetometer_Power(T_Power powerstatus)
{
	CheckIfInitDone();
	if (powerstatus == POWER_ON)	PORTC |=  (1 << 5);
	else									PORTC &= ~(1 << 5);
}



// Power control via int1 (PE5)
// This must be tristate if not in use, otherwise temperature sensor will be incorrect
void mts310cb_LightSensor_Power(T_Power powerstatus)
{
	CheckIfInitDone();
	if (powerstatus == POWER_ON)
	{
		mts310cb_TemperatureSensor_Power(POWER_OFF);		// Only one of these may be active at a time
		PORTE |= (1 << 5);
		DDRE  |= (1 << 5);
	}
	else
	{
		PORTE &= ~(1 << 5);
		DDRE  &= ~(1 << 5);
	}
}



// Power control via PW0
// This must be tristate if not in use, otherwise light sensor will be incorrect
void mts310cb_TemperatureSensor_Power(T_Power powerstatus)
{
	CheckIfInitDone();
	if (powerstatus == POWER_ON)
	{
		mts310cb_LightSensor_Power(POWER_OFF);				// Only one of these may be active at a time
		PORTC |= (1 << 0);
		DDRC  |= (1 << 0);
	}
	else
	{
		PORTC &= ~(1 << 0);
		DDRC  &= ~(1 << 0);
	}
}



// Power control line PW2
void mts310cb_Sounder_Power(T_Power powerstatus)
{
	CheckIfInitDone();
	if (powerstatus == POWER_ON)	PORTC |=  (1 << 2);
	else									PORTC &= ~(1 << 2);
}



// Power control line PW3
void mts310cb_Microphone_Power(T_Power powerstatus)
{
	CheckIfInitDone();
	if (powerstatus == POWER_ON)	PORTC |=  (1 << 3);
	else									PORTC &= ~(1 << 3);
}



// Adjust offset voltage at Opamp U6 in 256 steps
void mts310cb_Magnetometer_x_SetOffset(unsigned char value)
{
	CheckIfInitDone();
	#define ad5242_MagnetometerAddress 0
	ad5242_set(ad5242_MagnetometerAddress, 1, value);	// Channel 1
}



// Adjust offset voltage at Opamp U7 in 256 steps
void mts310cb_Magnetometer_y_SetOffset(unsigned char value)
{
	CheckIfInitDone();
	ad5242_set(ad5242_MagnetometerAddress, 2, value);	// Channel 2
}



// Adjust gain of Mic preamp in 256 steps
void mts310cb_Microphone_SetGain(unsigned char value)
{
	CheckIfInitDone();
	#define ad5242_MicrophoneAddress 1
	ad5242_set(ad5242_MicrophoneAddress, 1, value);		// Channel 1
}



// This is set with PW6
void mts310cb_Microphone_SetMode(T_MicMode mode)
{
	CheckIfInitDone();
	if (mode == MIC_RAW)                PORTC |=  (1 << 6); // tbd: is this the right logic or is it inverted?
	else if (mode == MIC_TONEDETECT)    PORTC &= ~(1 << 6);
}


// This is a ratiometric value, so use Vcc as reference
unsigned int mts310cb_LightSensor_GetCounts()
{
	CheckIfInitDone();
	mts310cb_LightSensor_Power(POWER_ON);
	return adc_GetChannel(1);
}



// This is a ratiometric value, so use Vcc as reference
unsigned int mts310cb_TemperatureSensor_GetCounts()
{
	CheckIfInitDone();
	mts310cb_TemperatureSensor_Power(POWER_ON);
	return adc_GetChannel(1);
}



unsigned int mts310cb_Microphone_GetCounts()
{
	CheckIfInitDone();
	return adc_GetChannel(2);
}



unsigned int mts310cb_Accelerometer_x_GetCounts()
{
	CheckIfInitDone();
	return adc_GetChannel(3);
}



unsigned int mts310cb_Accelerometer_y_GetCounts()
{
	CheckIfInitDone();
	return adc_GetChannel(4);
}



unsigned int mts310cb_Magnetometer_x_GetCounts()
{
	CheckIfInitDone();
	return adc_GetChannel(5);
}



unsigned int mts310cb_Magnetometer_y_GetCounts()
{
	CheckIfInitDone();
	return adc_GetChannel(6);
}
