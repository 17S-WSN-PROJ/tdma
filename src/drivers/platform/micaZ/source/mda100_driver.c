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
*  mda100 sensor board driver
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/


#include "mda100_driver.h"
#include "adc_driver.h"
#include <avr/io.h>
#include <math.h>


volatile unsigned char mda100_initDone = 0;

#define CheckIfInitDone() do{ if (mda100_initDone == 0) mda100_init(); }while(0)


// Do not forget to init everything before using it
void mda100_init()
{
	adc_init();
	mda100_initDone = 1;
}



void mda100_Powersave()
{
	CheckIfInitDone();
	adc_Powersave();
}



// Power control via int1 (PE5)
// This must be tristate if not in use, otherwise temperature sensor will be incorrect
void mda100_LightSensor_Power(T_Power powerstatus)
{
	CheckIfInitDone();
	// Switch off temperature sensor power first; Only one of these may be active at a time
	PORTC &= ~(1 << 0);
    DDRC  &= ~(1 << 0);

	if (powerstatus == POWER_ON)
	{
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
void mda100_TemperatureSensor_Power(T_Power powerstatus)
{
	CheckIfInitDone();
	// Switch off light sensor power first; Only one of these may be active at a time
    PORTE &= ~(1 << 5);
    DDRE  &= ~(1 << 5);

	if (powerstatus == POWER_ON)
	{
		PORTC |= (1 << 0);
		DDRC  |= (1 << 0);
	}
	else
	{
		PORTC &= ~(1 << 0);
		DDRC  &= ~(1 << 0);
	}
}



// This is a ratiometric value, so use Vcc as reference
unsigned int mda100_LightSensor_GetCounts()
{
	CheckIfInitDone();
	mda100_LightSensor_Power(POWER_ON);
	return adc_GetChannel(1);
}



// This is a ratiometric value, so use Vcc as reference
unsigned int mda100_TemperatureSensor_GetCounts()
{
	CheckIfInitDone();
	mda100_TemperatureSensor_Power(POWER_ON);
	return adc_GetChannel(1);
}



float mda100_TemperatureSensor_GetKelvin()
{
	float counts = mda100_TemperatureSensor_GetCounts();
	float lnRthr = log( 10e3 * (1023.0 - counts) / counts);
	float lnRthr3 = pow(lnRthr, 3);	// lnRthr³
	
	float a = 0.001010024;
	float b = 0.000242127;
	float c = 0.000000146;
	
	return ( 1 / ( a + b * lnRthr + c * lnRthr3) );
}



float mda100_TemperatureSensor_GetDegreeCelsius()
{
	return (mda100_TemperatureSensor_GetKelvin() - 273.15);
}
