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
*  ATmega128 analog to digital converter driver
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/



#define F_CPU 7372800

#include "adc_driver.h"
#include <avr/io.h>
#include <util/delay.h>



void adc_init()
{
  volatile unsigned int dummy;

  ADMUX = (0 << REFS1) | (1 << REFS0);      						// Vcc is reference
  ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);       // Prescaler = 64
  ADCSRA |= (1 << ADEN);                								// ADC on

	// One dummy read after init
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC));
  dummy = ADCW;
}



// If ADC should be used after a powersave period, call init again before starting a conversion
void adc_Powersave()
{
	ADCSRA &= ~(1 << ADEN);
}



unsigned int adc_GetChannel(unsigned char ch)
{
  ADMUX = (ADMUX & 0b11100000) | (ch & 0b00011111);	// Select channel
  ADCSRA |= (1 << ADSC);										// Start a single conversion
  while (ADCSRA & (1 << ADSC));  							// wait until conversion result is available
  return ADCW;
}


float adc_GetBatteryVoltage()
{
// adc channel 30 is connected to the internal 1.23 V reference
	unsigned int adValue;

	adValue = adc_GetChannel(30);

	return (1.23 * 1024.0 / (float)adValue);
}
