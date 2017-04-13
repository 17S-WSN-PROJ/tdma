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
*  Contributing Authors (specific to this file):
*  Peter Diener and others
*
***************************************************************************************/
//#include <I2C.h>
#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <stdlib.h>
#include <hal.h>

#include "TUM_LKN_Sensorboard_driver.h"


// Anmerkung: Diesen Treiber sollte man mal komplett neu schreiben.



// Hardware addresses
enum
{
	PCF8574 = 0x40,   		// digital IO PCF8574/external addr switch = 0
	PCF8591 = 0x90,       	// ADC PCF8591/external addr switch = 0
	PCA9533 = 0xC6   			// pwm led controller PCA9533
};



// Register entries defining the state of the AD/DA converter
enum
{
	REG_ADC_CHAN0					= 0x00,			// channel 0
	REG_ADC_CHAN1					= 0x01,			// channel 1
	REG_ADC_CHAN2					= 0x02,			// channel 2
	REG_ADC_CHAN3					= 0x03,			// channel 3
	REG_ADC_AUTOINCREMENT		= 0x04,			// auto-increment, sequentially read/write multiple registers
	REG_ADC_FOURSE					= 0x00,			// four single-ended inputs
	REG_ADC_3DIFF					= 0x10,			// three differential inputs
	REG_ADC_SEDIFFMIX				= 0x20,			// single-ended and differential mixed
	REG_ADC_2DIFF					= 0x30,			// two differential inputs
	REG_ADC_AOUTENABLE			= 0x40			// analogue output enable flag
};



// Control registers of the PWM LED controller
enum
{
	REG_LED_INPUT 					= 0x00,   		// r   Input register (writes are acknowledged but without effect)
	REG_LED_PSC0  					= 0x01,   		// r/w Frequency prescaler 0
	REG_LED_PWM0  					= 0x02,   		// r/w PWM register 0
	REG_LED_PSC1  					= 0x03,   		// r/w Frequency prescaler 1
	REG_LED_PWM1  					= 0x04,   		// r/w PWM register 1
	REG_LED_LS0   					= 0x05,    		// r/w LED selector
	REG_LED_AUTOINCREMENT		= 0x10
};



// Register entries defining the state (on/off) of the LEDs
enum
{
	REG_LED_OFF        			= 0x00,
	REG_LED_ON         			= 0x01,
	REG_LED_BLINK_PWM0 			= 0x02,
	REG_LED_BLINK_PWM1 			= 0x03,
};




// Power timeout for the stepper motor. Given in milliseconds. Power is interrupted
// after the stepper is idle for the given time.
uint16_t steppertimeout = 100;


// Bit pattern for controlling the stepper motor
uint8_t steps[ 8 ] = { 0x50, 0x10, 0x90, 0x80, 0xA0, 0x20, 0x60, 0x40 };
uint8_t m_currentstep = 0;


// This will be sent on the I2C bus
uint8_t i2cData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t i2cDataLen;


enum { i2cWrite, i2cRead } i2cDataDirection;

uint16_t i2cDestination;

// Bit pattern of last write access to the digital IO ports
uint8_t m_lastxs = 0x00;

// Holds the values written to or read from the adc/dac
double m_vbatt = 3.;
uint8_t m_channel;

// These variables keep track of the LED states
uint8_t m_ledstate = 0x00;
uint8_t m_prescaler[ 2 ] = { 0x00, 0x00 };
uint8_t m_pwm[ 2 ] = { 0xFF, 0xFF };



// ---- LOW LEVEL FUNCTIONS



//this is ok
void writeIOs( uint8_t data )
{
	i2cData[ 0 ] = m_lastxs = data;
	i2cDataLen = 1;
	i2cDataDirection = i2cWrite;
	i2cDestination = PCF8574;

	i2cAction();
}



void readIOs()
{
	i2cDataDirection = i2cRead;
	i2cDataLen = 1;
	i2cDestination = PCF8574;

	i2cAction();
}



void writeDAC( uint8_t value )
{
	i2cDataDirection = i2cWrite;		// write to the i2c bus
	i2cDataLen = 2;
	i2cData[ 0 ] = REG_ADC_FOURSE | REG_ADC_AOUTENABLE;
	i2cData[ 1 ] = value;
	i2cDestination = PCF8591;

	i2cAction();
}



void readADC( uint8_t channel )
{
	//set the correct channel first
	channel &= 0x03;
	m_channel = channel;
	i2cDataDirection = i2cWrite;		// write to the i2c bus
	i2cDataLen = 1;
	i2cData[ 0 ] = REG_ADC_FOURSE | REG_ADC_AOUTENABLE | channel;
	i2cDestination = PCF8591;

	i2cAction();

	//and now read the analog input
	i2cDataDirection = i2cRead;	// read from the i2c bus
	i2cDataLen = 2;
	i2cDestination = PCF8591;

	i2cAction();
}



// Apply current values of m_led0... m_led3
void setLeds()
{
	// This is setting both prescalers and both PWMs
	i2cDataLen = 6;
	i2cData[0] = REG_LED_PSC0 | REG_LED_AUTOINCREMENT;
	i2cData[1] = m_prescaler[ 0 ];
	i2cData[2] = m_pwm[ 0 ];
	i2cData[3] = m_prescaler[ 1 ];
	i2cData[4] = m_pwm[ 1 ];
	i2cData[5] = m_ledstate;
	i2cDataDirection = i2cWrite;
	i2cDestination = PCA9533;

	i2cAction();
}



void setLedMode( uint8_t led, uint8_t mode )
{
	led = (led & 0x03) << 1;						// every LED has two bits and there are 4 LEDs
	m_ledstate &= ~(0x03 << led);					// reset affected bits
	m_ledstate |= (mode << led) & (0x03 << led);		// and write the new value
}



uint8_t getLedMode( uint8_t led )
{
	led = (led & 0x03) << 1;						// ...
	return( (m_ledstate & (0x03 << led)) >> led );	// return the bits corresponding to the LED
}


// Conversion functions

double S2V( uint8_t s ) { return (m_vbatt * ((double)s)) / 256.; }
uint8_t V2S( double v ) { return (uint8_t) ( (v * 256.) / m_vbatt ); }

uint8_t setBits( uint8_t data, uint8_t mask, uint8_t bits ) { return( ( data & mask ) | bits ); }



// For a given time period (s), compute the value for the prescaler register
uint8_t prescalerSec2Hex(double t)
{
	if (t > 255.0 * 152.0)   // 38760 sec = 10h46'00''
		return( 0xFF );
	else if (t >= 1./152.0)
		return( (uint8_t) (t * 152 - 1) );
	else
		return( 0x00 );
}



// For a given fraction ([0; 1]), compute the value for the PWM register
uint8_t pwmFrac2Hex(double fraction)
{
	if (fraction >= 1.0)
		return( 0xFF );
	else if (fraction >= 1.0/256.0)
		return( (uint8_t) (fraction * 256) );
	else
		return( 0x00 );
}



// Convenience functions
void TUM_LKN_Sensorboard_init() 									{ writeIOs( 0xFF ); writeDAC( 0x00 ); }
//TUM_LKN_Sensorboard_readIOs() 								{ readIOs(); }
void TUM_LKN_Sensorboard_writeIOs( uint8_t data ) 			{ writeIOs( data ); }
void TUM_LKN_Sensorboard_writeIO( uint8_t io, bool set ) { writeIOs( setBits( m_lastxs, ~(0x01 << io), (set == TRUE) ? (0x01 << io) : 0x00 ) ); }
void TUM_LKN_Sensorboard_setBeeper( bool set ) 				{ writeIOs( setBits( m_lastxs, 0xF7, (set == FALSE) ? 0x08 : 0x00 ) ); }


void TUM_LKN_Sensorboard_StepperIdle()
{
    writeIOs( setBits( m_lastxs, 0x0F, 0xF0 ) );
}

void TUM_LKN_Sensorboard_stepCCW()
{
	writeIOs( setBits( m_lastxs, 0x0F, steps[ m_currentstep ] ) );
	m_currentstep = (m_currentstep + 2) & 0x06;
}



void TUM_LKN_Sensorboard_stepCW()
{
	writeIOs( setBits( m_lastxs, 0x0F, steps[ m_currentstep ] ) );
	m_currentstep = (m_currentstep + 6) & 0x06;
}



void TUM_LKN_Sensorboard_halfStepCCW()
{
	writeIOs( setBits( m_lastxs, 0x0F, steps[ m_currentstep ] ) );
	m_currentstep = (m_currentstep + 1) & 0x07;
}



void TUM_LKN_Sensorboard_halfStepCW()
{
	writeIOs( setBits( m_lastxs, 0x0F, steps[ m_currentstep ] ) );
	m_currentstep = (m_currentstep + 7) & 0x07;
}



unsigned char TUM_LKN_Sensorboard_getCurrentStep() { return m_currentstep; }

void TUM_LKN_Sensorboard_readPhotoVoltage() { readADC( 0 ); }
void TUM_LKN_Sensorboard_readChannel1() { readADC( 1 ); }
void TUM_LKN_Sensorboard_readChannel2() { readADC( 2 ); }
void TUM_LKN_Sensorboard_readChannel3() { readADC( 3 ); }
void TUM_LKN_Sensorboard_readChannel( uint8_t ch ) { readADC( ch ); }
void TUM_LKN_Sensorboard_writeValue( double voltage ) { writeDAC( V2S( voltage ) ); }



// We can define two pairs of prescaler/PWM.
// blinkPeriod is in seconds, dutyCycle is in [0; 1]
void TUM_LKN_Sensorboard_setControl0( double blinkPeriod, double dutyCycle )
{
	m_prescaler[ 0 ] = prescalerSec2Hex( blinkPeriod );
	m_pwm[ 0 ] = pwmFrac2Hex( dutyCycle );
}



void TUM_LKN_Sensorboard_setControl1( double blinkPeriod, double dutyCycle )
{
	m_prescaler[ 1 ] = prescalerSec2Hex( blinkPeriod );
	m_pwm[ 1 ] = pwmFrac2Hex( dutyCycle );
}



void TUM_LKN_Sensorboard_setBrightness0( double dutyCycle )
{
	m_prescaler[ 0 ] = 0x00;   // Highest frequency possible
	m_pwm[ 0 ] = pwmFrac2Hex( dutyCycle );
}



void TUM_LKN_Sensorboard_setBrightness1( double dutyCycle )
{
	m_prescaler[ 1 ] = 0x00;   // Highest frequency possible
	m_pwm[ 1 ] = pwmFrac2Hex( dutyCycle );
}



void TUM_LKN_Sensorboard_laserOn() 							{ setLedMode( 3, REG_LED_ON ); setLeds(); }
void TUM_LKN_Sensorboard_laserOff() 						{ setLedMode( 3, REG_LED_OFF ); setLeds(); }
void TUM_LKN_Sensorboard_laserToggle() 					{ setLedMode( 3, getLedMode( 3 ) ^ 0x01 ); setLeds(); }
void TUM_LKN_Sensorboard_laserBlink( uint8_t ctl ) 	{ setLedMode( 3, 0x02 | (ctl & 0x01) ); setLeds(); }

void TUM_LKN_Sensorboard_redOn() 							{ setLedMode( 0, REG_LED_ON ); setLeds(); }
void TUM_LKN_Sensorboard_redOff() 							{ setLedMode( 0, REG_LED_OFF ); setLeds(); }
void TUM_LKN_Sensorboard_redToggle() 						{ setLedMode( 0, getLedMode( 0 ) ^ 0x01 ); setLeds(); }
void TUM_LKN_Sensorboard_redBlink( uint8_t ctl ) 		{ setLedMode( 0, 0x02 | (ctl & 0x01) ); setLeds(); }

void TUM_LKN_Sensorboard_yellowOn() 						{ setLedMode( 1, REG_LED_ON ); setLeds(); }
void TUM_LKN_Sensorboard_yellowOff() 						{ setLedMode( 1, REG_LED_OFF ); setLeds(); }
void TUM_LKN_Sensorboard_yellowToggle() 					{ setLedMode( 1, getLedMode( 1 ) ^ 0x01 ); setLeds(); }
void TUM_LKN_Sensorboard_yellowBlink( uint8_t ctl )	{ setLedMode( 1, 0x02 | (ctl & 0x01) ); setLeds(); }

void TUM_LKN_Sensorboard_greenOn() 							{ setLedMode( 2, REG_LED_ON ); setLeds(); }
void TUM_LKN_Sensorboard_greenOff() 						{ setLedMode( 2, REG_LED_OFF ); setLeds(); }
void TUM_LKN_Sensorboard_greenToggle() 					{ setLedMode( 2, getLedMode( 2 ) ^ 0x01 ); setLeds(); }
void TUM_LKN_Sensorboard_greenBlink( uint8_t ctl ) 	{ setLedMode( 2, 0x02 | (ctl & 0x01) ); setLeds(); }



void TWI_write(unsigned char address, unsigned char length, unsigned char* payload)
{
	unsigned char datapointer;

	PORTD |= 0x03;		//enable Portpin pullups

	TWSR = 0;                             // set prescaler == 0
	//TWBR = (F_CPU / 50000UL - 16) / 2;   // set I2C baud rate
	TWBR = 62;
	TWAR = 0;
	TWCR = 0;

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
//		if ((TWSR & 0xF8) != START) {} //ERROR
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);								//clear TWINT to start transmission of address
	while (!(TWCR & (1<<TWINT)));									//wait until transmission is complete
//	  if ((TWSR & 0xF8) != MT_SLA_ACK) {}  //ERROR
	for(datapointer = 0; datapointer < length; datapointer++)
	{
		TWDR = payload[datapointer];
		TWCR = (1<<TWINT) | (1<<TWEN);							//clear TWINT to start transmission of data
		while (!(TWCR & (1<<TWINT)));								//wait until transmission is complete
//	  if ((TWSR & 0xF8) != MT_DATA_ACK) {} //ERROR
	}
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);				//Stop condition
}



void TWI_read(unsigned char address, unsigned char length, unsigned char* payload)
{
	unsigned char datapointer;

	address |= 0x01;	//Set read mode on i2c

	PORTD |= 0x03;		//enable Portpin pullups

	TWSR = 0;                             				// set prescaler == 0
	//TWBR = (F_CPU / 50000UL - 16) / 2;   				// set I2C baud rate
	TWBR = 62;
	TWAR = 0;
	TWCR = 0;

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
//		if ((TWSR & 0xF8) != START) {} //ERROR
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);								//clear TWINT to start transmission of address
	while (!(TWCR & (1<<TWINT)));									//wait until transmission is complete
//	  if ((TWSR & 0xF8) != MT_SLA_ACK) {}  //ERROR
	for(datapointer = 0; datapointer < length; datapointer++)
	{
		//is this the correct place for reading???
		if (datapointer < (length - 1))	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);			//clear TWINT to start transmission of data, send acknowledge
		else 														TWCR = (1<<TWINT)|(1<<TWEN);								//clear TWINT to start transmission of data, send not acknowledge (last byte)
		while (!(TWCR & (1<<TWINT)));								//wait until transmission is complete
		payload[datapointer] = TWDR;
		//or should be read here???
	}
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);				//Stop condition
}



void i2cAction()
{
	if( i2cDataDirection == i2cWrite )
	{
		TWI_write(i2cDestination, i2cDataLen, i2cData);
	}
	else
	{
		TWI_read(i2cDestination, i2cDataLen, i2cData);
	}
}
