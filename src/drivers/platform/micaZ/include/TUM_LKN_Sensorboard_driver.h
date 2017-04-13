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
*  Author: Peter Diener
*
***************************************************************************************/

// Call init once after boot
void TUM_LKN_Sensorboard_init();

// Digital IOs
void TUM_LKN_Sensorboard_writeIOs( uint8_t data );
void TUM_LKN_Sensorboard_writeIO( uint8_t io, bool set );

// Beeper
void TUM_LKN_Sensorboard_setBeeper( bool set );

// Stepper engine
void TUM_LKN_Sensorboard_StepperIdle();    // Call idle if stepper does not turn, otherwise driver circuit will overheat
void TUM_LKN_Sensorboard_stepCCW();
void TUM_LKN_Sensorboard_stepCW();
void TUM_LKN_Sensorboard_halfStepCCW();
void TUM_LKN_Sensorboard_halfStepCW();
unsigned char TUM_LKN_Sensorboard_getCurrentStep();

// ADC/DAC
// tbd. port from ecent based system (Tiny-OS) to nano-rk
//void TUM_LKN_Sensorboard_readPhotoVoltage();
//void TUM_LKN_Sensorboard_readChannel1();
//void TUM_LKN_Sensorboard_readChannel2();
//void TUM_LKN_Sensorboard_readChannel3();
//void TUM_LKN_Sensorboard_readChannel( uint8_t ch );
void TUM_LKN_Sensorboard_writeValue( double voltage );


// LEDs and Laser
// blinkPeriod is in seconds, dutyCycle is in [0; 1]
void TUM_LKN_Sensorboard_setControl0( double blinkPeriod, double dutyCycle );
void TUM_LKN_Sensorboard_setControl1( double blinkPeriod, double dutyCycle );
void TUM_LKN_Sensorboard_setBrightness0( double dutyCycle );
void TUM_LKN_Sensorboard_setBrightness1( double dutyCycle );

void TUM_LKN_Sensorboard_laserOn();
void TUM_LKN_Sensorboard_laserOff();
void TUM_LKN_Sensorboard_laserToggle();
void TUM_LKN_Sensorboard_laserBlink( uint8_t ctl );

void TUM_LKN_Sensorboard_redOn();
void TUM_LKN_Sensorboard_redOff();
void TUM_LKN_Sensorboard_redToggle();
void TUM_LKN_Sensorboard_redBlink( uint8_t ctl );

void TUM_LKN_Sensorboard_yellowOn();
void TUM_LKN_Sensorboard_yellowOff();
void TUM_LKN_Sensorboard_yellowToggle();
void TUM_LKN_Sensorboard_yellowBlink( uint8_t ctl );

void TUM_LKN_Sensorboard_greenOn();
void TUM_LKN_Sensorboard_greenOff();
void TUM_LKN_Sensorboard_greenToggle();
void TUM_LKN_Sensorboard_greenBlink( uint8_t ctl );

// internal function
void i2cAction(void);
