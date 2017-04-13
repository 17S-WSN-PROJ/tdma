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


typedef enum {POWER_OFF = 0, POWER_ON = 1}T_Power;
typedef enum {MIC_RAW = 0, MIC_TONEDETECT}T_MicMode;


// Do not forget to init everything before using it
void mts310cb_init();
void mts310cb_Powersave();
// Power control line PW4
void mts310cb_Accelerometer_Power(T_Power powerstatus);
// Power control line PW5
void mts310cb_Magnetometer_Power(T_Power powerstatus);
// Power control via int1 (PE5)
// This must be tristate if not in use, otherwise temperature sensor will be incorrect
void mts310cb_LightSensor_Power(T_Power powerstatus);
// Power control via PW0
// This must be tristate if not in use, otherwise light sensor will be incorrect
void mts310cb_TemperatureSensor_Power(T_Power powerstatus);
// Power control line PW2
void mts310cb_Sounder_Power(T_Power powerstatus);
// Power control line PW3
void mts310cb_Microphone_Power(T_Power powerstatus);
// Adjust offset voltage at Opamp U6 in 256 steps
void mts310cb_Magnetometer_x_SetOffset(unsigned char value);
// Adjust offset voltage at Opamp U7 in 256 steps
void mts310cb_Magnetometer_y_SetOffset(unsigned char value);
// Adjust gain of Mic preamp in 256 steps
void mts310cb_Microphone_SetGain(unsigned char value);
// This is set with PW6
void mts310cb_Microphone_SetMode(T_MicMode mode);
// This is a ratiometric value, so use Vcc as reference
unsigned int mts310cb_LightSensor_GetCounts();
// This is a ratiometric value, so use Vcc as reference
unsigned int mts310cb_TemperatureSensor_GetCounts();
unsigned int mts310cb_Microphone_GetCounts();
unsigned int mts310cb_Accelerometer_x_GetCounts();
unsigned int mts310cb_Accelerometer_y_GetCounts();
unsigned int mts310cb_Magnetometer_x_GetCounts();
unsigned int mts310cb_Magnetometer_y_GetCounts();
