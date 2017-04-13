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

typedef enum {POWER_OFF = 0, POWER_ON = 1}T_Power;


// Do not forget to init everything before using it
void mda100_init();
void mda100_Powersave();
// Power control via int1 (PE5)
// This must be tristate if not in use, otherwise temperature sensor will be incorrect
void mda100_LightSensor_Power(T_Power powerstatus);
// Power control via PW0
// This must be tristate if not in use, otherwise light sensor will be incorrect
void mda100_TemperatureSensor_Power(T_Power powerstatus);
// This is a ratiometric value, so use Vcc as reference
unsigned int mda100_LightSensor_GetCounts();
// This is a ratiometric value, so use Vcc as reference
unsigned int mda100_TemperatureSensor_GetCounts();
float mda100_TemperatureSensor_GetKelvin();
float mda100_TemperatureSensor_GetDegreeCelsius();
