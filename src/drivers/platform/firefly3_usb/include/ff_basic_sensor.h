/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
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
*  Contributing Authors (specific to this file):
*  Zane Starr
*  Anthony Rowe
*  Andrew Jameson
*******************************************************************************/


#include <stdio.h>

// GET/SET Status Options
#define SENSOR_SELECT  1

// SET_SENSOR options
#define TEMP		0
#define LIGHT		1
#define HUM		2
#define AUDIO		3
#define ACC_X		4
#define ACC_Y		5
#define ACC_Z		6
#define ANALOG_MOTION	6
#define BAT 		7	
#define ACC_P2P		8
#define PRESS           9
#define TEMP2		10
#define HUMIDITY	11	
#define AUDIO_P2P	12	
#define MOTION	 	13	


#define PWR_CTRL_PIN NRK_PORTE_3	
#define PWR_CTRL_MASK 0x08 


void delay();
int8_t dev_manager_ff3_sensors(uint8_t state,uint8_t opt,uint8_t * buffer,uint8_t size);
uint16_t get_adc_val();
uint16_t read_voltage_status();

/* These six functions handle what should happen for each of the 6 things that the driver
 *  may be required to do
 */
int8_t init(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
int8_t read(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
int8_t open(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
int8_t close(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
int8_t get_status(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
int8_t set_status(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);

/**
 * Returns 10X the temperature in Celcius. For example, if the temperature is
 * 20.7 degrees Celcius, will return 207. This is the method to call to get the
 * temperature and acts as a wrapper around the other methods.
 *
 * @returns the temperature
 *
 */
int32_t calc_temp();

/**
 * Returns the current pressure in Pascals. Note that standard pressure (1 ATM)
 * is 101,325 Pascals.
 *
 * @param oss - the oversampling mode. Valid values are 0 to 3. Currently the code
 *              appears to be working with all OSS values but the safest is 0.
 *
 * @returns the pressure in pascals
 *
 */
int32_t calc_press(uint8_t oss);


/**
 * Reads the uncompensated pressure value from the Bosch sensor. This method
 * should not be called directly. Instead use the calc_press() is a wrapper and
 * should be used.
 *
 * @param oss - the oversampling mode. Valid values are 0 to 3. Currently the code
 *              appears to be working with all OSS values but the safest is 0.
 *
 * @returns void
 *
 */
void read_uncomp_press(uint8_t oss);

/**
 * Reads the uncompensated temperature value from the Bosch sensor. This method
 * should not be called directly. Instead the calc_temp() is a wrapper and should
 * be used.
 *
 * @returns void
 *
 */
void read_uncomp_temp();

/**
 * Determines the actual compensated temperature value. This method should not be
 * called directly. Instead the calc_temp() is a wrapper and should be used.
 *
 * @returns - the correct temperature value
 *
 */
int32_t calc_true_temp();


/**
 * Determines the actual compensated pressure value. This method should not be called
 * directly. Instead the calc_press() is a wrapper and should be used.
 *
 * @param oss - the oversampling mode. Valid values are 0 to 3. Currently the code
 *              appears to be working with all OSS values but the safest is 0.
 *
 * @reuurns - the correct pressure value
 */
int32_t calc_true_press(uint8_t oss);

/**
 * Reads in the neccessary EEPROM values for the Bosch temperature/pressure
 * sensor. The values are used for compensating the raw values returned from the
 * sensor.
 *
 * @returns - void
 *
 */
void get_eeprom_values(void);

// Functions for initializing and updating sensor values
void init_adc();
