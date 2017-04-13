/******************************************************************************
*
*  Filename: twi_base_calls.h
*  Author: Andrew Jameson
*  Last Updated: 2011-02-23
*

*
*******************************************************************************/
#ifndef _I2C_DRIVER_H
#define	_I2C_DRIVER_H


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
#define BAT	0
#define LIGHT	1
#define ACC_X	2
#define AUDIO	3
#define TEMP	4
#define ACC_Y	5
#define ACC_Z	6
#define AUDIO_P2P	7
#define ACC_P2P		8
#define PRESS           9


#define PWR_CTRL_MASK 0x80


void delay();
uint8_t dev_manager_ff_sensors(uint8_t state,uint8_t opt,uint8_t * buffer,uint8_t size);

/* These six functions handle what should happen for each of the 6 things that the driver
 *  may be required to do
 */
uint8_t init(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
uint8_t read(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
uint8_t write(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
uint8_t open(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
uint8_t close(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
uint8_t get_status(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);
uint8_t set_status(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);

#endif	/* _I2C_DRIVER_H */

