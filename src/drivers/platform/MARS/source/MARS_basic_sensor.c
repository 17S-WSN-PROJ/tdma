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
 *  Created by frank mokaya on 3/22/12.
 *  Copyright 2012 Carnegie Mellon Univ. All rights reserved.
 *******************************************************************************/


#include <stdio.h>
#include <stdint.h>
#include <MARS_basic_sensor.h>
#include <math.h>


uint8_t MARS_sensor_val_to_uint8_buf(int16_t value_from_sensor, uint8_t *buffer, uint8_t sensorID) {
	
	uint8_t count = 0;
	
	//Break up the read sensor value to fit in the buffer
        buffer[count] = value_from_sensor & 0xFF;
        count++;
        buffer[count] = (value_from_sensor >> 8 ) & 0xFF;
        //count++;
        //buffer[count] = (value_from_sensor >> 16 ) & 0xFF;
        //count++;
        //buffer[count] = (value_from_sensor >> 24 ) & 0xFF;
		
    }
	return count;		
}
