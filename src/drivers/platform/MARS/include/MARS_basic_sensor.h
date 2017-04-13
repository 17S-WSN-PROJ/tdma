/*
 *  MARS_basic_sensor.h
 *  
 *
 *  Created by frank mokaya on 3/22/12.
 *  Copyright 2012 Carnegie Mellon Univ. All rights reserved.
 *
 */

#define ACC		0
#define GYRO		1
#define MAGNETO		2

uint8_t MARS_sensor_val_to_uint8_buf(int16_t value_from_sensor, uint8_t *buffer, uint8_t sensorID); 