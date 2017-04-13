/*
This code is borrowed from online(see below) and Modified by Frank Mokaya for Carnegie Mellon University Research 
HMC5883L.h - Header file for the HMC5883L Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf

*/

#ifndef HMC5883L_h
#define HMC5883L_h

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define INT_SCALER_MAG 10 //value to change floats into ints

#define HMC5883L_Address 0x1E
#define ConfigurationRegisterA 0x00 // affects the data output Rate (bits DO[n]) & the measurement configuration (bits MS[n])
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02 // controls the operating modes of the compass: 1) continous, 2)single measurement, 3) idle mode
#define DataRegisterBegin 0x03

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle 0x03

#define ErrorCode_1 "Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1"
#define ErrorCode_1_Num 1
#define ErrorCode_1_Num_accwr -1
#define AVERAGE_8_75_HZ 0x78 //ask compass to average 8 measurements at a sample rate 75 Hz

typedef struct 
{
	int16_t XAxis;
	int16_t YAxis;
	int16_t ZAxis;
} magnetometer_scaled;

typedef struct 
{
	int16_t XAxis;
	int16_t YAxis;
	int16_t ZAxis;
} magnetometer_raw;

float Mag_m_scale;

int8_t* set_up_HMC5883L();
magnetometer_raw read_magneto_raw_axis();
magnetometer_scaled read_magneto_scaled_axis();
void get_Magneto_Values_scaled(int16_t * result);
void get_Magneto_Values_flat(int16_t * result);
  
int set_mag_measurement_mode(uint8_t mode);
int set_mag_scale(float gauss);
void set_mag_data_rate_75_Hz_max();

//char* GetErrorText(int errorCode);

void mag_write(int address, int byte);
uint8_t* mag_read(int address, int length);

#endif