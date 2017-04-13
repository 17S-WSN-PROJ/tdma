/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Magnetometer Arduino Library.
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
#include <nrk.h>
#include <nrk_driver_list.h> 
#include <HMC5883L.h>
#include <include.h>
#include <stdio.h>
#include <ulib.h>
#include <nrk_error.h>
#include <nrk.h>
#include <stdint.h>
#include <basic_rf.h>
//#include <nrk_timer.h>
#include <hal.h>
#include <Wire.h>
#include <math.h>

int8_t* set_up_HMC5883L()
{
	Mag_m_scale = 1;
	int8_t error[2];
	int8_t temp;
	
	/* set data rate to 75 HZ
	 * set measurement mode to NORMAL
	 * Set the averging to 8 samples per output
	 */
	set_mag_data_rate_75_Hz_max();
	nrk_kprintf( PSTR("HMC5883L(Magnetometer)data rate 75Hz. Averaging 8 samples. . .\r\n") );
	//set scale
	error[0] =  set_mag_scale(1.3);

	//set measurement mode to Continous
	temp =  set_mag_measurement_mode(Measurement_Continuous); // Set the measurement mode to Continuous
	
	if(temp != 0)
	{
	    error[1] = 0;
		//printf("HMC error: continous mode . . .\r\n"); 
	}
	else
	{
		error[1] = 1;
		//printf("HMC measurement mode: continuous . .\r\n");						   
	}
	return error;
}

magnetometer_raw read_magneto_raw_axis()
{
  magnetometer_raw raw;
  uint8_t* buffer = mag_read(DataRegisterBegin, 6);
  
	
  raw.XAxis = (buffer[0] << 8) | buffer[1];
  raw.ZAxis = (buffer[2] << 8) | buffer[3];
  raw.YAxis = (buffer[4] << 8) | buffer[5];
  return raw;
}

magnetometer_scaled read_magneto_scaled_axis()
{
  magnetometer_raw raw = read_magneto_raw_axis();
  magnetometer_scaled scaled;
	
  scaled.XAxis = (int16_t)((raw.XAxis *  Mag_m_scale) * INT_SCALER_MAG);
  scaled.ZAxis = (int16_t)((raw.ZAxis *  Mag_m_scale) * INT_SCALER_MAG);
  scaled.YAxis = (int16_t)((raw.YAxis *  Mag_m_scale) * INT_SCALER_MAG);
  return scaled;
}
					
void get_Magneto_Values_scaled(int16_t * result)
{
	magnetometer_scaled scaled;
	scaled = read_magneto_scaled_axis();
	
	*result = scaled.XAxis; 
	*(result+1) = scaled.YAxis;
	*(result+2) = scaled.ZAxis; 
}
void get_Magneto_Values_flat(int16_t * result) // only good for when compass is level
{
	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	magnetometer_raw MagrawReading;
	MagrawReading = read_magneto_raw_axis();
	*result = MagrawReading.XAxis;
	*(result+1) = MagrawReading.YAxis;
	*(result+2) = MagrawReading.ZAxis;
			
	*(result+3) = atan2((read_magneto_raw_axis()).YAxis, (read_magneto_raw_axis()).XAxis);
			
	// Correct for when signs are reversed.
	if((*(result+3)) < 0) (*(result+3)) += 2*M_PI;
		
	// Convert radians to degrees for readability.
	*(result+4)= (int)( ((*(result+3)) * 180/M_PI) * INT_SCALER_MAG); 
}					
					
					
int  set_mag_scale(float gauss)
{
	uint8_t regValue = 0x00;
	if(gauss == 0.88)
	{
		regValue = 0x00;
		 Mag_m_scale = 0.73;
	}
	else if(gauss == 1.3)
	{
		regValue = 0x01;
		 Mag_m_scale = 0.92;
	}
	else if(gauss == 1.9)
	{
		regValue = 0x02;
		 Mag_m_scale = 1.22;
	}
	else if(gauss == 2.5)
	{
		regValue = 0x03;
		 Mag_m_scale = 1.52;
	}
	else if(gauss == 4.0)
	{
		regValue = 0x04;
		 Mag_m_scale = 2.27;
	}
	else if(gauss == 4.7)
	{
		regValue = 0x05;
		 Mag_m_scale = 2.56;
	}
	else if(gauss == 5.6)
	{
		regValue = 0x06;
		 Mag_m_scale = 3.03;
	}
	else if(gauss == 8.1)
	{
		regValue = 0x07;
		 Mag_m_scale = 4.35;
	}
	else
	{
		 Mag_m_scale = 0.73;
		return ErrorCode_1_Num_accwr;
	}
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	mag_write(ConfigurationRegisterB, regValue);
	
	return ErrorCode_1_Num;// success
}

int  set_mag_measurement_mode(uint8_t mode)
{
	mag_write(ModeRegister, mode);
}

void set_mag_data_rate_75_Hz_max()
{
	mag_write(ConfigurationRegisterA,AVERAGE_8_75_HZ);
}

void mag_write(int address, int data)
{
  beginTransmission_int(HMC5883L_Address);
  send_int(address);
  send_int(data);
  endTransmission();
}

uint8_t* mag_read(int address, int length)
{
  beginTransmission_int(HMC5883L_Address);
  send_int(address);
  endTransmission();
  
  beginTransmission_int(HMC5883L_Address);
 requestFrom_2int(HMC5883L_Address, length);

  uint8_t buffer[length];
  if(available() == length)
  {
	  for(uint8_t i = 0; i < length; i++)
	  {
		  buffer[i] = receive();
	  }
  }
  endTransmission();

  return buffer;
}

/*char* GetErrorText(int errorCode)
{
	if(ErrorCode_1_Num == 1)
		return ErrorCode_1;
	else if(ErrorCode_1_Num == ErrorCode_1_Num_accwr)
		return "HMC Failure. . .Check HMC";
	else
	return "Error not defined.";
}*/