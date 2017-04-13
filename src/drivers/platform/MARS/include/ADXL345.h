/*
ADXL345.h - Header file for the ADXL345 Triple Axis Accelerometer Arduino Library.
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

 Datasheet for ADXL345:
 http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf

*/

#ifndef ADXL345_h
#define ADXL345_h

//#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


/*Define boolean variable*/

//typedef enum {FALSE=0, TRUE} bool;

#define min(x, y) (((x) > (y)) ? (x) : (y))
#define max(x, y) (((x) < (y)) ? (x) : (y))
//#define abs(x) ((x)>0?(x):-(x)) //conflict with stdlib's version?
//#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))



#define DefaultADXL345_Address 0x53//0x1D
#define ADXL345_DEVID 0x00
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_TAP_AXES 0x2a

//define data rate codes for normal mode. ADXL manual ppg 14
#define ADXL345_SAMPL_RATE_32K_HZ 0x0f
#define ADXL345_SAMPL_RATE_16K_HZ 0x0e
#define ADXL345_SAMPL_RATE_800_HZ 0x0d
#define ADXL345_SAMPL_RATE_400_HZ 0x0c
#define ADXL345_SAMPL_RATE_200_HZ 0x0b
#define ADXL345_SAMPL_RATE_100_HZ 0x0a
#define ADXL345_SAMPL_RATE_50_HZ  0x09
#define ADXL345_SAMPL_RATE_25_HZ  0x08

//Data Rate Selectors
#define SAMPL_RATE_32K 10
#define SAMPL_RATE_16K 9
#define SAMPL_RATE_800 8
#define SAMPL_RATE_400 7
#define SAMPL_RATE_200 6
#define SAMPL_RATE_100 5
#define SAMPL_RATE_50  4
#define SAMPL_RATE_25  3

/* 
 Interrupt bit position
 */
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_SINGLE_TAP 0x06
#define ADXL345_DOUBLE_TAP 0x05

#define ADXL345_BW_RATE 0x2C
#define Register_PowerControl 0x2D
#define Register_DataFormat 0x31
#define Register_DataX 0x32
#define Register_DataY 0x34
#define Register_DataZ 0x36

#define ErrorCode_1 "Entered range was invalid. Should be 2, 4, 8 or 16g."
#define ErrorCode_1_Num 1
#define ErrorCode_1_Num_accwr -255

#define INT_SCALER 10000 //value to change floats into ints
#define ScaleFor2G 0.0039
#define ScaleFor4G 0.0078
#define ScaleFor8G 0.0156
#define ScaleFor16G 0.0312

typedef struct
{
	int16_t XAxis;
	int16_t YAxis;
	int16_t ZAxis;
}accelerometer_Scaled;

typedef struct
{
	int16_t XAxis;
	int16_t YAxis;
	int16_t ZAxis;
}accelerometer_Raw;

typedef uint8_t byte;

void set_up_ADXL345();
void acc_power_on();

void get_acc_values(int16_t * result);

/*protected members- was in C++ */
void Write(int address, int byte);
uint8_t* Read(int address, int length);

/** private members was in C++ */
int m_Address;
float m_Scale;
void setRegisterBit(byte regAdress, int bitPos, bool state);
bool getRegisterBit(byte regAdress, int bitPos);  

accelerometer_Raw ReadRawAxis();
accelerometer_Scaled ReadScaledAxis();
  
int SetRange(int range, bool fullResolution);
int EnableMeasurements();

char* GetErrorText(int errorCode);

uint8_t EnsureConnected();

uint8_t IsConnected;
	
	///----Frank Adddition----------///
void setTapThreshold(int tapThreshold);
int  getTapThreshold();
void setTapDuration(int tapDuration);
int  getTapDuration();
void  setDoubleTapLatency(int doubleTapLatency);
int  getDoubleTapLatency();
void setDoubleTapWindow(int doubleTapWindow);
int  getDoubleTapWindow();
bool isTapDetectionOnX();
void setTapDetectionOnX(bool state);
bool isTapDetectionOnY();
void setTapDetectionOnY(bool state);
bool isTapDetectionOnZ();
void setTapDetectionOnZ(bool state);
byte getInterruptSourceVoid(); // was byte instead of uint8t
bool getInterruptSource(byte interruptBit);
bool getInterruptMapping(byte interruptBit);
void setInterruptMapping(byte interruptBit, bool interruptPin);
bool isInterruptEnabled(byte interruptBit);
void setInterrupt(byte interruptBit, bool state);
bool triggered(byte interrupts, int mask);
void acc_set_data_rate(byte rate_selector);
	

	
#endif