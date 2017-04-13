/****************************************************************************
* ITG3200.h - ITG-3200/I2C library v0.5 for Arduino                         *
* Copyright 2010-2011 Filipe Vieira & various contributors					*
* Modified by Frank Mokaya, Carnegie Mellon University						*
* http://code.google.com/p/itg-3200driver                                   *
* This file is part of ITG-3200 Arduino library.                            *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Pei Zhangs design of Lazy board w/ AMTEL Mega 328               *
* SCL     -> pin 28     (no pull up resistors)								*
* SDA     -> pin 27     (no pull up resistors)								*
* CLK & GND -> pin GND                                                      *
* INT       -> not connected  (but can be used)                             *
* VIO & VDD -> pin 3.3V														*
* code modified by: Frank Mokaya											*
*****************************************************************************/
#ifndef ITG3200_h
#define ITG3200_h


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <nrk_time.h>


#define ITG3200_ADDR_AD0_HIGH  0x69   //AD0=1 0x69 I2C address when AD0 is connected to HIGH (VCC) - default for sparkfun breakout
#define ITG3200_ADDR_AD0_LOW   0x68   //AD0=0 0x68 I2C address when AD0 is connected to LOW (GND)- Default for Pei Zhang MARS board
// "The LSB bit of the 7 bit address is determined by the logic level on pin 9. 
// This allows two ITG-3200 devices to be connected to the same I2C bus.
// One device should have pin9 (or bit0) LOW and the other should be HIGH." source: ITG3200 datasheet
// Note that pin9 (AD0 - I2C Slave Address LSB) may not be available on some breakout boards so check 
// the schematics of your breakout board for the correct address to use.

#define GYROSTART_UP_DELAY_MS  70    //time in microsec for halwait fn. 50ms from gyro startup + 20ms register r/w startup
#define INT_SCALER_GYRO 100  

/* ---- Registers ---- */
#define WHO_AM_I           0x00  // RW   SETUP: I2C address   
#define SMPLRT_DIV         0x15  // RW   SETUP: Sample Rate Divider       
#define DLPF_FS            0x16  // RW   SETUP: Digital Low Pass Filter/ Full Scale range
#define INT_CFG            0x17  // RW   Interrupt: Configuration
#define INT_STATUS         0x1A  // R	Interrupt: Status
#define TEMP_OUT           0x1B  // R	SENSOR: Temperature 2bytes
#define GYRO_XOUT          0x1D  // R	SENSOR: Gyro X 2bytes  
#define GYRO_YOUT          0x1F  // R	SENSOR: Gyro Y 2bytes
#define GYRO_ZOUT          0x21  // R	SENSOR: Gyro Z 2bytes
#define PWR_MGM            0x3E  // RW	Power Management

/* ---- bit maps ---- */
#define DLPFFS_FS_SEL             0x18  // 00011000
#define DLPFFS_DLPF_CFG           0x07  // 00000111
#define INTCFG_ACTL               0x80  // 10000000
#define INTCFG_OPEN               0x40  // 01000000
#define INTCFG_LATCH_INT_EN       0x20  // 00100000
#define INTCFG_INT_ANYRD_2CLEAR   0x10  // 00010000
#define INTCFG_ITG_RDY_EN         0x04  // 00000100
#define INTCFG_RAW_RDY_EN         0x01  // 00000001
#define INTSTATUS_ITG_RDY         0x04  // 00000100
#define INTSTATUS_RAW_DATA_RDY    0x01  // 00000001
#define PWRMGM_HRESET             0x80  // 10000000
#define PWRMGM_SLEEP              0x40  // 01000000
#define PWRMGM_STBY_XG            0x20  // 00100000
#define PWRMGM_STBY_YG            0x10  // 00010000
#define PWRMGM_STBY_ZG            0x08  // 00001000
#define PWRMGM_CLK_SEL            0x07  // 00000111

/************************************/
/*    REGISTERS PARAMETERS    */
/************************************/
// Sample Rate Divider
#define NOSRDIVIDER         0 // default    FsampleHz=SampleRateHz/(divider+1)
// Gyro Full Scale Range
#define RANGE2000           3   // default
// Digital Low Pass Filter BandWidth and SampleRate
#define BW256_SR8           0   // default    256 hz lowpas filter BW and 8Khz SR
#define BW188_SR1           1
#define BW098_SR1           2
#define BW042_SR1           3
#define BW020_SR1           4
#define BW010_SR1           5
#define BW005_SR1           6
#define BW000_SR1           7
// Interrupt Active logic lvl
#define ACTIVE_ONHIGH       0 // default
#define ACTIVE_ONLOW        1
// Interrupt drive type
#define PUSH_PULL           0 // default
#define OPEN_DRAIN          1
// Interrupt Latch mode
#define PULSE_50US          0 // default
#define UNTIL_INT_CLEARED   1
// Interrupt Latch clear method
#define READ_STATUSREG      0 // default
#define READ_ANYREG         1
// Power management
#define NORMAL              0 // default
#define STANDBY             1
// Clock Source - user parameters
#define INTERNALOSC         0   // default
#define PLL_XGYRO_REF       1
#define PLL_YGYRO_REF       2
#define PLL_ZGYRO_REF       3
#define PLL_EXTERNAL32      4   // 32.768 kHz
#define PLL_EXTERNAL19      5   // 19.2 Mhz

typedef uint8_t byte;
nrk_time_t gyro_start_wait_time;

float scalefactor[3];    // Scale Factor for gain and polarity
int offsets[3];
uint8_t _dev_address;
uint8_t _buff[6]; 

//function that sets gyro scale factor and offsets
void set_up_ITG3200();
  
// Gyro initialization
void init_gyro_uint_address(uint16_t address);

void init_gyro_all(uint16_t address, byte _SRateDiv, byte _Range, byte _filterBW, byte _ClockSrc, bool _ITGReady, bool _INTRawDataReady);      
    
// Who Am I
byte get_gyro_addr();

void set_gyro_addr(uint16_t _addr);

// Sample Rate Divider
byte get_gyro_sample_rate_div();     

void set_gyro_sample_rate_div(byte _SampleRate);

// Digital Low Pass Filter BandWidth and SampleRate 
byte get_gyro_FS_range();

void set_gyro_FS_range(byte _Range); // RANGE2000

byte get_gyro_filter_BW(); 

void set_gyro_filter_BW(byte _BW); // see register parameters above

// Interrupt Configuration
bool is_gyro_INT_active_On_Low();

void set_gyro_INT_Logic_lvl(bool _State); //ACTIVE_ONHIGH, ACTIVE_ONLOW

// Interrupt drive type
bool is_gyro_INT_open_drain();

void setINTDriveType(bool _State); //OPEN_DRAIN, PUSH_PULL

// Interrupt Latch mode
bool isLatchUntilCleared();

void setLatchMode(bool _State); //UNTIL_INT_CLEARED, PULSE_50US

// Interrupt Latch clear method
bool isAnyRegClrMode();

void setLatchClearMode(bool _State); //READ_ANYREG, READ_STATUSREG

// INT pin triggers
bool isITGReadyOn();          

void setITGReady(bool _State);

bool isRawDataReadyOn();

void setRawDataReady(bool _State);      

// Trigger Status
bool isITGReady();

bool isRawDataReady();

// Gyro Sensors
void readTemp(float *_Temp);  

void read_gyro_raw_xyz_three_ptr( int *_GyroX, int *_GyroY, int *_GyroZ); // uncalibrated raw values

void read_gyro_raw_xyz_one_ptr( int *_GyroXYZ); // uncalibrated raw values

void setScaleFactor(float _Xcoeff, float _Ycoeff, float _Zcoeff, bool _Radians);  // negative ciefficient = Reversed

void setOffsets(int _Xoffset, int _Yoffset, int _Zoffset);

void gyro_zero_calibrate(uint16_t totSamples, uint16_t sampleDelayMS);	// assuming gyroscope is stationary (updates XYZ offsets)

void read_gyro_raw_cal_xyz_three_ptr(int *_GyroX, int *_GyroY, int *_GyroZ); // raw value with offset

void read_gyro_raw_cal_xyz_one_ptr(int *_GyroXYZ); // raw value with offset

void read_gyro_xyz_three_ptr(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ); // deg/sec calibrated & ScaleFactor 

void read_gyro_xyz_one_ptr(int16_t *_GyroXYZ); // deg/sec calibrated & ScaleFactor  

// Power management
void reset(); // after reset all registers have default values

bool isLowPower();

void setPowerMode(bool _State); // NORMAL, STANDBY

bool isXgyroStandby();            

bool isYgyroStandby();

bool isZgyroStandby();

void setXgyroStandby(bool _Status); // NORMAL, STANDBY

void setYgyroStandby(bool _Status);

void setZgyroStandby(bool _Status);

byte getClockSource();

void setClockSource(byte _CLKsource); // see register parameters above
  
void writemem(uint8_t _addr, uint8_t _val);

void readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]);

#endif