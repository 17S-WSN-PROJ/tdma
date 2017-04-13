/*
ADXL345.cpp - Header file for the ADXL345 Triple Axis Accelerometer Arduino Library.
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
#include <nrk_driver_list.h>
#include <ADXL345.h>
#include <include.h>
#include <stdio.h>
#include <ulib.h>
#include <nrk_error.h>
#include <nrk.h>
#include <stdint.h>
#include <basic_rf.h>
#include <nrk_timer.h>
#include <Wire.h>


#include <math.h>

#define DEVICE (0x53)

//#define SerialDebug

void set_up_ADXL345()
{
  m_Address = DefaultADXL345_Address;
  acc_power_on();
	
  if(EnsureConnected()){
	nrk_kprintf( PSTR("ADXL345(Accelerometer) Boot Success.\r\n") );
  }else {
		printf("Could not connect to ADXL345.\r\n");
  }
  
  // Set the range of the accelerometer to a maximum of 2G.
  if(SetRange(2, true)){
	nrk_kprintf( PSTR("ADXL345 Range set. . .\r\n") );
  }else {
	  nrk_kprintf( PSTR("Could not set range.\r\n") );
  }
	
// Start taking measurements.
  if(EnableMeasurements()){
	  nrk_kprintf( PSTR("ADXL345 Measurement enabled. . .\r\n") );
  }else {
	  nrk_kprintf( PSTR("Enable measurement fail.\r\n") );
  }	

}


accelerometer_Raw ReadRawAxis()
{
#ifdef SerialDebug
	printf( PSTR("Reading raw axis.\n") );
#endif
	accelerometer_Raw raw;
	
	uint8_t* buffer = Read(Register_DataX, 6);
	
	raw.XAxis = (buffer[1] << 8) | buffer[0];
	raw.YAxis = (buffer[3] << 8) | buffer[2];
	raw.ZAxis = (buffer[5] << 8) | buffer[4];
	return raw;
}

accelerometer_Scaled ReadScaledAxis()
{
	uint16_t  scaler = INT_SCALER;
	float xAxis,yAxis,zAxis;
	
	accelerometer_Scaled scaled;
	accelerometer_Raw raw = ReadRawAxis();
	
	xAxis = (float)raw.XAxis * m_Scale * (float)scaler;
	yAxis = (float)raw.YAxis * m_Scale * (float)scaler;
	zAxis = (float)raw.ZAxis * m_Scale * (float)scaler;
	
	scaled.XAxis = (int16_t)xAxis;
	scaled.YAxis = (int16_t)yAxis;
	scaled.ZAxis = (int16_t)zAxis;
	return scaled;
}
void acc_power_on() {
	//Turning on the ADXL345
	Write(Register_PowerControl, 0);      
	Write(Register_PowerControl, 16);
	Write(Register_PowerControl, 8); 
	acc_set_data_rate(SAMPL_RATE_16K);
}
int EnableMeasurements()
{

	#ifdef SerialDebug
		printf("Enabling measurements.");
		printf("/n");
	#endif

		Write(Register_PowerControl, 0x08);
		return 1; //success
	return ErrorCode_1_Num_accwr;
}

void get_acc_values(int16_t * result)
{
	accelerometer_Scaled acc_scaled;
    acc_scaled = ReadScaledAxis();
    *result = acc_scaled.XAxis;
    *(result+1) = acc_scaled.YAxis;
    *(result+2) = acc_scaled.ZAxis;
}

int SetRange(int range, bool fullResolution)
{
	
	#ifdef SerialDebug
		printf("Setting range to: ");
		printf(range);
		printf("/n");
	#endif

		// Get current data from this register.
		uint8_t data = Read(Register_DataFormat, 1)[0];

		// We AND with 0xF4 to clear the bits are going to set.
		// Clearing ----X-XX
		data &= 0xF4;

		// By default (range 2) or FullResolution = true, scale is 2G.
		m_Scale = ScaleFor2G;
	
		// Set the range bits.
		switch(range)
		{
			case 2:
				break;
			case 4:
				data |= 0x01;
				if(!fullResolution) { m_Scale = ScaleFor4G; }
				break;
			case 8:
				data |= 0x02;
				if(!fullResolution) { m_Scale = ScaleFor8G; }
				break;
			case 16:
				data |= 0x03;
				if(!fullResolution) { m_Scale = ScaleFor16G; }
				break;
			default:
				return ErrorCode_1_Num;
		}	

		// Set the full resolution bit.
		if(fullResolution)
			data |= 0x08;

			Write(Register_DataFormat, data);
	return ErrorCode_1_Num;
}

uint8_t EnsureConnected()
{
	
	uint8_t data;
	data = Read(ADXL345_DEVID, 1)[0]; //changed to variable and put in .h file
	if(data == 0xE5)
		IsConnected = true;
	else
		IsConnected = false;

	return IsConnected;
	
}

void Write(int address, int data)
{
#ifdef SerialDebug
	printf("Writing ");
	printf(data, HEX);
	printf(" to register ");
	printf(address, HEX);
	printf("/n");
#endif
	
	beginTransmission_int(m_Address);//Wire.c
	send_int(address);				 //Wire.c
	send_int(data);					 //Wire.c
	endTransmission();
	//rv = ee24xx_write_bytes(eeaddr,len,buf);	
}

uint8_t* Read(int address, int length)
{
	uint8_t buffer[length];
	
#ifdef SerialDebug
	printf("The raw values from the Accelerometer are: \r\n");
	
	for(j = 0; j<length; j++)
        printf("%02x ", buffer[j]);
	printf("\r\n");
#endif

	beginTransmission_int(m_Address);//Wire.c
	send_int(address);			     //Wire.c
	endTransmission();           //Wire.c
  
	beginTransmission_int(m_Address);
	requestFrom_2int(m_Address, length);

	
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

char* GetErrorText(int errorCode)
{
	if(ErrorCode_1_Num == 1)
		return ErrorCode_1;
	
	return "Error not defined.";
}

///----- Frank Addition --------------///
// Sets the THRESH_TAP byte value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior

void setTapThreshold(int tapThreshold) {
	tapThreshold = min(max(tapThreshold,0),255);
	byte _b = (byte)(tapThreshold);
	Write(ADXL345_THRESH_TAP, _b);  
}

// Gets the THRESH_TAP byte value
// return value is comprised between 0 and 255
// the scale factor is 62.5 mg/LSB
int getTapThreshold() {
	byte _b;
	_b = *Read(ADXL345_THRESH_TAP, 1);  
	return (int)(_b);
}
// Sets the DUR byte
// The DUR byte contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625µs/LSB
// A value of 0 disables the tap/double tap funcitons. Max value is 255.
void setTapDuration(int tapDuration) {
	tapDuration = min(max(tapDuration,0),255);
	byte _b = (byte)(tapDuration);
	Write(ADXL345_DUR, _b);  
}

// Gets the DUR byte
int getTapDuration() {
	byte _b;
	_b =  *Read(ADXL345_DUR, 1);  
	return (int)(_b);
}

// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the double tap function.
// It accepts a maximum value of 255.
void setDoubleTapLatency(int doubleTapLatency) {
	byte _b = (byte)(doubleTapLatency);
	Write(ADXL345_LATENT, _b);  
}

// Gets the Latent value
int getDoubleTapLatency() {
	byte _b;
	_b =  *Read(ADXL345_LATENT, 1);  
	return (int)(_b);
}

// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the double tap function. The maximum value is 255.
void setDoubleTapWindow(int doubleTapWindow) {
	doubleTapWindow = min(max(doubleTapWindow,0),255);
	byte _b = (byte) (doubleTapWindow);
	Write(ADXL345_WINDOW, _b);  
}
// Gets the Window register
int getDoubleTapWindow() {
	byte _b;
	_b = *Read(ADXL345_WINDOW, 1);  
  return (int)(_b);
}
byte getInterruptSourceVoid() {
	uint8_t _b;
	_b = *Read(ADXL345_INT_SOURCE, 1);
	return _b;
}
bool getInterruptSource(byte interruptBit) {
	return getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
}

bool getInterruptMapping(byte interruptBit) {
	return getRegisterBit(ADXL345_INT_MAP,interruptBit);
}
void setInterruptMapping(byte interruptBit, bool interruptPin) {
	setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

bool isInterruptEnabled(byte interruptBit) {
	return getRegisterBit(ADXL345_INT_ENABLE,interruptBit);
}

void setInterrupt(byte interruptBit, bool state) {
	setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void setRegisterBit(byte regAdress, int bitPos, bool state) {
	byte _b;
	_b = *Read(regAdress, 1);
	if (state) {
		_b |= (1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
	} 
	else {
		_b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
	}
	Write(regAdress, _b);  
}

bool getRegisterBit(byte regAdress, int bitPos) {
	byte _b;
	_b = *Read(regAdress, 1);
	return ((_b >> bitPos) & 1);
}

bool isTapDetectionOnX(){ 
	return getRegisterBit(ADXL345_TAP_AXES, 2); 
}
void setTapDetectionOnX(bool state) {  
	setRegisterBit(ADXL345_TAP_AXES, 2, state); 
}
bool isTapDetectionOnY(){ 
	return getRegisterBit(ADXL345_TAP_AXES, 1); 
}
void setTapDetectionOnY(bool state) {  
	setRegisterBit(ADXL345_TAP_AXES, 1, state); 
}
bool isTapDetectionOnZ(){ 
	return getRegisterBit(ADXL345_TAP_AXES, 0); 
}
void setTapDetectionOnZ(bool state) {  
	setRegisterBit(ADXL345_TAP_AXES, 0, state);
}
//Used to check if action was triggered in interrupts
//Example triggered(interrupts, ADXL345_SINGLE_TAP);
bool triggered(byte interrupts, int mask){
	return ((interrupts >> mask) & 1);
}

/* sets the data rate by writing to the BW_RATE register.
 * Defaults to 800Hz
 * @param rate_selector - number defining the rate to select. see ADXL345.h
 */
void acc_set_data_rate(byte rate_selector){
	
	byte _s;
	switch (rate_selector) {
		case 10:
			_s = 0x0f;
			break;
		case 9:
			_s = 0x0e;
			break;
		case 8:
			_s = 0x0d;
			break;
		case 7:
			_s = 0x0c;
			break;
		case 6:
			_s = 0x0b;
			break;
		case 5:
			_s = 0x0a;
			break;
		case 4:
			_s = 0x09;
			break;
		case 3:
			_s = 0x08;
			break;
		default:
			_s = 0x0d;
			break;
	}
	
	//int v = (int) (rate / 6.25);
	Write(ADXL345_BW_RATE, _s);
}


