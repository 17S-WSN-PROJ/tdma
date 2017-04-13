/****************************************************************************
 * ITG3200.h - ITG-3200/I2C library v0.5 for Arduino                         *
 * Copyright 2010-2011 Filipe Vieira & various contributors                  *
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
 * Tested on Arduino Mega with ITG-3200 Breakout                             *
 * SCL     -> pin 21     (no pull up resistors)                              *
 * SDA     -> pin 20     (no pull up resistors)                              *
 * CLK & GND -> pin GND                                                      *
 * INT       -> not connected  (but can be used)                             *
 * VIO & VDD -> pin 3.3V                                                     *
 *****************************************************************************/
#include <nrk_driver_list.h>
#include <ITG3200.h>
#include <include.h>
#include <stdio.h>
#include <ulib.h>
#include <nrk_error.h>
#include <nrk.h>
//#include <stdint.h>
#include <basic_rf.h>
#include <Wire.h>
#include <math.h>

void set_up_ITG3200() {
	setOffsets(0,0,0);
	setScaleFactor(1.0, 1.0, 1.0, false);  // true to change readGyro output to radians
	init_gyro_uint_address(ITG3200_ADDR_AD0_LOW);
	nrk_kprintf( PSTR("ITG3200(Gyro) Boot Success. Zero Calibrating. . .\r\n") );
	gyro_zero_calibrate(1000, 2);
	nrk_kprintf( PSTR(" Gyro calibrated\r\n"));
}


void init_gyro_uint_address(uint16_t  address) {
	//ITG3200();
	// Uncomment or change your default ITG3200 initialization
	// fast sample rate - divisor = 0 filter = 0 clocksrc = 0, 1, 2, or 3  (raw values) 
	init_gyro_all(address, NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true);//BW256_SR8
	
	// slow sample rate - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 0, 1, 2, or 3  (raw values)
	//init(NOSRDIVIDER, RANGE2000, BW010_SR1, INTERNALOSC, true, true);
	
	// fast sample rate 32Khz external clock - divisor = 0  filter = 0  clocksrc = 4  (raw values)
	//init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_EXTERNAL32, true, true);
	
	// slow sample rate 32Khz external clock - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 4  (raw values)
	//init(NOSRDIVIDER, RANGE2000, BW010_SR1, PLL_EXTERNAL32, true, true);
}

void init_gyro_all(uint16_t address, byte _SRateDiv, byte _Range, byte _filterBW, byte _ClockSrc, bool _ITGReady, bool _INTRawDataReady) {
	_dev_address = address;
	set_gyro_sample_rate_div(_SRateDiv);
	set_gyro_FS_range(_Range);
	set_gyro_filter_BW(_filterBW);
	setClockSource(_ClockSrc);
	setITGReady(_ITGReady);
	setRawDataReady(_INTRawDataReady);  
	
	//setup gyro startup delay	
	gyro_start_wait_time.secs = 0;
	gyro_start_wait_time.nano_secs = GYROSTART_UP_DELAY_MS*NANOS_PER_MS;
	nrk_wait(gyro_start_wait_time);  // startup 
}

byte get_gyro_addr() {
	/*readmem(WHO_AM_I, 1, &_buff[0]); 
	 return _buff[0];  */
	return _dev_address;
}

void set_gyro_addr(uint16_t  _addr) {
	writemem(WHO_AM_I, _addr); 
	_dev_address = _addr;
}

byte get_gyro_sample_rate_div() {
	readmem(SMPLRT_DIV, 1, &_buff[0]);
	return _buff[0];
}

void set_gyro_sample_rate_div(byte _SampleRate) {//****
	writemem(SMPLRT_DIV, _SampleRate);
}

byte get_gyro_FS_range() { //***
	readmem(DLPF_FS, 1, &_buff[0]);
	return ((_buff[0] & DLPFFS_FS_SEL) >> 3);
}

void set_gyro_FS_range(byte _Range) {
	readmem(DLPF_FS, 1, &_buff[0]);   
	writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_FS_SEL) | (_Range << 3)) ); 
}

byte get_gyro_filter_BW() {  
	readmem(DLPF_FS, 1, &_buff[0]);
	return (_buff[0] & DLPFFS_DLPF_CFG); 
}

void set_gyro_filter_BW(byte _BW) {   
	readmem(DLPF_FS, 1, &_buff[0]);
	writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_DLPF_CFG) | _BW)); 
}

bool is_gyro_INT_active_On_Low() {  
	readmem(INT_CFG, 1, &_buff[0]);
	return ((_buff[0] & INTCFG_ACTL) >> 7);
}

void set_gyro_INT_Logic_lvl(bool _State) { //****
	readmem(INT_CFG, 1, &_buff[0]);
	writemem(INT_CFG, ((_buff[0] & ~INTCFG_ACTL) | (_State << 7))); 
}

bool is_gyro_INT_open_drain() {  
	readmem(INT_CFG, 1, &_buff[0]);
	return ((_buff[0] & INTCFG_OPEN) >> 6);
}

void setINTDriveType(bool _State) {
	readmem(INT_CFG, 1, &_buff[0]);
	writemem(INT_CFG, ((_buff[0] & ~INTCFG_OPEN) | _State << 6)); 
}

bool isLatchUntilCleared() {    
	readmem(INT_CFG, 1, &_buff[0]);
	return ((_buff[0] & INTCFG_LATCH_INT_EN) >> 5);
}

void setLatchMode(bool _State) {
	readmem(INT_CFG, 1, &_buff[0]);
	writemem(INT_CFG, ((_buff[0] & ~INTCFG_LATCH_INT_EN) | _State << 5)); 
}

bool isAnyRegClrMode() {    
	readmem(INT_CFG, 1, &_buff[0]);
	return ((_buff[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

void setLatchClearMode(bool _State) {
	readmem(INT_CFG, 1, &_buff[0]);
	writemem(INT_CFG, ((_buff[0] & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4)); 
}

bool isITGReadyOn() {   
	readmem(INT_CFG, 1, &_buff[0]);
	return ((_buff[0] & INTCFG_ITG_RDY_EN) >> 2);
}

void setITGReady(bool _State) {
	readmem(INT_CFG, 1, &_buff[0]);
	writemem(INT_CFG, ((_buff[0] & ~INTCFG_ITG_RDY_EN) | _State << 2)); 
}

bool isRawDataReadyOn() {
	readmem(INT_CFG, 1, &_buff[0]);
	return (_buff[0] & INTCFG_RAW_RDY_EN);
}

void setRawDataReady(bool _State) {
	readmem(INT_CFG, 1, &_buff[0]);
	writemem(INT_CFG, ((_buff[0] & ~INTCFG_RAW_RDY_EN) | _State)); 
}

bool isITGReady() {
	readmem(INT_STATUS, 1, &_buff[0]);
	return ((_buff[0] & INTSTATUS_ITG_RDY) >> 2);
}

bool isRawDataReady() {
	readmem(INT_STATUS, 1, &_buff[0]);
	return (_buff[0] & INTSTATUS_RAW_DATA_RDY);
}

void readTemp(float *_Temp) {
	readmem(TEMP_OUT,2,_buff);
	*_Temp = 35 + ((_buff[0] << 8 | _buff[1]) + 13200) / 280.0;    // F=C*9/5+32
}

void read_gyro_raw_xyz_three_ptr( int *_GyroX, int *_GyroY, int *_GyroZ){
	readmem(GYRO_XOUT, 6, _buff);
	*_GyroX = _buff[0] << 8 | _buff[1];
	*_GyroY = _buff[2] << 8 | _buff[3]; 
	*_GyroZ = _buff[4] << 8 | _buff[5];
}

void read_gyro_raw_xyz_one_ptr( int *_GyroXYZ){
	read_gyro_raw_xyz_three_ptr(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void setScaleFactor(float _Xcoeff, float _Ycoeff, float _Zcoeff, bool _Radians) { 
	scalefactor[0] = 14.375 * _Xcoeff;   
	scalefactor[1] = 14.375 * _Ycoeff;
	scalefactor[2] = 14.375 * _Zcoeff;    
    
	if (_Radians){
		scalefactor[0] /= 0.0174532925;//0.0174532925 = PI/180
		scalefactor[1] /= 0.0174532925;
		scalefactor[2] /= 0.0174532925;
	}
}

void setOffsets(int _Xoffset, int _Yoffset, int _Zoffset) {
	offsets[0] = _Xoffset;
	offsets[1] = _Yoffset;
	offsets[2] = _Zoffset;
}

void gyro_zero_calibrate(uint16_t totSamples, uint16_t sampleDelayMS) {
	float tmpOffsets[] = {0,0,0};
	int xyz[3];
	nrk_time_t gyro_sample_delay_nanos;
	
	gyro_sample_delay_nanos.secs = 0;
	gyro_sample_delay_nanos.nano_secs = sampleDelayMS*NANOS_PER_MS;
	
	//uint16_t delay_micros = sampleDelayMS*1000;
	
	for (int i = 0;i < totSamples;i++){
		nrk_wait(gyro_sample_delay_nanos);
		if(!(i%100))printf(". ");
		read_gyro_raw_xyz_one_ptr(xyz);
		tmpOffsets[0] += xyz[0];
		tmpOffsets[1] += xyz[1];
		tmpOffsets[2] += xyz[2];
	}
	setOffsets(-tmpOffsets[0] / totSamples + 0.5, -tmpOffsets[1] / totSamples + 0.5, -tmpOffsets[2] / totSamples + 0.5);
}

void read_gyro_raw_cal_xyz_three_ptr(int *_GyroX, int *_GyroY, int *_GyroZ) { 
	read_gyro_raw_xyz_three_ptr(_GyroX, _GyroY, _GyroZ); 
	*_GyroX += offsets[0]; 
	*_GyroY += offsets[1]; 
	*_GyroZ += offsets[2]; 
} 

void read_gyro_raw_cal_xyz_one_ptr(int *_GyroXYZ) { 
	read_gyro_raw_cal_xyz_three_ptr(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2); 
} 

void read_gyro_xyz_three_ptr(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ){
	int x, y, z; 
	read_gyro_raw_cal_xyz_three_ptr(&x, &y, &z); // x,y,z will contain calibrated integer values from the sensor 
	*_GyroX = (int16_t)( (x / scalefactor[0]) * INT_SCALER_GYRO ); 
	*_GyroY = (int16_t)( (y / scalefactor[1]) * INT_SCALER_GYRO ); 
	*_GyroZ = (int16_t)( (z / scalefactor[2]) * INT_SCALER_GYRO );  
} 

void read_gyro_xyz_one_ptr(int16_t *_GyroXYZ){
	read_gyro_xyz_three_ptr(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void reset() {     
	writemem(PWR_MGM, PWRMGM_HRESET); 
	gyro_start_wait_time.secs = 0;
	gyro_start_wait_time.nano_secs = GYROSTART_UP_DELAY_MS*NANOS_PER_MS;
	nrk_wait(gyro_start_wait_time);  // startup delay	
}

bool isLowPower() {   
	readmem(PWR_MGM, 1, &_buff[0]);
	return (_buff[0] & PWRMGM_SLEEP) >> 6;
}

void setPowerMode(bool _State) {
	readmem(PWR_MGM, 1, &_buff[0]);
	writemem(PWR_MGM, ((_buff[0] & ~PWRMGM_SLEEP) | _State << 6));  
}

bool isXgyroStandby() {
	readmem(PWR_MGM, 1, &_buff[0]);
	return (_buff[0] & PWRMGM_STBY_XG) >> 5;
}

bool isYgyroStandby() {
	readmem(PWR_MGM, 1, &_buff[0]);
	return (_buff[0] & PWRMGM_STBY_YG) >> 4;
}

bool isZgyroStandby() {
	readmem(PWR_MGM, 1, &_buff[0]);
	return (_buff[0] & PWRMGM_STBY_ZG) >> 3;
}

void setXgyroStandby(bool _Status) {
	readmem(PWR_MGM, 1, &_buff[0]);
	writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_XG) | _Status << 5));
}

void setYgyroStandby(bool _Status) {
	readmem(PWR_MGM, 1, &_buff[0]);
	writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_YG) | _Status << 4));
}

void setZgyroStandby(bool _Status) {
	readmem(PWR_MGM, 1, &_buff[0]);
	writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_ZG) | _Status << 3));
}

byte getClockSource() {  
	readmem(PWR_MGM, 1, &_buff[0]);
	return (_buff[0] & PWRMGM_CLK_SEL);
}

void setClockSource(byte _CLKsource) {   
	readmem(PWR_MGM, 1, &_buff[0]);
	writemem(PWR_MGM, ((_buff[0] & ~PWRMGM_CLK_SEL) | _CLKsource)); 
}

void writemem(uint8_t _addr, uint8_t _val) {
	beginTransmission_uint8(_dev_address);   // start transmission to device 
	send_uint8(_addr); // send register address
	send_uint8(_val); // send value to write
	endTransmission(); // end transmission
}

void readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
	beginTransmission_uint8(_dev_address); // start transmission to device 
	send_uint8(_addr); // sends register address to read from
	endTransmission(); // end transmission
	
	beginTransmission_uint8(_dev_address); // start transmission to device 
	requestFrom_2uint8(_dev_address, _nbytes);// send data n-bytes read
	uint8_t i = 0; 
	while (available()) {
		__buff[i] = receive(); // receive DATA
		i++;
	}
	endTransmission(); // end transmission
}


