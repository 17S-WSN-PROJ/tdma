/*
 *  Wire.h
 *
 TwoWire.h - TWI/I2C library for Arduino & Wiring
 Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 *  Edited for MARS node NanoRK by frank mokaya on 3/6/12.
 *  Copyright 2012 Carnegie Mellon Univ. All rights reserved.
 */

#ifndef Wire_h
#define Wire_h

#include <inttypes.h>

#define BUFFER_LENGTH 32

//class TwoWire
//{
// Moved to wire.c

/*static uint8_t rxBuffer[];
static uint8_t rxBufferIndex;
static uint8_t rxBufferLength;
	
static uint8_t txAddress;
static uint8_t txBuffer[];
static uint8_t txBufferIndex;
static uint8_t txBufferLength;
	
static uint8_t transmitting;
static void (*user_onRequest)(void);
static void (*user_onReceive)(int);*/
static void onRequestService(void);//****
static void onReceiveService(uint8_t*, int);//****

void begin();//****
void begin_uint8(uint8_t);//****
void begin_int(int); //****
void beginTransmission_uint8(uint8_t);//****
void beginTransmission_int(int);//****
uint8_t endTransmission(void);//****
uint8_t requestFrom_2uint8(uint8_t, uint8_t);//****
uint8_t requestFrom_2int(int, int);//****
void send_uint8(uint8_t);//****
void send_uint8st_uint8(uint8_t*, uint8_t);//****
void send_int(int);//****
void send_char(char*);//****
uint8_t available(void);//****
uint8_t receive(void);//****
void onReceive( void (*)(int) );//****
void onRequest( void (*)(void) );//****

#endif


