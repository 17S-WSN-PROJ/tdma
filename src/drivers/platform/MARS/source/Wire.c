/*
 TwoWire.cpp - TWI/I2C library for Wiring & Arduino
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
 *
 *  Created by frank mokaya on 3/6/12.
 *  Copyright 2012 Carnegie Mellon Univ. All rights reserved.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "twi.h"
#include "Wire.h"

// Initialize Class Variables //////////////////////////////////////////////////

uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

uint8_t transmitting = 0;
void (*user_onRequest)(void);
void (*user_onReceive)(int);


// Public Methods //////////////////////////////////////////////////////////////

void begin(void)
{
	rxBufferIndex = 0;
	rxBufferLength = 0;
	
	txBufferIndex = 0;
	txBufferLength = 0;
	
	twi_init();
}

void begin_uint8(uint8_t address)
{
	twi_setAddress(address);
	twi_attachSlaveTxEvent(onRequestService);
	twi_attachSlaveRxEvent(onReceiveService);
	begin();
}

void begin_int(int address)
{
	begin_uint8((uint8_t)address);
}

uint8_t requestFrom_2uint8(uint8_t address, uint8_t quantity)
{
	// clamp to buffer length
	if(quantity > BUFFER_LENGTH){
		quantity = BUFFER_LENGTH;
	}
	// perform blocking read into buffer
	uint8_t read = twi_readFrom(address, rxBuffer, quantity);
	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = read;
	
	return read;
}

uint8_t requestFrom_2int(int address, int quantity)
{
	return requestFrom_2uint8((uint8_t)address, (uint8_t)quantity);
}

void beginTransmission_uint8(uint8_t address)
{
	// indicate that we are transmitting
	transmitting = 1;
	// set address of targeted slave
	txAddress = address;
	// reset tx buffer iterator vars
	txBufferIndex = 0;
	txBufferLength = 0;
}

void beginTransmission_int(int address)
{
	beginTransmission_uint8((uint8_t)address);
}

uint8_t endTransmission(void)
{
	// transmit buffer (blocking)
	int8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
	// reset tx buffer iterator vars
	txBufferIndex = 0;
	txBufferLength = 0;
	// indicate that we are done transmitting
	transmitting = 0;
	return ret;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void send_uint8(uint8_t data)
{
	if(transmitting){
		// in master transmitter mode
		// don't bother if buffer is full
		if(txBufferLength >= BUFFER_LENGTH){
			return;
		}
		// put byte in tx buffer
		txBuffer[txBufferIndex] = data;
		++txBufferIndex;
		// update amount in buffer   
		txBufferLength = txBufferIndex;
	}else{
		// in slave send mode
		// reply to master
		twi_transmit(&data, 1);
	}
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void send_uint8st_uint8(uint8_t* data, uint8_t quantity)
{
	if(transmitting){
		// in master transmitter mode
		for(uint8_t i = 0; i < quantity; ++i){
			send_uint8(data[i]);
		}
	}else{
		// in slave send mode
		// reply to master
		twi_transmit(data, quantity);
	}
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void send_char(char* data)
{
	send_uint8st_uint8((uint8_t*)data, strlen(data));
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void send_int(int data)
{
	send_uint8((uint8_t)data);
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
uint8_t available(void)
{
	return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
uint8_t receive(void)
{
	// default to returning null char
	// for people using with char strings
	uint8_t value = '\0';
	
	// get each successive byte on each call
	if(rxBufferIndex < rxBufferLength){
		value = rxBuffer[rxBufferIndex];
		++rxBufferIndex;
	}
	
	return value;
}

// behind the scenes function that is called when data is received
void onReceiveService(uint8_t* inBytes, int numBytes)
{
	// don't bother if user hasn't registered a callback
	if(!user_onReceive){
		return;
	}
	// don't bother if rx buffer is in use by a master requestFrom() op
	// i know this drops data, but it allows for slight stupidity
	// meaning, they may not have read all the master requestFrom() data yet
	if(rxBufferIndex < rxBufferLength){
		return;
	}
	// copy twi rx buffer into local read buffer
	// this enables new reads to happen in parallel
	for(uint8_t i = 0; i < numBytes; ++i){
		rxBuffer[i] = inBytes[i];    
	}
	// set rx iterator vars
	rxBufferIndex = 0;
	rxBufferLength = numBytes;
	// alert user program
	user_onReceive(numBytes);
}

// behind the scenes function that is called when data is requested
void onRequestService(void)
{
	// don't bother if user hasn't registered a callback
	if(!user_onRequest){
		return;
	}
	// reset tx buffer iterator vars
	// !!! this will kill any pending pre-master sendTo() activity
	txBufferIndex = 0;
	txBufferLength = 0;
	// alert user program
	user_onRequest();
}

// sets function called on slave write
void onReceive( void (*function)(int) )
{
	user_onReceive = function;
}

// sets function called on slave read
void onRequest( void (*function)(void) )
{
	user_onRequest = function;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

// TwoWire Wire = TwoWire();

