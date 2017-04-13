#include <avr/io.h>
#include <avr/interrupt.h>
#include "ade7878.h"

int8_t ade_init(void){

	int8_t ret;

	// Lock SPI mode
	ret = ade_write8(CONFIG2, 0x0);
	if(ret < 0)
		return -1;
	
	// Pair current inputs with matching voltage inputs
	ret = ade_write16(CONFIG, VTOIA_A | VTOIB_B | VTOIC_C);
	if(ret < 0)
		return -1;

	// Calibrate for -7° phase error
	ret = ade_write16(CPHCAL , 260);
	if(ret < 0)
		return -1;

	// Activate DSP
	ret = ade_write16(RUN, START);
	if(ret < 0)
		return -1;

	return 0;
}
	 
uint8_t ade_read8(uint16_t addr){
	
	uint8_t data;
	
	// Turn on RX LED
	PORTD &= ~(_BV(PORTD6));

	// Initiate communication
	PORTB &= ~(_BV(PORTB0));
	SPDR = 0x01;
	while(!(SPSR & _BV(SPIF)));
	
	// Send register address to be read 
	SPDR = addr >> 0x8;
	while(!(SPSR & _BV(SPIF)));
	SPDR = addr;
	while(!(SPSR & _BV(SPIF)));

	// Receive data
	SPDR = 0x00;
	while(!(SPSR & (1<<SPIF)));
	data = SPDR;

	// Stop communication and turn off RX LED
	PORTB |= _BV(PORTB0);
	PORTD |= _BV(PORTD6);

	return data;
}

uint16_t ade_read16(uint16_t addr){
	
	uint8_t i;
	uint16_t read;

	// Turn on RX LED
	PORTD &= ~(_BV(PORTD6));

	// Initiate communication
	PORTB &= ~(_BV(PORTB0));
	SPDR = 0x01;
	while(!(SPSR & _BV(SPIF)));
	
	// Send register address to be read 
	SPDR = addr >> 0x8;
	while(!(SPSR & _BV(SPIF)));
	SPDR = addr;
	while(!(SPSR & _BV(SPIF)));

	// Receive data
	for(i=0; i<2; i++){
		SPDR = 0x00;
		while(!(SPSR & _BV(SPIF)));
		read = read << 0x8;
		read |= SPDR;	
	}

	// Stop communication and turn off RX LED
	PORTB |= _BV(PORTB0);
	PORTD |= _BV(PORTD6);

	return read;
}

uint32_t ade_read32(uint16_t addr){
	
	uint8_t i;
	uint32_t data;

	// Turn on RX LED
	PORTD &= ~(_BV(PORTD6));

	// Initiate communication
	PORTB &= ~(_BV(PORTB0));
	SPDR = 0x01;
	while(!(SPSR & _BV(SPIF)));
	
	// Send register address to be read 
	SPDR = addr >> 0x8;
	while(!(SPSR & _BV(SPIF)));
	SPDR = addr;
	while(!(SPSR & _BV(SPIF)));

	// Receive data
	for(i=0; i<4; i++){
		SPDR = 0x00;
		while(!(SPSR & _BV(SPIF)));
		data = data << 0x8;
		data |= SPDR;
	}

	// Stop communication and turn off RX LED
	PORTB |= _BV(PORTB0);
	PORTD |= _BV(PORTD6);
	
	return data;
}

int8_t ade_write8(uint16_t addr, uint8_t data){
	
	// Turn on TX LED
	PORTD &= ~(_BV(PORTD7));

	// Initiate communication
	PORTB &= ~(_BV(PORTB0));
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	
	// Send register address to be written to 
	SPDR = addr >> 0x8;
	while(!(SPSR & _BV(SPIF)));
	SPDR = addr;
	while(!(SPSR & _BV(SPIF)));

	// Send data
	SPDR = data;
	while(!(SPSR & _BV(SPIF)));
	
	// Stop communication and turn off TX LED
	PORTB |= _BV(PORTB0);
	PORTD |= _BV(PORTD7);

	// Check what was just written
	if(ade_read8(addr) == data)
		return 0;
	else
		return -1;
}

int8_t ade_write16(uint16_t addr, uint16_t data){

	int8_t i;

	// Turn on TX LED
	PORTD &= ~(_BV(PORTD7));

	// Initiate communication
	PORTB &= ~(_BV(PORTB0));
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	
	// Send register address to be written to 
	SPDR = addr >> 0x8;
	while(!(SPSR & _BV(SPIF)));
	SPDR = addr;
	while(!(SPSR & _BV(SPIF)));

	// Send data
	for(i=8; i>=0; i-=8){
		SPDR = data >> i;
		while(!(SPSR & _BV(SPIF)));
	}
	
	// Stop communication and turn off TX LED
	PORTB |= _BV(PORTB0);
	PORTD |= _BV(PORTD7);

	// Check what was just written
	if(ade_read16(addr) == data)
		return 0;
	else
		return -1;
}

int8_t ade_write32(uint16_t addr, uint32_t data){

	int8_t i;
	
	// Turn on TX LED
	PORTD &= ~(_BV(PORTD7));

	// Initiate communication
	PORTB &= ~(_BV(PORTB0));
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	
	// Send register address to be written to 
	SPDR = addr >> 0x8;
	while(!(SPSR & _BV(SPIF)));
	SPDR = addr;
	while(!(SPSR & _BV(SPIF)));

	// Send data
	for(i=24; i>=0; i-=8){
		SPDR = data >> i;
		while(!(SPSR & _BV(SPIF)));
	}
	
	// Stop communication and turn off TX LED
	PORTB |= _BV(PORTB0);
	PORTD |= _BV(PORTD7);

	// Check what was just written
	if(ade_read32(addr) == data)
		return 0;
	else
		return -1;
}

int8_t ade_set8(uint16_t addr, uint8_t data){
	
	uint8_t current = ade_read8(addr);
	current |= data;
	return ade_write8(addr, current);
}

int8_t ade_set16(uint16_t addr, uint16_t data){

	uint16_t current = ade_read16(addr);
	current |= data;
	return ade_write16(addr, current);
}

int8_t ade_set32(uint16_t addr, uint32_t data){
	
	uint32_t current = ade_read32(addr);
	current |= data;
	return ade_write32(addr, current);
}

int8_t ade_clear8(uint16_t addr, uint8_t data){
	
	uint8_t current = ade_read8(addr);
	current &= ~data;
	return ade_write8(addr, current);
}

int8_t ade_clear16(uint16_t addr, uint16_t data){
	
	uint16_t current = ade_read16(addr);
	current &= ~data;
	return ade_write16(addr, current);
}

int8_t ade_clear32(uint16_t addr, uint32_t data){
	
	uint32_t current = ade_read32(addr);
	current &= ~data;
	return ade_write32(addr, current);
}
