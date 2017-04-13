/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Contributing Authors (specific to this file):
*  Nuno Pereira
*  Anthony Rowe
*******************************************************************************/


#ifndef NRK_PIN_DEFINE_H
#define NRK_PIN_DEFINE_H

/*******************************************************************************************************
 *******************************************************************************************************
 **************************                        GPIO                       **************************
 *******************************************************************************************************
 *******************************************************************************************************/


//---------------------------------------------------------------------------------------------
// Port A

//---------------------------------------------------------------------------------------------
// Port B
// PB.0 is unconnected
#define SCK             1  // PB.1 - Output: SPI Serial Clock (SCLK)          digital  13
//#define LED_2			1
#define GPIO_13         1
#define MOSI            2  // PB.2 - Output: SPI Master out - slave in (MOSI) 
#define MISO            3  // PB.3 - Input:  SPI Master in - slave out (MISO) digital  12
#define GPIO_12         1
#define GPIO_8			4  // PB.4  digital 8
#define GPIO_11			5  // PB.5  digital 11
#define GPIO_10			6  // PB.6  digital 10
#define GPIO_9			7  // PB.7  digital 9

//---------------------------------------------------------------------------------------------
//PORT C

//----------------------------------------------------------------------------------------------
// Port D
#define I2C_SCI			0  //PD.0 i2c 1
#define I2C_SDA			1  //PD.1 i2c 2
#define UART1_RXD       2 // PD.2 - Input:  UART1 RXD  unconnected
#define UART1_TXD       3 // PD.3 - Output: UART1 TXD  unconnected
// PD.4 is unconnected
#define LED_0           5  // PD.5 - Output: RFTX LED 
#define LED_1           6  // PD.6 - Output: RFRX LED
//PD.7 is connected to GND

//----------------------------------------------------------------------------------------------
// Port E
#define UART0_RXD       0 // PE.0 - Input:  UART0 RXD  digital 0
#define UART0_TXD       1 // PE.1 - Output: UART0 TXD  digital 1
#define GPIO_4			2 // PE.2  digital 4
#define GPIO_5			3 // PE.3  digital 5
#define GPIO_6			4 // PE.4  digital 7
#define GPIO_3			5 // PE.5  digital 3
#define GPIO_2			6 // PE.6  digital 2
#define GPIO_7			7 // PE.7  digital 7
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
// Port F
#define ADC_INPUT_0     0 // PF.0 
#define ADC_INPUT_1     1 // PF.1 
#define ADC_INPUT_2     2 // PF.2 
#define ADC_INPUT_3     3 // PF.3 
#define ADC_INPUT_4     4 // PF.4 
#define ADC_INPUT_5     5 // PF.5 
//PF.6 is unconnected 
//PF.7 is unconnected


//-------------------------------------------------------------------------------------------------------
// External RAM interface:
//     PA and PC - Multiplexed address/data
//     PG.0 - Output: Write enable: WR_N
//     PG.1 - Output: Read enable: RD_N
//     PG.2 - Output: Address Latch Enable: ALE
//-------------------------------------------------------------------------------------------------------



//-------------------------------
// GPIO handling functions
// these macros perform raw hw access
// ports and pins are acctual hw ports and pins

// use pin_port, and pin; ie: nkr_gpio_raw_set( PORTB, DEBUG_0 )
#define nrk_gpio_raw_set( _port, _pin ) {do { _port |= BM(_pin); } while(0);}
// use pin_port, and pin; ie: nkr_gpio_raw_clr( PORTB, DEBUG_0 )
#define nrk_gpio_raw_clr( _port, _pin ) {do { _port &= ~BM(_pin); } while(0);}
// use pin_port, and pin; ie: nkr_gpio_raw_get( PINB, DEBUG_0 )
#define nrk_gpio_raw_get( _pin_port, _pin ) (_pin_port & BM(_pin))
// use pin_port, port and pin; ie: nkr_gpio_raw_toggle( PINB, PORTB, DEBUG_0 )
#define nrk_gpio_raw_toggle( _pin_port, _port, _pin ) { \
        if ((_pin_port & BM(_pin))) do{ _port &= ~BM(_pin); } while(0); \
        else do { _port |= BM(_pin); }while(0);  \
}
// use direction; ie: nkr_gpio_raw_direction( DDRB, DEBUG_0 )
#define nrk_gpio_raw_direction( _direction_port_name, _pin, _pin_direction ) { \
        if (_pin_direction == NRK_PIN_INPUT) { \
                _direction_port_name &= ~BM( _pin ); \
        } else { \
                _direction_port_name |= BM( _pin ); \
        } \
}

// when a platform does not support one
// of the NRK_<pin name> declared below, it
// must define it has an invalid pin in the
// platform ulib.c (e.g. a platform that does not
// support NRK_DEBUG_0 should have the following in
// ulib.c NRK_INVALID_PIN( NRK_DEBUG_0 ) )
#define NRK_INVALID_PIN_VAL 0xFF

// nrk ports NRK_<hw port> used for the mapping
// to the real hw. (3 bits reserved for ports)
#define NRK_PORTA 0
#define NRK_PORTB 1
#define NRK_PORTC 2
#define NRK_PORTD 3
#define NRK_PORTE 4
#define NRK_PORTF 5

// define pin directions
#define NRK_PIN_INPUT 0
#define NRK_PIN_OUTPUT 1


//---------------------------------------------------------------------------------------------
// GPIO related definitions

// macros to define a pin as used by higher level programs.
// higher level programs refer to pin as NRK_<pin name>
// these functions declare these NRK_<pin name> pins and provide
// the mappings to the hardware
#define DECLARE_NRK_PIN( _pin_name ) extern const uint8_t NRK_ ## _pin_name;
#define NRK_PIN( _pin_name, _pin , _port ) const uint8_t NRK_ ## _pin_name = (_pin << 3) + (_port & 0x07);
#define NRK_INVALID_PIN( _pin_name ) const uint8_t NRK_ ## _pin_name = NRK_INVALID_PIN_VAL;

// declare pins as used by higher level programs
// mapping to the hardware is done by ulib.c
DECLARE_NRK_PIN( SCK )
//DECLARE_NRK_PIN( LED_2 )
DECLARE_NRK_PIN( MOSI )
DECLARE_NRK_PIN( MISO )
DECLARE_NRK_PIN( GPIO_13 )
DECLARE_NRK_PIN( GPIO_12 )
DECLARE_NRK_PIN( GPIO_11 )
DECLARE_NRK_PIN( GPIO_10 )
DECLARE_NRK_PIN( GPIO_9	)
DECLARE_NRK_PIN( GPIO_8	)


// Port D
DECLARE_NRK_PIN( I2C_SCI )
DECLARE_NRK_PIN( I2C_SDA )
DECLARE_NRK_PIN( UART1_RXD )
DECLARE_NRK_PIN( UART1_TXD )
DECLARE_NRK_PIN( LED_0 )
DECLARE_NRK_PIN( LED_1 )


DECLARE_NRK_PIN( UART0_RXD )
DECLARE_NRK_PIN( UART0_TXD )
DECLARE_NRK_PIN( GPIO_4 )
DECLARE_NRK_PIN( GPIO_5 )
DECLARE_NRK_PIN( GPIO_6 )
DECLARE_NRK_PIN( GPIO_3 )
DECLARE_NRK_PIN( GPIO_2 )
DECLARE_NRK_PIN( GPIO_7 )

// Port F
DECLARE_NRK_PIN( ADC_INPUT_0 )
DECLARE_NRK_PIN( ADC_INPUT_1 )
DECLARE_NRK_PIN( ADC_INPUT_2 )
DECLARE_NRK_PIN( ADC_INPUT_3 )
DECLARE_NRK_PIN( ADC_INPUT_4 )
DECLARE_NRK_PIN( ADC_INPUT_5 )

#endif
