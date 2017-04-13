/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2008-11 by Technische Universität München (www.vmi.ei.tum.de)
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
*
*  Driver for Electronic Assembly DOGM128-6 Full Graphic Display
*
*  Contributing Authors (specific to this file):
*  Peter Diener
*
*
******************************************************************************/


/*
The display has to be connected to usart1 spi
chipselect is 	
A0 is			

example of usage: 	    DOGM128_6_LCD_print("Hello world!", 5*6, 0);

One character is 6 pixels in width, so in the example we start writing at a position that is 5 characters from the left
*/

//Pushbuttons
#define BTleftTop	!(PINC & (1<<3))
#define BTleftBot	!(PINC & (1<<4))
#define BTrightTop	!(PINC & (1<<5))
#define BTrightBot	!(PINC & (1<<6))

// Usage of a button:
// if (BTrightBot) doSomething();

typedef enum {OFF, ON}type_ON_OFF;

// Erase the display completely
void DOGM128_6_clear_display(void);

// Call this only once after boot
void DOGM128_6_display_init(void);

// Switch backlight on or off
// Example usage: display_backlight(ON);
void DOGM128_6_display_backlight(type_ON_OFF backlight);

// Print test on the display, x and y are start coordinates
// x is given in pixels; range: 0..127
// y is given in lines containing a character; range 0..7
// Example: DOGM128_6_LCD_print(hello world, 0, 0);
void DOGM128_6_LCD_print(char text[], unsigned char x, unsigned char y);
