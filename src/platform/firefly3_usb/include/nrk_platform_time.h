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
*  Anthony Rowe
*  Zane Starr
*  Anand Eswaren
*******************************************************************************/

#ifndef NRK_PLATFORM_TIME_H
#define NRK_PLATFORM_TIME_H


// External OSC = 32768 Hz
// Divde by 32 prescaler

#define NANOS_PER_TICK      976563
#define US_PER_TICK         977 
#define TICKS_PER_SEC       1024 

// Precision OSC = 16000000 Hz
#define NANOS_PER_PRECISION_TICK      	63
// This is the number of nano seconds after which the precsion OS timer overflows
#define NANOS_PER_MAX_PRECISION_TICKS	4096000
//#define PRECISION_TICKS_PER_TICK	488
#define PRECISION_TICKS_PER_TICK  	15501	

// This is the deep sleep wakeup penalty...
#ifndef NRK_SLEEP_WAKEUP_TIME
#define NRK_SLEEP_WAKEUP_TIME	3	
#endif


#define CONTEXT_SWAP_TIME_BOUND    1500 

#endif
