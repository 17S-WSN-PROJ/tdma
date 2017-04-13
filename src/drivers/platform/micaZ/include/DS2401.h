/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2011 Technische Universität München (www.vmi.ei.tum.de)
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
*  Serial ID Driver for Maxim DS2401.h
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/

#ifndef _DS2401_H_
#define _DS2401_H_

#include <nrk.h>
		
#define F_CPU   7372800	// CPU core frequency in Hz

// Definition of the communication pin
#define OUTPUT    PORTA
#define INPUT     PINA
#define DIRECTION DDRA
#define PIN       4


inline void DS2401_init(void);

// Reads the 32 bit world wide unique serial number
// valid is set to 0 in case of success, -2 in case of family code mismatch, -1 in case of CRC error
inline uint32_t DS2401_getSerialNumber(int8_t *valid);

#endif
