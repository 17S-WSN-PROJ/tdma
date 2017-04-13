/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2008-11 Technische Universität München (www.vmi.ei.tum.de)
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
*  Electronic Assembly SMALLPROTOKOLL driver by P. Diener
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/
 

// Befehle/Daten zum Display senden
// Display_send_packet sends a packet to the display.
// The user has to provide a block of memory with the payload and the length of the payload
// In case of success 0 is returned, in case of an error -1 is returned
signed char eDIP240_7_Display_send_packet(char * data, unsigned char length);


// Inhalt des Sendepuffers anfordern
// Display_get_buffer reads a payload packet from the display to a memory block provided by the user
// In case of success the length of data is returned, in case of an error -1 is returned
signed char eDIP240_7_Display_get_buffer(unsigned char * data);


// Pufferinformationen anfordern
// Display_request_buffer_info checks if there is any payload stored in the display that can be transmitted.
// The data is provided by a call by reference method.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_request_buffer_info(char * bytes_ready, char * bytes_free);


// Protokolleinstellungen
// Display_set_protocol can set the maximum transmission block size and the serial interface data block timeout.
// sendbuffer_size is set in bytes (1 .. 64)
// timeout is set in 1/100 seconds (0 .. 255)
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_set_protocol(unsigned char sendbuffer_size, unsigned char timeout);


// Protokollinformationen anfordern
// Display_get_protocoll_info requests the current sendbuffer_size and timeout settings and the sendbuffer_level.
// sendbuffer_size: current setting of the maimum number of bytes that can be stored in the display-sendbuffer.
// sendbuffer_level: current number of bytes that are stored in the display-sendbuffer.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_get_protocoll_info(unsigned char * sendbuffer_size, unsigned char * sendbuffer_level, unsigned char * timeout);


// Letztes Datenpaket wiederholen
// Display_repeat_last_packet will cause the display to repeat the last packet.
// The user has to provide a block of memory (data) where the received packet payload will be stored.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_repeat_last_packet(unsigned char * data);


// Adressierung nur bei RS232/RS485 Betrieb
// Display_select will select the display with the provided address as bus receiver.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_select(unsigned char address);


// Adressierung nur bei RS232/RS485 Betrieb
// Display_deselect will deselect the display with the provided address.
// A deselected display will not listen to data sent over the serial line and will not transmit to the line.
// Return value: 0 -> ok;  -1 -> error


signed char eDIP240_7_Display_deselect(unsigned char address);
// End of Electronic Assembly SMALLPROTOKOLL driver



// Use this for sending ascii commands to the Display
void eDIP240_7_Display_send_string(char * str);


void eDIP240_7_Display_send_string_with_NULL(char * str);
