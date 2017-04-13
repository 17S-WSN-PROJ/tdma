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
*  eDIP240-7 TouchPanel 
*
*  Contributing Authors (specific to this file):
*  Author: Peter Diener
*
***************************************************************************************/
#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <stdlib.h>
#include <hal.h>

// Interface functions
void USART0_putchar(char data)
{
	putchar(data);
}



signed int USART0_getchar()
{
	if (nrk_uart_data_ready(NRK_DEFAULT_UART)!=0)
		return (signed int)(unsigned char)getchar();
	else return -1;
}




// Electronic Assembly SMALLPROTOKOLL driver by P. Diener
// Befehle/Daten zum Display senden
// Display_send_packet sends a packet to the display.
// The user has to provide a block of memory with the payload and the length of the payload
// In case of success 0 is returned, in case of an error -1 is returned
signed char eDIP240_7_Display_send_packet(char * data, unsigned char length)
{
 unsigned char i=0, checksum=0, ack=0;
 volatile unsigned long ack_timeout=100000;

 USART0_putchar(0x11);       // Send command (sending packet) to the display
 USART0_putchar(length);     // Send payload size to the display
 checksum = 0x11 + length;
 while(i < length)
 {
	USART0_putchar(data[i]);  // Send payload to the display
	checksum += data[i];
	i++;
 }
 USART0_putchar(checksum);   // Send checksum to the display
// while (Tx0Empty == 0) {}    // Wait for complete transmission of the packet
 do                          // Wait for receiving ack from the display
 {
	ack = USART0_getchar();
	ack_timeout--;
 }while((ack != 0x06) && (ack_timeout > 0));

 if (ack == 0x06) return 0;  // Success!
 else return -1;             // No success! User has to repeat the packet
}



// Inhalt des Sendepuffers anfordern
// Display_get_buffer reads a payload packet from the display to a memory block provided by the user
// In case of success the length of data is returned, in case of an error -1 is returned
signed char eDIP240_7_Display_get_buffer(unsigned char * data)
{
 unsigned char i=0, checksum=0, ack=0, error=0, length = 0;
 signed int ch;
 volatile unsigned long ack_timeout=100000;

 USART0_putchar(0x12);                 // Send command (sending packet) to the display
 USART0_putchar(0x01);                 // Send payload size to the display
 USART0_putchar(0x53);                 // Payload
 USART0_putchar(0x12 + 0x01 + 0x53);   // Send checksum to the display
// while (Tx0Empty == 0) {}              // Wait for complete transmission of the packet
 do                                    // Wait for receiving ack from the display
 {
	ack = USART0_getchar();
	ack_timeout--;
 }while((ack != 0x06) && (ack_timeout > 0));
 if (ack != 0x06) error = 1;

 // Now we have to receive and parse the answer from the display
 do{ ch = USART0_getchar(); }while (ch == -1);       // Get the command byte
 if (ch == 0x11)                                     // Abort if command byte is incorrect
 {
	do{ ch = USART0_getchar(); }while (ch == -1);     // Get payload length
	length = ch;
	checksum = 0x11 + length;                         // Start checksum calculation
	while (i < length)
	{
	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get payload data
	  data[i] = ch;
	  checksum += ch;                                 // Calculate checksum
	  i++;
	}
	// Now we have received all the payload bytes.
	do{ ch = USART0_getchar(); }while (ch == -1);   // Get checksum
	// If received checksum and calculated checksum do not match, we have an error!
	if (ch != checksum) error = 1;
 }
 else error = 1;

 if (error == 0) return length;   // Success!
 else return -1;                   // No success! User has to repeat the packet
}



// Pufferinformationen anfordern
// Display_request_buffer_info checks if there is any payload stored in the display that can be transmitted.
// The data is provided by a call by reference method.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_request_buffer_info(char * bytes_ready, char * bytes_free)   //TBD
{
 unsigned char checksum=0, ack=0, error=0, length;
 signed int ch;
 volatile unsigned long ack_timeout=1000000;

 USART0_putchar(0x12);                 // Send command (sending packet) to the display
 USART0_putchar(0x01);                 // Send payload size to the display
 USART0_putchar(0x49);                 // Payload
 USART0_putchar(0x12 + 0x01 + 0x49);   // Send checksum to the display
 do                                    // Wait for receiving ack from the display
 {
	ack = USART0_getchar();
	ack_timeout--;
 }while((ack != 0x06) && (ack_timeout > 0));
 if (ack != 0x06) error = 1;

 // Now we have to receive and parse the answer from the display
 do{ ch = USART0_getchar(); }while (ch == -1);       // Get the command byte
 if (ch == 0x12)                                     // Abort if command byte is incorrect
 {
	do{ ch = USART0_getchar(); }while (ch == -1);     // Get payload length
	length = ch;
	if (length == 2)                                  // Abort if length is incorrect
	{
	  checksum = 0x12 + length;                         // Start checksum calculation

	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get payload data
	  checksum += ch;                                 // Calculate checksum
	  *bytes_ready = ch;

	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get payload data
	  checksum += ch;                                 // Calculate checksum
	  *bytes_free = ch;

	  // Now we have received all the payload bytes.
	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get checksum
	  // If received checksum and calculated checksum do not match, we have an error!
	  if (ch != checksum) error = 1;
	}
	else error = 1;
 }
 else error = 1;

 if (error == 0) return 0;  // Success!
 else return -1;             // No success! User has to repeat the packet
}


// Protokolleinstellungen
// Display_set_protocol can set the maximum transmission block size and the serial interface data block timeout.
// sendbuffer_size is set in bytes (1 .. 64)
// timeout is set in 1/100 seconds (0 .. 255)
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_set_protocol(unsigned char sendbuffer_size, unsigned char timeout)
{
 unsigned char ack=0;
 volatile unsigned long ack_timeout=1000000;

 USART0_putchar(0x12);                 // Send command DC2 (sending packet) to the display
 USART0_putchar(3);
 USART0_putchar('D');
 USART0_putchar(sendbuffer_size);
 USART0_putchar(timeout);
 USART0_putchar( (unsigned char) (0x12 + 3 + 'D' + sendbuffer_size + timeout) );                     // Checksum
 do                                    // Wait for receiving ack from the display
 {
	ack = USART0_getchar();
	ack_timeout--;
 }while((ack != 0x06) && (ack_timeout > 0));
 if (ack != 0x06) return (-1);
 return 0;
}



// Protokollinformationen anfordern
// Display_get_protocoll_info requests the current sendbuffer_size and timeout settings and the sendbuffer_level.
// sendbuffer_size: current setting of the maimum number of bytes that can be stored in the display-sendbuffer.
// sendbuffer_level: current number of bytes that are stored in the display-sendbuffer.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_get_protocoll_info(unsigned char * sendbuffer_size, unsigned char * sendbuffer_level, unsigned char * timeout)
{
 unsigned char checksum=0, ack=0, error=0, length;
 signed int ch;
 volatile unsigned long ack_timeout=1000000;

 USART0_putchar(0x12);                 // Send command DC2 (sending packet) to the display
 USART0_putchar(1);
 USART0_putchar('P');
 USART0_putchar( (unsigned char) (0x12 + 1 + 'P') );
 do                                    // Wait for receiving ack from the display
 {
	ack = USART0_getchar();
	ack_timeout--;
 }while((ack != 0x06) && (ack_timeout > 0));
 if (ack != 0x06) return (-1);

 // Now we have to receive and parse the answer from the display
 do{ ch = USART0_getchar(); }while (ch == -1);       // Get the command byte
 if (ch == 0x12)                                     // Abort if command byte is incorrect
 {
	do{ ch = USART0_getchar(); }while (ch == -1);     // Get payload length
	length = ch;
	if (length == 3)                                  // Abort if length is incorrect
	{
	  checksum = 0x12 + length;                         // Start checksum calculation

	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get payload data
	  checksum += ch;                                 // Calculate checksum
	  *sendbuffer_size = ch;

	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get payload data
	  checksum += ch;                                 // Calculate checksum
	  *sendbuffer_level = ch;

	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get payload data
	  checksum += ch;                                 // Calculate checksum
	  *timeout = ch;

	  // Now we have received all the payload bytes.
	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get checksum
	  // If received checksum and calculated checksum do not match, we have an error!
	  if (ch != checksum) error = 1;
	}
	else error = 1;
 }
 else error = 1;

 if (error == 0) return 0;  // Success!
 else return -1;             // No success! User has to repeat the packet

 return 0;

}



// Letztes Datenpaket wiederholen
// Display_repeat_last_packet will cause the display to repeat the last packet.
// The user has to provide a block of memory (data) where the received packet payload will be stored.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_repeat_last_packet(unsigned char * data)
{
 unsigned char i=0, checksum=0, ack=0, error=0, length;
 signed int ch;
 volatile unsigned long ack_timeout=1000000;

 USART0_putchar(0x12);                 // Send command DC2 (sending packet) to the display
 USART0_putchar(1);
 USART0_putchar('R');
 USART0_putchar( (unsigned char) (0x12 + 1 + 'R') );                     // Checksum
 do                                    // Wait for receiving ack from the display
 {
	ack = USART0_getchar();
	ack_timeout--;
 }while((ack != 0x06) && (ack_timeout > 0));
 if (ack != 0x06) return (-1);

 // Now we have to receive and parse the answer from the display
 do{ ch = USART0_getchar(); }while (ch == -1);       // Get the command byte
 if (ch == 0x11)                                     // Abort if command byte is incorrect
 {
	do{ ch = USART0_getchar(); }while (ch == -1);     // Get payload length
	length = ch;
	checksum = 0x11 + length;                         // Start checksum calculation
	while (i < length)
	{
	  do{ ch = USART0_getchar(); }while (ch == -1);   // Get payload data
	  data[i] = ch;
	  checksum += ch;                                 // Calculate checksum
	  i++;
	}
	// Now we have received all the payload bytes.
	do{ ch = USART0_getchar(); }while (ch == -1);   // Get checksum
	// If received checksum and calculated checksum do not match, we have an error!
	if (ch != checksum) error = 1;
 }
 else error = 1;

 if (ack == 0x06) return 0;  // Success!
 else return -1;             // No success! User has to repeat the packet
}



// Adressierung nur bei RS232/RS485 Betrieb
// Display_select will select the display with the provided address as bus receiver.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_select(unsigned char address)
{
 unsigned char ack=0;
 volatile unsigned long ack_timeout=1000000;

 USART0_putchar(0x12);                 // Send command DC2 (sending packet) to the display
 USART0_putchar(3);
 USART0_putchar('A');
 USART0_putchar('S');
 USART0_putchar(address);
 USART0_putchar( (unsigned char) (0x12 + 3 + 'A' + 'S' + address) );                     // Checksum
 do                                    // Wait for receiving ack from the display
 {
	ack = USART0_getchar();
	ack_timeout--;
 }while((ack != 0x06) && (ack_timeout > 0));
 if (ack != 0x06) return (-1);
 return 0;
}


// Adressierung nur bei RS232/RS485 Betrieb
// Display_deselect will deselect the display with the provided address.
// A deselected display will not listen to data sent over the serial line and will not transmit to the line.
// Return value: 0 -> ok;  -1 -> error
signed char eDIP240_7_Display_deselect(unsigned char address)
{
 unsigned char ack=0;
 volatile unsigned long ack_timeout=1000000;

 USART0_putchar(0x12);                 // Send command DC2 (sending packet) to the display
 USART0_putchar(3);
 USART0_putchar('A');
 USART0_putchar('D');
 USART0_putchar(address);
 USART0_putchar( (unsigned char) (0x12 + 3 + 'A' + 'D' + address) );                     // Checksum
 do                                    // Wait for receiving ack from the display
 {
	ack = USART0_getchar();
	ack_timeout--;
 }while((ack != 0x06) && (ack_timeout > 0));
 if (ack != 0x06) return (-1);
 return 0;
}
// End of Electronic Assembly SMALLPROTOKOLL driver



// Use this for sending ascii commands to the Display
void eDIP240_7_Display_send_string(char * str)
{
 eDIP240_7_Display_send_packet(str, strlen(str));
}



void eDIP240_7_Display_send_string_with_NULL(char * str)
{
 eDIP240_7_Display_send_packet(str, strlen(str) + 1);
}
