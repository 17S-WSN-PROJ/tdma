/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2014, Wireless Sensing and Embedded Systems (WiSE) Lab, Carnegie Mellon University
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
*******************************************************************************/

#include <include.h>
#include <ulib.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <nrk.h>
#include <nrk_events.h>
#include <nrk_timer.h>
#include <nrk_error.h>
#include <nrk_reserve.h>
#include <sparkmac.h>
#include <sparkmac_pkt.h>
#include <nrk_cfg.h>

//#define GPIO_DEBUG
#define TX_PKT_RETRY	1



#ifndef SM_STACKSIZE
#define SM_STACKSIZE	256 
#endif



static nrk_task_type sm_task;
static NRK_STK sm_task_stack[SM_STACKSIZE];


// These are global packet structures used by the network task.
// When packets arrive, the sm_rx_pkt is populated with network
// header information and contains the payload data for apps.
// This data is then copied into the applications sm_rec()'s buf
// buffer. Likewise, the sm_tx_pkt is used internally for all tx data. 
static sm_pkt_t sm_app_tx_pkt;
static sm_pkt_t sm_tx_pkt;
static sm_pkt_t sm_rx_pkt;

static uint8_t rx_buf[SM_MAX_PKT_SIZE];
static uint8_t tx_buf[SM_MAX_PKT_SIZE];


// These buffers hold the payload packets to and from the application
static uint8_t *app_tx_payload;
static uint8_t app_tx_len;
static uint8_t app_rx_len;

// General state variables held from the EEPROM
// Good for routing, adding to last_hop etc.
static uint8_t sm_my_mac;
static uint8_t sm_chan;
static uint8_t sm_is_enabled;

// Used to coordinate protocol startup
static volatile uint8_t sm_running;


// This flag is true when there is pending outgoing
// data from the application.  
static uint8_t sm_app_tx_data_ready;

// This flag is true when there is incoming data
// buffered for the application.
static uint8_t sm_app_rx_data_ready;



static nrk_time_t tmp_time;

static int8_t tx_reserve;

// This keeps track of how many incoming messages are dropped
// due to the application not calling sm_receive() quickly enough 
// when packets are arriving. Ideally, this should always be 0.
static uint8_t sm_rx_failure_cnt; 

// If for some reason you want to disable AES Encryption
// this flag holds that state.
static uint8_t sm_aes_enabled; 

// Mode variable holds if the node is an
// EDGE_NODE   normal metering device
// ROUTER      forwarding node
// GATEWAY     node that connects to computer
static uint8_t sm_mode; 

static int rx_handler_flag=0;

void _rx_handler(void)
{
nrk_event_signal(sm_rx_pkt_signal);
//nrk_kprintf(PSTR("RX callback\r\n"));
// Send signal to wakeup NW task
//nrk_kprintf(PSTR("sig sent\r\n"));
//rx_handler_flag=1;
}

void sm_aes_setkey(uint8_t *key)
{
uint8_t i;
	aes_setkey(key);
}


void sm_aes_enable()
{
  sm_aes_enabled=1;
}

void sm_aes_disable()
{
  sm_aes_enabled=1;
}


int8_t sm_set_rf_power (uint8_t power)
{
  if (power > 31)
    return NRK_ERROR;
  rf_tx_power (power);
  return NRK_OK;
}

int8_t sm_set_channel (uint8_t chan)
{
  if (chan > 26)
    return NRK_ERROR;
  sm_chan = chan;
  rf_init (&sm_rfRxInfo, chan, 0x2420, 0x1214);
  return NRK_OK;
}


int8_t sm_send (uint8_t *buf, uint8_t len)
{
  uint32_t mask;
  uint8_t i;
  int8_t v;


// If there is already a packet waiting to TX, return error
  if (sm_app_tx_data_ready == 1)
     return NRK_ERROR;

// If the buffer is chunk, return error
  if (buf  == NULL)
    return NRK_ERROR;

// If length is out of bounds, return error 
  if (len == 0 || len > MAX_PAYLOAD)
    return NRK_ERROR;

// Reservations check how often a node can transmit per unit
// of time to stop them from spamming the network.
// Check if reservation exists, if so add a transmit 
#ifdef NRK_MAX_RESERVES
  if (tx_reserve != -1) {
    if (nrk_reserve_consume (tx_reserve) == NRK_ERROR) {
      return NRK_ERROR;
    }
  }
#endif

  // Copy user payload into app
  for(i=0; i<len; i++ ) sm_app_tx_pkt.payload[i]=buf[i];
   sm_app_tx_pkt.payload_len=len;
   sm_app_tx_pkt.src_mac=sm_my_mac;
   sm_app_tx_pkt.dst_mac=0x0;
   sm_app_tx_pkt.proto_version=1;
   sm_app_tx_pkt.mac_contention_control=0x1;
   sm_app_tx_pkt.ttl=3;
   sm_app_tx_pkt.seq_num++;

  // Flag for network task to know that data is ready to be sent
  sm_app_tx_data_ready = 1;

  nrk_event_signal (sm_tx_pkt_signal);


  // Wait for network task to actually send the packet 
  nrk_event_wait (SIG (sm_tx_pkt_done_signal));

  return NRK_OK;
}


// This function takes an application buffer (up to MAX_PAYLOAD) and 
// copies any incomming payload destine for the app.  It is blocking and
// returns once new data arrives.
int8_t sm_recv (uint8_t *buf)
{

  nrk_sig_mask_t event;
  uint8_t i;
  int8_t v;


  // Check if another sm_recv is already running...
  if (sm_app_rx_data_ready == 0) 
	{
  	// Wait for NW to signal new application data
  	nrk_signal_register (sm_rx_pkt_ready_signal);

  	// Wait for signal from network stack
	event = nrk_event_wait (SIG (sm_rx_pkt_ready_signal));
	}

	// Copy payload from main struct to app payload memory
  	for(i=0; i<sm_rx_pkt.payload_len; i++ )
			buf[i]=sm_rx_pkt.payload[i];
	app_rx_len=sm_rx_pkt.payload_len;

  // Clear the lock on app data now that it has been copied 
  sm_app_rx_data_ready = 0;

  // Return the payload length
  return app_rx_len; 

}

void sm_ttl_set(uint8_t ttl)
{
sm_app_tx_pkt.ttl=ttl;
}

int8_t sm_init (uint8_t mode, uint8_t chan, uint16_t my_mac)
{
  tx_reserve = -1;
  sm_rx_failure_cnt = 0;
  sm_mode = mode;
  //sm_ttl=SM_DEFAULT_TTL;


  #ifdef GPIO_DEBUG
    nrk_gpio_direction(NRK_MOSI,NRK_PIN_OUTPUT);
    nrk_gpio_direction(NRK_MISO,NRK_PIN_OUTPUT);
  #endif

  sm_rx_pkt_signal = nrk_signal_create ();
  if (sm_rx_pkt_signal == NRK_ERROR) {
    nrk_kprintf (PSTR ("SM ERROR: creating rx signal failed\r\n"));
    nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
    return NRK_ERROR;
  }

  sm_rx_pkt_ready_signal = nrk_signal_create ();
  if (sm_rx_pkt_ready_signal == NRK_ERROR) {
    nrk_kprintf (PSTR ("SM ERROR: creating rx signal ready failed\r\n"));
    nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
    return NRK_ERROR;
  }

  sm_tx_pkt_signal = nrk_signal_create ();
  if (sm_tx_pkt_signal == NRK_ERROR) {
    nrk_kprintf (PSTR ("SM ERROR: creating tx signal ready failed\r\n"));
    nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
    return NRK_ERROR;
  }

  sm_tx_pkt_done_signal = nrk_signal_create ();
  if (sm_tx_pkt_done_signal == NRK_ERROR) {
    nrk_kprintf (PSTR ("SM ERROR: creating tx signal failed\r\n"));
    nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
    return NRK_ERROR;
  }


//printf( "rx_pkt_ready=%d\r\n",sm_rx_pkt_ready_signal );
//printf( "rx_pkt_signal=%d\r\n",sm_rx_pkt_signal);
//printf( "tx_pkt_signal=%d\r\n",sm_tx_pkt_signal);
//printf( "tx_pkt_done_signal=%d\r\n",sm_tx_pkt_done_signal);

/*  sm_enable_signal = nrk_signal_create ();
  if (sm_enable_signal == NRK_ERROR) {
    nrk_kprintf (PSTR ("SM ERROR: creating enable signal failed\r\n"));
    nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
    return NRK_ERROR;
  }
*/
  // Set the one main rx buffer
  sm_rfRxInfo.pPayload = rx_buf;
  sm_rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
  sm_rfRxInfo.ackRequest= 0;
  sm_app_rx_data_ready = 0;

  // Set the one main tx buffer
  sm_rfTxInfo.pPayload = tx_buf;
  sm_rfTxInfo.length = 0;
  sm_rfTxInfo.destAddr = 0xffff;
  sm_rfTxInfo.cca = 0;
  sm_rfTxInfo.ackRequest = 0;
  sm_app_tx_data_ready = 0;


  // Setup the radio 
  // XXX: Set the subnet to the SID
  rf_init (&sm_rfRxInfo, chan, 0xffff, my_mac);
  sm_chan = chan;
  sm_my_mac = my_mac;

  //FASTSPI_SETREG (CC2420_RSSI, 0xE580); // CCA THR=-25
  //FASTSPI_SETREG (CC2420_TXCTRL, 0x80FF);       // TX TURNAROUND = 128 us
  //FASTSPI_SETREG (CC2420_RXCTRL1, 0x0A56);
  // default cca thresh of -45
  rf_set_cca_thresh (-45);

  asm volatile ("":::"memory");
  sm_is_enabled = 1;


  return NRK_OK;
}


nrk_sig_t sm_get_rx_pkt_signal ()
{
  nrk_signal_register (sm_rx_pkt_signal);
  return (sm_rx_pkt_signal);
}

nrk_sig_t sm_get_tx_done_signal ()
{
  nrk_signal_register (sm_tx_pkt_done_signal);
  return (sm_tx_pkt_done_signal);
}




int8_t sm_rx_pkt_release(void)
{
    sm_app_rx_data_ready=1;
return NRK_OK;
}


void sm_disable ()
{
  sm_is_enabled = 0;
  rf_power_down ();
}

void sm_enable ()
{
  sm_is_enabled = 1;
  rf_power_up ();
}

void topology_module()
{
nrk_kprintf( PSTR("Calling topologoy module\r\n" ));
// Add footer to sm_tx_pkt 
}

uint8_t router_module()
{
// sm_rx_pkt has the incoming data
// sm_tx_pkt should be set for outgoing data
nrk_kprintf( PSTR("Calling router module\r\n" ));
// don't forward
return 0;
}

void sm_nw_task ()
{
  int8_t v, i,fwd;
  nrk_sig_mask_t my_sigs;


  // Wait until after sm_init() is called...
  // Note, that init creates the signals
  do {
    nrk_wait_until_next_period ();
  } while (!sm_is_enabled);

  v=nrk_signal_register (sm_rx_pkt_signal);
   if(v==NRK_ERROR) nrk_kprintf(PSTR("sig reg failed\r\n"));
  v=nrk_signal_register (sm_tx_pkt_signal);
   if(v==NRK_ERROR) nrk_kprintf(PSTR("sig reg failed\r\n"));



  // Setup RX packet call back
  // Callback does first TS call

  // Set the RX packet handler callback
  rx_start_callback(&_rx_handler);
  rf_power_up();
  rf_rx_on();

  // Set the go flag for everone else...
  sm_running = 1;


  while (1) {

  	// Turn radio on
  	rf_rx_on();

	my_sigs=nrk_event_wait( SIG(sm_rx_pkt_signal) | SIG(sm_tx_pkt_signal) ); 
	if(my_sigs==0) continue;

	// Wait for signal...
	 if(my_sigs & SIG(sm_rx_pkt_signal))
	 {
	   v=sm_rx_raw_pkt(&sm_rx_pkt);
	   if(v==NRK_ERROR) continue;
	   // At this point sm_rx_pkt should be good and ready to process

	     fwd=router_module();
	     if(fwd==1)
	     {
	         // Forward packet
	         topology_module();
		 sm_tx_raw_pkt(&sm_tx_pkt);
	     }

	     // Check if this packet is for this node
	     if(sm_rx_pkt.dst_mac==sm_my_mac || sm_rx_pkt.dst_mac==0xffff) 
	       {
	         if(sm_app_rx_data_ready==0 )
	         {
	           sm_rx_failure_cnt=0;
		   sm_app_rx_data_ready=1;
 	         nrk_event_signal (sm_rx_pkt_ready_signal);
  	         } else
  		   sm_rx_failure_cnt++;
	      }
	}
	
	if((my_sigs & SIG(sm_tx_pkt_signal)) || sm_app_tx_data_ready)
	{
	// Turn off receiver so we can TX
	   if(sm_app_tx_data_ready)
		{
		   sm_tx_raw_pkt(&sm_app_tx_pkt);
		   sm_app_tx_data_ready=0;
		}
	   // Send signal to release sm_send() which is waiting...
  	   nrk_event_signal (sm_tx_pkt_done_signal);
	} 
	
	}	      	

}


int8_t sm_started ()
{
  return sm_running;
}




int8_t sm_rx_raw_pkt (sm_pkt_t *p)
{
  int8_t v, i;
  v = 0;


  #ifdef GPIO_DEBUG
    nrk_gpio_set(NRK_MISO);
  #endif
     v = rf_rx_packet_nonblock ();
      if (v == 1) {
        // Grab packet, do good stuff
        if (sm_rfRxInfo.length > SM_HEADER) {
	if(sm_aes_enabled)
	{
          aes_decrypt(sm_rfRxInfo.pPayload, sm_rfRxInfo.length );
	  if(sm_rfRxInfo.pPayload[sm_rfRxInfo.length-1]!= 0xCA ||
	     sm_rfRxInfo.pPayload[sm_rfRxInfo.length-2]!= 0xFE ||
	     sm_rfRxInfo.pPayload[sm_rfRxInfo.length-3]!= 0xBE ||
	     sm_rfRxInfo.pPayload[sm_rfRxInfo.length-4]!= 0xEF ) {
  #ifdef GPIO_DEBUG
    nrk_gpio_clr(NRK_MISO);
  #endif
			return NRK_ERROR; 
			}
	 }
          //nrk_event_signal (sm_rx_pkt_signal);
        }
      } else return NRK_ERROR;


  // Take the raw buffer if everything is good and convert to pkt struct
  v=sm_buf_to_pkt( &sm_rfRxInfo.pPayload[0], p);

  #ifdef GPIO_DEBUG
    nrk_gpio_clr(NRK_MISO);
  #endif

  // This last return value means that the packet was good and it was
  // unpacked correctly.
  return v;
}


int8_t sm_tx_raw_pkt (sm_pkt_t *p)
{
  int8_t v;
  uint8_t checksum, i;
  uint8_t *data_start, *frame_start = &TRXFBST;
 
  // Copy sm_tx_pkt into sm_rfTxInfo payload
  v=sm_pkt_to_buf( p, sm_rfTxInfo.pPayload);
  sm_rfTxInfo.length=v;

if(sm_aes_enabled)
{
   // Add 0xCAFEBEEF as a magic number for AES MAC
   sm_rfTxInfo.length=sm_rfTxInfo.length+4;

   // Make packet a multiple of 16
   if((sm_rfTxInfo.length%16)!=0) sm_rfTxInfo.length=((sm_rfTxInfo.length/16)+1)*16;
   
//   printf( "l2: %d\r\n",sm_rfTxInfo.length );
   sm_rfTxInfo.pPayload[sm_rfTxInfo.length-1]=0xCA;
   sm_rfTxInfo.pPayload[sm_rfTxInfo.length-2]=0xFE;
   sm_rfTxInfo.pPayload[sm_rfTxInfo.length-3]=0xBE;
   sm_rfTxInfo.pPayload[sm_rfTxInfo.length-4]=0xEF;
}
//  checksum=0;
//  for(i=0; i<sm_rfTxInfo.length; i++ )
//	checksum+=sm_rfTxInfo.pPayload[i];

if(sm_aes_enabled)   aes_encrypt(sm_rfTxInfo.pPayload, sm_rfTxInfo.length );
  

//  data_start = frame_start + 9 + 1 + sm_rfTxInfo.length;
//  memcpy(data_start, &checksum, sizeof(uint8_t));

  #ifdef GPIO_DEBUG
    nrk_gpio_set(NRK_MOSI);
  #endif

for(i=0; i<p->mac_contention_control; i++ )
{
  v = rf_tx_packet (&sm_rfTxInfo);
  // Too delay or not?
}

  sm_app_tx_data_ready = 0;
  #ifdef GPIO_DEBUG
    nrk_gpio_clr(NRK_MOSI);
  #endif
  return NRK_OK;
}


void sm_task_config ()
{
  nrk_task_set_entry_function (&sm_task, sm_nw_task);
  nrk_task_set_stk (&sm_task, sm_task_stack, SM_STACKSIZE);
  sm_task.prio = SM_TASK_PRIORITY;
  sm_task.FirstActivation = TRUE;
  sm_task.Type = BASIC_TASK;
  sm_task.SchType = PREEMPTIVE;
  sm_task.period.secs = 1;
  sm_task.period.nano_secs = 0;       // Example: 20 * NANOS_PER_MS;
  sm_task.cpu_reserve.secs = 5;       // FIX reserve value... 
  sm_task.cpu_reserve.nano_secs = 0;
  sm_task.offset.secs = 0;
  sm_task.offset.nano_secs = 0;
  nrk_activate_task (&sm_task);
}
