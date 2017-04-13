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
#include <pcf_tdma.h>
#include <nrk_cfg.h>

//#define GPIO_DEBUG
#define TX_PKT_RETRY	1


#ifndef PCF_TDMA_TIMEOUT 
#define PCF_TDMA_TIMEOUT 7 
#endif

#ifndef TDMA_STACKSIZE
#define TDMA_STACKSIZE 256 
#endif
static uint8_t tdma_wakeup_flag=0;
static nrk_task_type tdma_task;
static NRK_STK tdma_task_stack[TDMA_STACKSIZE];
static uint8_t tdma_tx_buf[TDMA_MAX_PKT_SIZE];
static uint8_t tdma_rx_buf[TDMA_MAX_PKT_SIZE];

static uint16_t tdma_tx_sched[TDMA_MAX_TX_SLOTS];
static uint8_t tdma_tx_slots;
static uint8_t _tdma_aes_enabled=0;
static uint8_t tdma_my_mac;
static uint8_t tdma_mode;
static uint8_t tdma_ttl;
static uint8_t tx_data_ready;

//#define DEBUG
static uint16_t tdma_slot_len_ms;
static uint16_t tdma_slots_per_cycle;
static uint16_t tdma_last_tx_slot;

static uint8_t tdma_tx_data_ready;
static uint8_t tdma_rx_buf_empty;
static volatile uint8_t tdma_running;
static uint8_t tdma_chan;
static uint8_t tdma_is_enabled;
static uint16_t tdma_failure_cnt;

static uint8_t ttl_delay;
static nrk_time_t _ttl_delay_total;
static nrk_time_t _ttl_delay_per_tx;

static nrk_time_t _tdma_slot_time;
static nrk_time_t _tdma_next_wakeup;
static nrk_time_t tmp_time;

static int8_t tx_reserve;
static uint16_t tdma_rx_failure_cnt;

static uint8_t sync_status;
static int8_t (*tdma_error_callback)(uint16_t cons_err_cnt);


/**
 *  This is a callback if you require immediate response to a packet
 */
RF_RX_INFO *rf_rx_callback (RF_RX_INFO * pRRI)
{
  // Any code here gets called the instant a packet is received from the interrupt   
  return pRRI;
}

void tdma_aes_setkey(uint8_t *key)
{
uint8_t i;
	aes_setkey(key);
}


void tdma_aes_enable()
{
  _tdma_aes_enabled=1;
}

void tdma_aes_disable()
{
  _tdma_aes_enabled=1;
}

uint8_t tdma_sync_ok()
{
return sync_status;
}

int8_t tdma_set_error_callback(void (*fp)(void))
{
	if(fp==NULL ) return NRK_ERROR;
	tdma_error_callback=fp;
return NRK_OK;
}


int8_t tdma_tx_slot_add (uint16_t slot)
{
  tdma_tx_sched[0] = slot;
  tdma_tx_slots = 1;
  return NRK_OK;
}

int8_t tdma_tx_reserve_set (nrk_time_t * period, uint16_t pkts)
{

#ifdef NRK_MAX_RESERVES
// Create a reserve if it doesn't exist
  if (tx_reserve == -1)
    tx_reserve = nrk_reserve_create ();
  if (tx_reserve >= 0)
    return nrk_reserve_set (tx_reserve, period, pkts, NULL);
  else
    return NRK_ERROR;
#else
  return NRK_ERROR;
#endif
}

uint16_t tdma_tx_reserve_get ()
{
#ifdef NRK_MAX_RESERVES
  if (tx_reserve >= 0)
    return nrk_reserve_get (tx_reserve);
  else
    return 0;
#else
  return 0;
#endif
}

int8_t tdma_set_rf_power (uint8_t power)
{
  if (power > 31)
    return NRK_ERROR;
  rf_tx_power (power);
  return NRK_OK;
}

int8_t tdma_set_channel (uint8_t chan)
{
  if (chan > 26)
    return NRK_ERROR;
  tdma_chan = chan;
//rf_init (&tdma_rfRxInfo, chan, 0xFFFF, 0x00000);
  rf_init (&tdma_rfRxInfo, chan, 0x2420, 0x1214);
  return NRK_OK;
}

int8_t tdma_set_slot_len_ms (uint16_t len)
{
  tdma_slot_len_ms = len;
  _tdma_slot_time.nano_secs = len * NANOS_PER_MS;
  _tdma_slot_time.secs = 0;
  return NRK_OK;
}


int8_t tdma_set_slots_per_cycle (uint16_t slots_per_cycle)
{

  tdma_slots_per_cycle = slots_per_cycle;
}

int8_t tdma_send (tdma_info * fd, uint8_t * buf, uint8_t len, uint8_t flags)
{
  uint32_t mask;
  uint8_t i;

  if (tx_data_ready == 1)
    return NRK_ERROR;
  if (len == 0)
    return NRK_ERROR;
  if (buf == NULL)
    return NRK_ERROR;
  if (fd == NULL)
    return NRK_ERROR;

// If reserve exists check it
#ifdef NRK_MAX_RESERVES
  if (tx_reserve != -1) {
    if (nrk_reserve_consume (tx_reserve) == NRK_ERROR) {
      return NRK_ERROR;
    }
  }
#endif
  if (flags == TDMA_BLOCKING)
    nrk_signal_register (tdma_tx_pkt_done_signal);

  tx_data_ready = 1;

  tdma_rfTxInfo.pPayload = tdma_tx_buf;
// Setup the header data
  tdma_rfTxInfo.pPayload[TDMA_SLOT_HIGH] = (fd->slot >> 8) & 0xff;
  tdma_rfTxInfo.pPayload[TDMA_SLOT_LOW] = (fd->slot & 0xff);
  tdma_rfTxInfo.pPayload[TDMA_DST_HIGH] = (fd->dst >> 8) & 0xff;
  tdma_rfTxInfo.pPayload[TDMA_DST_LOW] = (fd->dst & 0xff);
//  tdma_rfTxInfo.pPayload[TDMA_SRC_HIGH] = (fd->src >> 8) & 0xff;
  tdma_rfTxInfo.pPayload[TDMA_SRC_HIGH] = (tdma_my_mac & 0xff);
  tdma_rfTxInfo.pPayload[TDMA_SRC_LOW] = (tdma_my_mac >> 8) & 0xff;
  fd->seq_num++;
  tdma_rfTxInfo.pPayload[TDMA_SEQ_NUM_HIGH] = ((fd->seq_num>>8) & 0xff);
  tdma_rfTxInfo.pPayload[TDMA_SEQ_NUM_LOW] = (fd->seq_num & 0xff);
  tdma_rfTxInfo.pPayload[TDMA_CYCLE_SIZE_HIGH] = (fd->cycle_size >> 8) & 0xff;
  tdma_rfTxInfo.pPayload[TDMA_CYCLE_SIZE_LOW] = (fd->cycle_size & 0xff);
  tdma_rfTxInfo.pPayload[TDMA_TTL] = tdma_ttl<<4 | tdma_ttl;

// Copy the user payload to the back of the header
  for (i = 0; i < len; i++)
    tdma_rfTxInfo.pPayload[i + TDMA_PCF_HEADER] = buf[i];
// Set packet length with header
  tdma_rfTxInfo.length = len + TDMA_PCF_HEADER;
#ifdef DEBUG
  nrk_kprintf (PSTR ("Waiting for tx done signal\r\n"));
#endif

  if (flags == TDMA_BLOCKING) {
    mask = nrk_event_wait (SIG (tdma_tx_pkt_done_signal));
    if (mask == 0)
      nrk_kprintf (PSTR ("TDMA TX: Error calling event wait\r\n"));
    if ((mask & SIG (tdma_tx_pkt_done_signal)) == 0)
      nrk_kprintf (PSTR ("TDMA TX: Woke up on wrong signal\r\n"));
    return NRK_OK;
  }

  return NRK_OK;
}

int8_t tdma_recv (tdma_info * fd, uint8_t * buf, uint8_t * len, uint8_t flags)
{
  nrk_sig_mask_t event;
  uint8_t i;
  if (flags == TDMA_BLOCKING) {
    if (tdma_rx_buf_empty == 1) {
      nrk_signal_register (tdma_rx_pkt_signal);
      event = nrk_event_wait (SIG (tdma_rx_pkt_signal));
    }
  }
  else if (tdma_rx_buf_empty == 1)
    return NRK_ERROR;

  if (tdma_rfRxInfo.length < TDMA_PCF_HEADER)
    return NRK_ERROR;
  // Set the length
  *len = (uint8_t) (tdma_rfRxInfo.length - TDMA_PCF_HEADER);
  // Copy the payload data
  for (i = 0; i < *len; i++)
    buf[i] = tdma_rfRxInfo.pPayload[i + TDMA_PCF_HEADER];

  // Fill the information struct
  fd->rssi = tdma_rfRxInfo.rssi;
  fd->actualRssi = tdma_rfRxInfo.actualRssi;
  fd->energyDetectionLevel = tdma_rfRxInfo.energyDetectionLevel;
  fd->linkQualityIndication = tdma_rfRxInfo.linkQualityIndication;
  fd->src =
    ((uint16_t) tdma_rfRxInfo.pPayload[TDMA_SRC_HIGH] << 8) | tdma_rfRxInfo.
    pPayload[TDMA_SRC_LOW];
  fd->dst =
    ((uint16_t) tdma_rfRxInfo.pPayload[TDMA_DST_HIGH] << 8) | tdma_rfRxInfo.
    pPayload[TDMA_DST_LOW];
  fd->slot =
    ((uint16_t) tdma_rfRxInfo.pPayload[TDMA_SLOT_HIGH] << 8) | tdma_rfRxInfo.
    pPayload[TDMA_SLOT_LOW];
  fd->seq_num =
    ((uint16_t) tdma_rfRxInfo.
     pPayload[TDMA_SEQ_NUM_HIGH] << 8) | tdma_rfRxInfo.
    pPayload[TDMA_SEQ_NUM_LOW];
  fd->cycle_size =
    ((uint16_t) tdma_rfRxInfo.
     pPayload[TDMA_CYCLE_SIZE_HIGH] << 8) | tdma_rfRxInfo.
    pPayload[TDMA_CYCLE_SIZE_LOW];

  fd->ttl= (uint8_t) tdma_rfRxInfo.pPayload[TDMA_TTL]; 


  // Check if it was a time out instead of packet RX signal
  if (flags == TDMA_BLOCKING)
    if ((event & SIG (tdma_rx_pkt_signal)) == 0)
      return NRK_ERROR;

  // Set the buffer as empty
  tdma_rx_buf_empty = 1;
  return NRK_OK;

}


int8_t tdma_rx_pkt_set_buffer (uint8_t * buf, uint8_t size)
{
  if (buf == NULL)
    return NRK_ERROR;
  tdma_rfRxInfo.pPayload = buf;
  tdma_rfRxInfo.max_length = size;
  tdma_rx_buf_empty = 1;
  return NRK_OK;
}

void tdma_ttl_set(uint8_t ttl)
{
tdma_ttl=ttl;
}

int8_t tdma_init (uint8_t mode, uint8_t chan, uint16_t my_mac)
{
  tx_reserve = -1;
  tdma_rx_failure_cnt = 0;
  tdma_mode = mode;
  tdma_tx_slots = 0;
  tdma_ttl=TDMA_DEFAULT_TTL;
  sync_status=0;


  #ifdef GPIO_DEBUG
    nrk_gpio_direction(NRK_MOSI,NRK_PIN_OUTPUT);
    nrk_gpio_direction(NRK_MISO,NRK_PIN_OUTPUT);
  #endif


  tdma_slots_per_cycle = TDMA_DEFAULT_SLOTS_PER_CYCLE;

  _tdma_slot_time.nano_secs = TDMA_DEFAULT_SLOT_MS * NANOS_PER_MS;
  _tdma_slot_time.secs = 0;

  tdma_rx_pkt_signal = nrk_signal_create ();
  if (tdma_rx_pkt_signal == NRK_ERROR) {
    nrk_kprintf (PSTR ("TDMA ERROR: creating rx signal failed\r\n"));
    nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
    return NRK_ERROR;
  }
  tdma_tx_pkt_done_signal = nrk_signal_create ();
  if (tdma_tx_pkt_done_signal == NRK_ERROR) {
    nrk_kprintf (PSTR ("TDMA ERROR: creating tx signal failed\r\n"));
    nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
    return NRK_ERROR;
  }
  tdma_enable_signal = nrk_signal_create ();
  if (tdma_enable_signal == NRK_ERROR) {
    nrk_kprintf (PSTR ("TDMA ERROR: creating enable signal failed\r\n"));
    nrk_kernel_error_add (NRK_SIGNAL_CREATE_ERROR, nrk_cur_task_TCB->task_ID);
    return NRK_ERROR;
  }


  // Set the one main rx buffer
  tdma_rx_pkt_set_buffer (tdma_rx_buf, TDMA_MAX_PKT_SIZE);
  tdma_rx_buf_empty = 1;
  tx_data_ready = 0;


  // Setup the radio 
  rf_init (&tdma_rfRxInfo, chan, 0xffff, my_mac);
  tdma_chan = chan;
  tdma_my_mac = my_mac;

  //FASTSPI_SETREG (CC2420_RSSI, 0xE580); // CCA THR=-25
  //FASTSPI_SETREG (CC2420_TXCTRL, 0x80FF);       // TX TURNAROUND = 128 us
  //FASTSPI_SETREG (CC2420_RXCTRL1, 0x0A56);
  // default cca thresh of -45
  rf_set_cca_thresh (-45);

  asm volatile ("":::"memory");
  tdma_running = 1;
  tdma_is_enabled = 1;
  return NRK_OK;
}


nrk_sig_t tdma_get_rx_pkt_signal ()
{
  nrk_signal_register (tdma_rx_pkt_signal);
  return (tdma_rx_pkt_signal);
}

nrk_sig_t tdma_get_tx_done_signal ()
{
  nrk_signal_register (tdma_tx_pkt_done_signal);
  return (tdma_tx_pkt_done_signal);
}



uint8_t *tdma_rx_pkt_get (uint8_t * len, int8_t * rssi)
{

  if (tdma_rx_buf_empty == 1) {
    *len = 0;
    *rssi = 0;
    return NULL;
  }
  *len = tdma_rfRxInfo.length;
  *rssi = tdma_rfRxInfo.rssi;
  return tdma_rfRxInfo.pPayload;
}


int8_t tdma_rx_pkt_release(void)
{
    tdma_rx_buf_empty=1;
return NRK_OK;
}


void tdma_disable ()
{
  tdma_is_enabled = 0;
  rf_power_down ();
}

void tdma_enable ()
{
  tdma_is_enabled = 1;
  rf_power_up ();
  nrk_event_signal (tdma_enable_signal);
}


void tdma_nw_task ()
{
  int8_t v, i;
  uint16_t slot, tmp,tmp2,sync;
  nrk_sig_mask_t event;

  do {
    nrk_wait_until_next_period ();
  } while (!tdma_started ());

//register the signal after bmac_init has been called
  v = nrk_signal_register (tdma_enable_signal);
  if (v == NRK_ERROR)
    nrk_kprintf (PSTR ("Failed to register signal\r\n"));
  slot = 0;
//rf_set_rx (&tdma_rfRxInfo, tdma_chan);



  while (1) {
    if (tdma_mode == TDMA_HOST) {
	    sync_status=1; // HOST is always synced
      // This is the downstream transmit slot
      if (slot == 0) {

        //  for(i=0; i<100; i++ ) tdma_rfTxInfo.pPayload[i] = 0;  
        //rf_rx_off();
        // If there is no pending packet, lets make an empty one
        if (tx_data_ready == 0) {
          tdma_rfTxInfo.pPayload = tdma_tx_buf;
          // Setup the header data
          tdma_rfTxInfo.pPayload[TDMA_DST_LOW] = 0xff;  // dst
          tdma_rfTxInfo.pPayload[TDMA_DST_HIGH] = 0xff;
          tdma_rfTxInfo.pPayload[TDMA_SRC_LOW] = 0x00;  // src
          tdma_rfTxInfo.pPayload[TDMA_SRC_HIGH] = 0x00;
          tdma_rfTxInfo.pPayload[TDMA_SEQ_NUM_LOW] = 0x00;      // seq num
          tdma_rfTxInfo.pPayload[TDMA_SEQ_NUM_HIGH] = 0x00;
          tdma_rfTxInfo.length = TDMA_PCF_HEADER;
        }
        tdma_rfTxInfo.pPayload[TDMA_CYCLE_SIZE_LOW] = tdma_slots_per_cycle & 0xff;      // cycle size 
        tdma_rfTxInfo.pPayload[TDMA_CYCLE_SIZE_HIGH] =
          tdma_slots_per_cycle >> 8;
        tdma_rfTxInfo.pPayload[TDMA_SLOT_LOW] = 0;      // slot
        tdma_rfTxInfo.pPayload[TDMA_SLOT_HIGH] = 0;
        tdma_rfTxInfo.pPayload[TDMA_TTL] = tdma_ttl<<4 | (tdma_ttl);
        tdma_rfTxInfo.pPayload[TDMA_SLOT_SIZE] = tdma_slot_len_ms;
        nrk_time_get (&_tdma_next_wakeup);
        tdma_rfTxInfo.destAddr = 0xffff;
        tdma_rfTxInfo.ackRequest = 0;
        tdma_rfTxInfo.cca = 0;
        _tdma_tx ();
        _tdma_tx ();
        rf_rx_on ();
	if(tdma_wakeup_flag==1)
		{
		for(slot=0; slot<1000; slot++ )
			{
				if((slot%5)==0)nrk_led_toggle(BLUE_LED);
        			tdma_rfTxInfo.pPayload[TDMA_SLOT_LOW] = 0xff;      // slot
        			tdma_rfTxInfo.pPayload[TDMA_SLOT_HIGH] = 0xff;
          			tdma_rfTxInfo.pPayload[TDMA_SRC_LOW] = 0x0;  // src
          			tdma_rfTxInfo.pPayload[TDMA_SRC_HIGH] = 0x0;
          			tdma_rfTxInfo.length = TDMA_PCF_HEADER;
        			_tdma_tx ();
        			//rf_rx_on ();
				nrk_wait_until_ticks(10);
			}
		nrk_led_clr(BLUE_LED);
		slot=0;
		tdma_wakeup_flag=0;
		}
      }
      else {
        // Upstream data slot
        // This configures the packet receive interrupt to call the _tdma_rx_master function
        // The _tdma_rx_master function triggers a signal to the tdma_rx function.
        // rf_rx_on();
	// rx_end_callback(&_tdma_rx_master);
        _tdma_rx_master();
        if (v == 1)
          tdma_last_tx_slot = slot;
      }
      slot++;
	// For the last slot, we wakeup a bit late to make sure all other nodes have woken up in time to
	// receive the beacon message.  
      if (slot > tdma_slots_per_cycle ){
        nrk_wait_until_ticks(TDMA_WAKEUP_GAURD_TIME_MS);
	slot = 0;
	}
      else {
      nrk_time_add (&_tdma_next_wakeup, _tdma_next_wakeup, _tdma_slot_time);
      nrk_time_compact_nanos (&_tdma_next_wakeup);
      nrk_wait_until (_tdma_next_wakeup);
	
      }
    }
    // TDMA slave node
    else {
      if (slot == 0) {

	//rf_rx_off ();
	//rf_power_down();
	      	
	sync=0;
	rf_power_up();
	rf_rx_on ();
        do {
          v = _tdma_rx ();
          nrk_time_get (&_tdma_next_wakeup);
          if (v == NRK_OK) {

	    // See if its part of a wakeup packet stream
            tmp =
              (tdma_rfRxInfo.pPayload[TDMA_SRC_HIGH] << 8) | tdma_rfRxInfo.
              pPayload[TDMA_SRC_LOW];
	    // Check if its slot 0 packet
            tmp2 =
              (tdma_rfRxInfo.pPayload[TDMA_SLOT_HIGH] << 8) | tdma_rfRxInfo.
              pPayload[TDMA_SLOT_LOW];

	    if(tmp==0x0 && tmp2==0xffff) sync=0;

            if (tmp2 != 0) {
              v = NRK_ERROR;
          	tdma_rx_buf_empty = 1;
	      // agr remove next 3 lines?
		//rf_rx_off();
		//rf_power_down();
	      	//nrk_wait_until_next_period();
	      	//rf_rx_on();
	    }
	    if(v!=NRK_ERROR)
	    {
	    // set TTL to TTL_MAX from gateway
	    tdma_ttl = tdma_rfRxInfo.pPayload[TDMA_TTL]>>4;
            tdma_slots_per_cycle =
              (tdma_rfRxInfo.
               pPayload[TDMA_CYCLE_SIZE_HIGH] << 8) | tdma_rfRxInfo.
              pPayload[TDMA_CYCLE_SIZE_LOW];
            tdma_slot_len_ms = tdma_rfRxInfo.pPayload[TDMA_SLOT_SIZE];
            _tdma_slot_time.nano_secs = tdma_slot_len_ms * NANOS_PER_MS;
            _tdma_slot_time.secs = 0;
	    }
          }
	  // Make this time based so that it doesn't get shorter with non sync packets
	  if(sync>=100) { 
		  		sync_status=0; /*nrk_led_set(RED_LED);*/ 
				//sync=30000; 
				if(tdma_error_callback!=NULL ) i=tdma_error_callback(sync-100);
		if(i==NRK_OK) { tdma_rx_buf_empty=1; /*nrk_kprintf( PSTR("sync=0\r\n"));*/ sync=0; }
	  } 
	  if(sync<30000 ) sync++;
	tdma_rx_pkt_release();
        } while (v != NRK_OK);
      //if(sync==30000)  sync_status=1; /*nrk_led_clr(RED_LED);*/ 
      }

      // Compute TTL offset
      ttl_delay=(tdma_rfRxInfo.pPayload[TDMA_TTL]>>4)-(tdma_rfRxInfo.pPayload[TDMA_TTL]&0xf);
      if(ttl_delay>16) ttl_delay=0;
      _ttl_delay_total.secs=0;
      _ttl_delay_total.nano_secs=0;
      _ttl_delay_per_tx.secs=0;
      _ttl_delay_per_tx.nano_secs=0;

      _ttl_delay_per_tx.secs=0;
      _ttl_delay_per_tx.nano_secs=((uint32_t)tdma_rfRxInfo.length+(uint32_t)17)*32000;   // (payload_bytes + header) * 32 us

      for (tmp = 0; tmp < ttl_delay; tmp++)
        nrk_time_add (&_ttl_delay_total, _ttl_delay_total, _ttl_delay_per_tx);


      sync_status=1;
	rf_rx_off ();
	rf_power_down();

      // Find next slot
      slot = tdma_tx_sched[0];
      if (slot > tdma_slots_per_cycle)
        slot = tdma_slots_per_cycle;
 //         nrk_time_get (&tmp_time);
//      printf( "tt=%lu %lu\r\n",tmp_time.secs,tmp_time.nano_secs/NANOS_PER_MS);
//      printf( "nw=%lu %lu\r\n",_tdma_next_wakeup.secs,_tdma_next_wakeup.nano_secs/NANOS_PER_MS);
      for (tmp = 0; tmp < slot; tmp++)
        nrk_time_add (&_tdma_next_wakeup, _tdma_next_wakeup, _tdma_slot_time);
      // Subtract TTL delay time
      nrk_time_sub(&_tdma_next_wakeup, _tdma_next_wakeup, _ttl_delay_total);

      nrk_time_compact_nanos (&_tdma_next_wakeup);

  //    printf( "nw2=%lu %lu\r\n",_tdma_next_wakeup.secs,_tdma_next_wakeup.nano_secs/NANOS_PER_MS);
      nrk_wait_until (_tdma_next_wakeup);

      // Transmit on slot
      if (tx_data_ready == 1) {
        tdma_rfTxInfo.pPayload[TDMA_SLOT_LOW] = slot & 0xff;    // slot
        tdma_rfTxInfo.pPayload[TDMA_SLOT_HIGH] = slot >> 8;
        tdma_rfTxInfo.destAddr = 0xffff;
        tdma_rfTxInfo.ackRequest = 0;
        tdma_rfTxInfo.cca = 0;
	rf_power_up();
        _tdma_tx ();
      }

      // Sleep until end of cycle 
	rf_rx_off ();
	rf_power_down();
      for (tmp = 0; tmp < ((uint16_t)tdma_slots_per_cycle - (uint16_t)slot); tmp++)
        nrk_time_add (&_tdma_next_wakeup, _tdma_next_wakeup, _tdma_slot_time);
      nrk_time_compact_nanos (&_tdma_next_wakeup);
      nrk_wait_until (_tdma_next_wakeup);
      slot = 0;
      while(!tdma_is_enabled) nrk_wait_until_next_period();
		
      slot=0;
    }
  }

}

void tdma_wakeup()
{
tdma_wakeup_flag=1;

}

int8_t tdma_started ()
{
  return tdma_running;
}


int8_t _tdma_rx_master ()
{
  int8_t v, i;
  static uint8_t cnt=0;
  v = 0;
  
  if (tdma_rx_buf_empty != 1) {
    rf_rx_off();
    rf_rx_on();
    cnt++;
    // catch annoying race condition, FIXME: why does this really happen?
    if(cnt>2){ tdma_rx_buf_empty=0;
          nrk_event_signal (tdma_rx_pkt_signal);
	}
    return NRK_ERROR;
    }
cnt=0;
  #ifdef GPIO_DEBUG
    nrk_gpio_set(NRK_MISO);
  #endif
//  if (rf_rx_check_fifop () == 1) {
  //    v = rf_polling_rx_packet ();
  v = rf_rx_packet_nonblock ();
      if (v == 1) {
        // Grab packet, do good stuff
        if (tdma_rfRxInfo.length > TDMA_PCF_HEADER) {
	if(_tdma_aes_enabled)
	{
          aes_decrypt(tdma_rfRxInfo.pPayload, tdma_rfRxInfo.length );
	  if(tdma_rfRxInfo.pPayload[tdma_rfRxInfo.length-1]!= 0xCA ||
	     tdma_rfRxInfo.pPayload[tdma_rfRxInfo.length-2]!= 0xFE ||
	     tdma_rfRxInfo.pPayload[tdma_rfRxInfo.length-3]!= 0xBE ||
	     tdma_rfRxInfo.pPayload[tdma_rfRxInfo.length-4]!= 0xEF ) 
		{
  		#ifdef GPIO_DEBUG
   		 	nrk_gpio_clr(NRK_MISO);
  		#endif
			return NRK_ERROR; 
		}
	 }
	  tdma_rx_buf_empty = 0;
          nrk_event_signal (tdma_rx_pkt_signal);
        }
      }

  #ifdef GPIO_DEBUG
    nrk_gpio_clr(NRK_MISO);
  #endif

  return v;
}


int8_t _tdma_rx ()
{
  int8_t v, i;
  v = 0;

  if (tdma_rx_buf_empty != 1)
    return NRK_ERROR;

  #ifdef GPIO_DEBUG
    nrk_gpio_set(NRK_MISO);
  #endif
//  if (rf_rx_check_fifop () == 1) {
    for (i = 0; i < 100; i++) {
  //    v = rf_polling_rx_packet ();
  v = rf_rx_packet_nonblock ();
      if (v == 1) {
        // Grab packet, do good stuff
        if (tdma_rfRxInfo.length > TDMA_PCF_HEADER) {
	if(_tdma_aes_enabled)
	{
          aes_decrypt(tdma_rfRxInfo.pPayload, tdma_rfRxInfo.length );
	  if(tdma_rfRxInfo.pPayload[tdma_rfRxInfo.length-1]!= 0xCA ||
	     tdma_rfRxInfo.pPayload[tdma_rfRxInfo.length-2]!= 0xFE ||
	     tdma_rfRxInfo.pPayload[tdma_rfRxInfo.length-3]!= 0xBE ||
	     tdma_rfRxInfo.pPayload[tdma_rfRxInfo.length-4]!= 0xEF ) {
  #ifdef GPIO_DEBUG
    nrk_gpio_clr(NRK_MISO);
  #endif
			return NRK_ERROR; 
			}
	 }
	  tdma_rx_buf_empty = 0;
          nrk_event_signal (tdma_rx_pkt_signal);
        }
        break;
      }
      nrk_spin_wait_us (100);
    }

//  }

  #ifdef GPIO_DEBUG
    nrk_gpio_clr(NRK_MISO);
  #endif
  return v;
}


int8_t _tdma_tx ()
{
  int8_t v;
  uint8_t checksum, i;
  uint8_t *data_start, *frame_start = &TRXFBST;
 

if(_tdma_aes_enabled)
{
   // Add 0xCAFEBEEF as a magic number for AES MAC
   tdma_rfTxInfo.length=tdma_rfTxInfo.length+4;

   // Make packet a multiple of 16
   if((tdma_rfTxInfo.length%16)!=0) tdma_rfTxInfo.length=((tdma_rfTxInfo.length/16)+1)*16;
   
//   printf( "l2: %d\r\n",tdma_rfTxInfo.length );
   tdma_rfTxInfo.pPayload[tdma_rfTxInfo.length-1]=0xCA;
   tdma_rfTxInfo.pPayload[tdma_rfTxInfo.length-2]=0xFE;
   tdma_rfTxInfo.pPayload[tdma_rfTxInfo.length-3]=0xBE;
   tdma_rfTxInfo.pPayload[tdma_rfTxInfo.length-4]=0xEF;
}
  checksum=0;
  for(i=0; i<tdma_rfTxInfo.length; i++ )
	checksum+=tdma_rfTxInfo.pPayload[i];

if(_tdma_aes_enabled)   aes_encrypt(tdma_rfTxInfo.pPayload, tdma_rfTxInfo.length );
  

  data_start = frame_start + 9 + 1 + tdma_rfTxInfo.length;
  memcpy(data_start, &checksum, sizeof(uint8_t));

  #ifdef GPIO_DEBUG
    nrk_gpio_set(NRK_MOSI);
  #endif

for(i=0; i<TX_PKT_RETRY; i++ )
{
  v = rf_tx_packet (&tdma_rfTxInfo);
  // Too delay or not?
}

  tx_data_ready = 0;
  nrk_event_signal (tdma_tx_pkt_done_signal);
  #ifdef GPIO_DEBUG
    nrk_gpio_clr(NRK_MOSI);
  #endif
  return NRK_OK;
}


void tdma_task_config ()
{
  nrk_task_set_entry_function (&tdma_task, tdma_nw_task);
  nrk_task_set_stk (&tdma_task, tdma_task_stack, TDMA_STACKSIZE);
  tdma_task.prio = TDMA_TASK_PRIORITY;
  tdma_task.FirstActivation = TRUE;
  tdma_task.Type = BASIC_TASK;
  tdma_task.SchType = PREEMPTIVE;
  tdma_task.period.secs = 1;
  tdma_task.period.nano_secs = 0; // 20 * NANOS_PER_MS;
  tdma_task.cpu_reserve.secs = PCF_TDMA_TIMEOUT;       // bmac reserve , 0 to disable
  tdma_task.cpu_reserve.nano_secs = 0;
  tdma_task.offset.secs = 0;
  tdma_task.offset.nano_secs = 0;
  nrk_activate_task (&tdma_task);
}
