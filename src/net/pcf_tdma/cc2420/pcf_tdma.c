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

#ifndef PCF_TDMA_TIMEOUT 
#define PCF_TDMA_TIMEOUT 5 
#endif

#ifndef TDMA_STACKSIZE
#define TDMA_STACKSIZE 256
#endif
static nrk_task_type tdma_task;
static NRK_STK tdma_task_stack[TDMA_STACKSIZE];
static uint8_t tdma_tx_buf[TDMA_MAX_PKT_SIZE];
static uint8_t tdma_rx_buf[TDMA_MAX_PKT_SIZE];

static uint16_t tdma_tx_sched[TDMA_MAX_TX_SLOTS];
static uint8_t tdma_tx_slots;

static uint8_t tdma_my_mac;
static uint8_t tdma_mode;
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

static nrk_time_t _tdma_slot_time;
static nrk_time_t _tdma_next_wakeup;

static int8_t tx_reserve;
static uint16_t tdma_rx_failure_cnt;

static uint8_t sync_status;

static void (*tdma_error_callback)(void);



int8_t tdma_set_error_callback(void (*fp)(void))
{
	if(fp==NULL ) return NRK_ERROR;
	tdma_error_callback=fp;
return NRK_OK;
}

uint8_t tdma_sync_ok()
{
return sync_status;
}

/**
 *  This is a callback if you require immediate response to a packet
 */
RF_RX_INFO *rf_rx_callback (RF_RX_INFO * pRRI)
{
  // Any code here gets called the instant a packet is received from the interrupt   
  return pRRI;
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
  tdma_rfTxInfo.pPayload[TDMA_SRC_HIGH] = (fd->src >> 8) & 0xff;
  tdma_rfTxInfo.pPayload[TDMA_SRC_LOW] = (tdma_my_mac & 0xff);
  tdma_rfTxInfo.pPayload[TDMA_SEQ_NUM_HIGH] = (tdma_my_mac >> 8) & 0xff;
  tdma_rfTxInfo.pPayload[TDMA_SEQ_NUM_LOW] = (fd->seq_num & 0xff);
  tdma_rfTxInfo.pPayload[TDMA_CYCLE_SIZE_HIGH] = (fd->cycle_size >> 8) & 0xff;
  tdma_rfTxInfo.pPayload[TDMA_CYCLE_SIZE_LOW] = (fd->cycle_size & 0xff);
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

int8_t tdma_init (uint8_t mode, uint8_t chan, uint16_t my_mac)
{
  tx_reserve = -1;
  tdma_rx_failure_cnt = 0;
  tdma_mode = mode;
  tdma_tx_slots = 0;
  sync_status=0;

  tdma_slots_per_cycle = TDMA_DEFAULT_SLOTS_PER_CYCLE;


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


  // Setup the cc2420 chip
  rf_init (&tdma_rfRxInfo, chan, 0xffff, my_mac);
  tdma_chan = chan;
  tdma_my_mac = my_mac;

  FASTSPI_SETREG (CC2420_RSSI, 0xE580); // CCA THR=-25
  FASTSPI_SETREG (CC2420_TXCTRL, 0x80FF);       // TX TURNAROUND = 128 us
  FASTSPI_SETREG (CC2420_RXCTRL1, 0x0A56);
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
  uint16_t slot, tmp,sync;
  nrk_sig_mask_t event;

  do {
    nrk_wait_until_next_period ();
  } while (!tdma_started ());
  _tdma_slot_time.nano_secs = TDMA_DEFAULT_SLOT_MS * NANOS_PER_MS;
  _tdma_slot_time.secs = 0;

//register the signal after bmac_init has been called
  v = nrk_signal_register (tdma_enable_signal);
  if (v == NRK_ERROR)
    nrk_kprintf (PSTR ("Failed to register signal\r\n"));
  slot = 0;
//rf_set_rx (&tdma_rfRxInfo, tdma_chan);
  while (1) {
    if (tdma_mode == TDMA_HOST) {
      // This is the downstream transmit slot
      sync_status=1;
      if (slot == 0) {

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
        tdma_rfTxInfo.pPayload[TDMA_SLOT_SIZE] = tdma_slot_len_ms;
        nrk_time_get (&_tdma_next_wakeup);
        tdma_rfTxInfo.destAddr = 0xffff;
        tdma_rfTxInfo.ackRequest = 0;
        tdma_rfTxInfo.cca = 0;
        _tdma_tx ();
        rf_rx_on ();
      }
      else {
        // Upstream data slot
        v = _tdma_rx ();
        if (v == 1)
          tdma_last_tx_slot = slot;
      }
      slot++;
      if (slot >= tdma_slots_per_cycle + 1)
        slot = 0;
      nrk_time_add (&_tdma_next_wakeup, _tdma_next_wakeup, _tdma_slot_time);
      nrk_time_compact_nanos (&_tdma_next_wakeup);
      nrk_wait_until (_tdma_next_wakeup);

    }
    else {
      if (slot == 0) {

	sync=0;
	rf_rx_on ();
        do {
          v = _tdma_rx ();
          nrk_time_get (&_tdma_next_wakeup);
          if (v == NRK_OK) {
            tmp =
              (tdma_rfRxInfo.pPayload[TDMA_SLOT_HIGH] << 8) | tdma_rfRxInfo.
              pPayload[TDMA_SLOT_LOW];
            if (tmp != 0) {
              v = NRK_ERROR;
          	tdma_rx_buf_empty = 1;
	      //rf_rx_off();
	      //nrk_wait_until_next_period();
	      //rf_rx_on();
	    }
	    if(v!=NRK_ERROR)
	    {
            tdma_slots_per_cycle =
              (tdma_rfRxInfo.
               pPayload[TDMA_CYCLE_SIZE_HIGH] << 8) | tdma_rfRxInfo.
              pPayload[TDMA_CYCLE_SIZE_LOW];
            tdma_slot_len_ms = tdma_rfRxInfo.pPayload[TDMA_SLOT_SIZE];
            _tdma_slot_time.nano_secs = tdma_slot_len_ms * NANOS_PER_MS;
            _tdma_slot_time.secs = 0;
	    }
          }
	  sync++;
	  if(sync>=30000) { 
		  sync_status=0; /*nrk_led_set(RED_LED);*/ sync=30000; 
				if(tdma_error_callback!=NULL ) tdma_error_callback();
	  	}
        } while (v != NRK_OK);
      if(sync==30000) sync_status=1; /*nrk_led_clr(RED_LED);*/
      }

      rf_rx_off ();

      // Find next slot
      slot = tdma_tx_sched[0];
      if (slot > tdma_slots_per_cycle)
        slot = tdma_slots_per_cycle;
      for (tmp = 0; tmp < slot; tmp++)
        nrk_time_add (&_tdma_next_wakeup, _tdma_next_wakeup, _tdma_slot_time);
      nrk_time_compact_nanos (&_tdma_next_wakeup);
      nrk_wait_until (_tdma_next_wakeup);

      // Transmit on slot
      if (tx_data_ready == 1) {
        tdma_rfTxInfo.pPayload[TDMA_SLOT_LOW] = slot & 0xff;    // slot
        tdma_rfTxInfo.pPayload[TDMA_SLOT_HIGH] = slot >> 8;
        tdma_rfTxInfo.destAddr = 0xffff;
        tdma_rfTxInfo.ackRequest = 0;
        tdma_rfTxInfo.cca = 0;
        _tdma_tx ();
      }

      // Sleep until end of cycle 
      rf_rx_off ();
      for (tmp = 0; tmp < (tdma_slots_per_cycle - slot); tmp++)
        nrk_time_add (&_tdma_next_wakeup, _tdma_next_wakeup, _tdma_slot_time);
      nrk_time_compact_nanos (&_tdma_next_wakeup);
      nrk_wait_until (_tdma_next_wakeup);
      slot = 0;
    }
  }

}



int8_t tdma_started ()
{
  return tdma_running;
}


int8_t _tdma_rx ()
{
  int8_t v, i;
  v = 0;

  if (tdma_rx_buf_empty != 1)
    return NRK_ERROR;

  if (rf_rx_check_fifop () == 1) {
    for (i = 0; i < 40; i++) {
      v = rf_polling_rx_packet ();
      if (v == 1) {
        // Grab packet, do good stuff
        if (tdma_rfRxInfo.length > TDMA_PCF_HEADER) {
          tdma_rx_buf_empty = 0;
          nrk_event_signal (tdma_rx_pkt_signal);
        }
        break;
      }
      nrk_spin_wait_us (100);
    }

  }

  return v;
}


int8_t _tdma_tx ()
{
  int8_t v;

  v = rf_tx_packet (&tdma_rfTxInfo);
  tx_data_ready = 0;
  nrk_event_signal (tdma_tx_pkt_done_signal);
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
  tdma_task.period.secs = 0;
  tdma_task.period.nano_secs = 20 * NANOS_PER_MS;
  tdma_task.cpu_reserve.secs = PCF_TDMA_TIMEOUT;       // bmac reserve , 0 to disable
  tdma_task.cpu_reserve.nano_secs = 0;
  tdma_task.offset.secs = 0;
  tdma_task.offset.nano_secs = 0;
  nrk_activate_task (&tdma_task);
}
