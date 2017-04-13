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
#ifndef _SPARKMAC_H
#define _SPARKMAC_H
#include <include.h>
#include <basic_rf.h>
#include <nrk.h>
#include <sparkmac_pkt.h>

/************************************************************************
Describe protocol here.

************************************************************************/

#define SM_MAX_PKT_SIZE		128	
#define SM_MAX_PAYLOAD_SIZE	(SM_MAX_PKT_SIZE-SM_HEADER)	
#define SM_DEFAULT_TTL		3

#ifndef SM_TASK_PRIORITY
#define SM_TASK_PRIORITY		20
#endif

#define SM_GATEWAY	1
#define SM_EDGE_DEVICE	2
#define SM_ROUTERS	3

#define SM_BLOCKING		0
#define SM_NONBLOCKING		1



int8_t sm_tx_reserve_set (nrk_time_t * period, uint16_t pkts);
uint16_t sm_tx_reserve_get ();

// Signal sent by radio callback when a packet is received
nrk_sig_t sm_rx_pkt_signal;
// This signal is for forcing a TX
nrk_sig_t sm_tx_pkt_signal;
// This is the signal that tells the app tx command to finish
nrk_sig_t sm_tx_pkt_done_signal;
// This is the signal that tells the app tx command to finish
nrk_sig_t sm_rx_pkt_ready_signal;


RF_RX_INFO sm_rfRxInfo;
RF_TX_INFO sm_rfTxInfo;

void _rx_handler(void);

void sm_disable ();
void sm_aes_enable();
void sm_aes_disable ();

void sm_ttl_set_default(uint8_t ttl);
void sm_task_config ();
int8_t sm_set_channel (uint8_t chan);
int8_t sm_send (uint8_t *buf, uint8_t len);
int8_t sm_recv (uint8_t *buf);


nrk_sig_t sm_get_tx_done_signal ();
nrk_sig_t sm_get_rx_pkt_signal ();

int8_t sm_started ();
int8_t sm_init (uint8_t sm_mode, uint8_t chan, uint16_t my_mac);

int8_t sm_rx_raw_pkt (sm_pkt_t *p);
int8_t sm_tx_raw_pkt (sm_pkt_t *p);

#endif
