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

//#define DEBUG-NRK

//#define DEBUG_ERROR
#define DEBUG_LOGIC

#ifdef RADIO_VERBOSE
#define vprintf(...)		printf(__VA_ARGS__)
#else
#define vprintf(...)
#endif

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
static uint8_t tdma_contention_slots;

static uint8_t tdma_rx_buf_empty;
static volatile uint8_t tdma_running;
static uint8_t tdma_chan;
static uint8_t tdma_is_enabled;

static nrk_time_t _tdma_slot_time;
static nrk_time_t _tdma_next_wakeup;

static int8_t tx_reserve;
static uint16_t tdma_rx_failure_cnt;

static uint8_t sync_status;
static void (*tdma_error_callback)(void);

static uint16_t host_slot;
static uint8_t tdma_level = 1;
static uint32_t tdma_cycle_len_ms;

static tdma_sync_node sync_node;
static uint16_t seq = 0;

/**
 * @brief Schedule table for clients
 * tdma_schedule_i - the index of the current schedule slot
 * tdma_schedule_length - the length of the current schedule
 */
static tdma_schedule_event tdma_schedule[2 * TDMA_MAX_FWD_CLIENTS
		+ TDMA_DEFAULT_CONTENSION_SLOTS + 2];
static uint8_t tdma_schedule_i = 0;
static uint8_t tdma_schedule_length = 0;

/**
 * @brief Number of children that a client node is in charge of
 */
static uint8_t tdma_client_num_children = 0;

/**
 * @brief Schedule table for masters
 * tdma_host_pending_requests - the pending requests to be processed by master
 * tdma_host_pending_request_i - the index of the next available slot for pending requests
 */
static tdma_host_schedule host_schedule[TDMA_DEFAULT_SLOTS_PER_CYCLE];
static tdma_add_request tdma_host_pending_requests[TDMA_BEACON_MAX_ADDITIONS];
static uint8_t tdma_host_pending_requests_i = 0;

/**
 * @brief Contains the (pseudo) beacon info obtained from master or sync node
 */
static tdma_beacon_info last_beacon_info;

/**
 * @brief Contains the last data packet received by master to be passed
 *        to application layer
 */
static tdma_info last_pkt_info;
static uint8_t last_pkt_buf[TDMA_MAX_PKT_SIZE];
static uint8_t last_pkt_buf_length = 0;

/**
 * @brief Contains the next data packet to be sent by client in TX slot
 */
static uint8_t tx_pkt_buf[TDMA_MAX_PKT_SIZE];
static uint8_t tx_pkt_buf_length = 0;

/**
 * @brief Contains the last contention request to be forwarded
 * last_contention_fresh - 1 if we haven't forwarded this content, 0 otherwise
 */
static tdma_contention_add_request_info last_contention_info;
static uint8_t last_contention_fresh = 0;
static uint8_t last_contention_level = 0;

/**
 * @brief Contains the contention requests forwarded on behalf of other nodes
 * 		  that have not been acknowledged by master yet.
 */
static uint16_t tdma_client_pending_requests[TDMA_MAX_FWD_CLIENTS];
static uint8_t tdma_client_pending_requests_num = 0;

/**
 * @brief The packets to be forwarded on behalf of our children nodes
 */
static uint8_t fwd_pkts[TDMA_MAX_PKT_SIZE];
static uint8_t fwd_pkt_len = 0;

/**
 *  This is a callback if you require immediate response to a packet
 */
RF_RX_INFO *rf_rx_callback(RF_RX_INFO * pRRI) {
	// Any code here gets called the instant a packet is received from the interrupt   
	return pRRI;
}

uint8_t tdma_sync_ok() {
	return sync_status;
}

int8_t tdma_set_error_callback(void(*fp)(void)) {
	if (fp == NULL)
		return NRK_ERROR;
	tdma_error_callback = fp;
	return NRK_OK;
}

int8_t tdma_tx_slot_add(uint16_t slot) {
	tdma_tx_sched[0] = slot;
	tdma_tx_slots = 1;
	return NRK_OK;
}

int8_t tdma_tx_reserve_set(nrk_time_t * period, uint16_t pkts) {
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

uint16_t tdma_tx_reserve_get() {
#ifdef NRK_MAX_RESERVES
	if (tx_reserve >= 0)
	return nrk_reserve_get (tx_reserve);
	else
	return 0;
#else
	return 0;
#endif
}

int8_t tdma_set_rf_power(uint8_t power) {
	if (power > 31)
		return NRK_ERROR;
	rf_tx_power(power);
	return NRK_OK;
}

int8_t tdma_set_channel(uint8_t chan) {
	if (chan > 26)
		return NRK_ERROR;
	tdma_chan = chan;
//rf_init (&tdma_rfRxInfo, chan, 0xFFFF, 0x00000);
	rf_init(&tdma_rfRxInfo, chan, 0x2420, 0x1214);
	return NRK_OK;
}

int8_t tdma_set_slot_len_ms(uint16_t len) {
	tdma_slot_len_ms = len;
	_tdma_slot_time.nano_secs = len * NANOS_PER_MS;
	_tdma_slot_time.secs = 0;
	return NRK_OK;
}

int8_t tdma_set_slots_per_cycle(uint16_t slots_per_cycle) {
	tdma_slots_per_cycle = slots_per_cycle + tdma_contention_slots;
	return NRK_OK;
}

int8_t tdma_set_contention_slots(uint8_t cslots_per_cycle) {
	tdma_contention_slots = cslots_per_cycle;
	return NRK_OK;
}

/**
 * @brief Check the last (pseudo-)beacon received for any add request
 *        that had gone through
 * @param mac The addr to pass out the mac of the add request accepted
 * @param num_slots The addr to pass out the number of slots for this add request.
 * @param slots The addr to pass out the slots number to be used for this add request.
 * @return NRK_OK on success and NRK_ERROR on failure
 */
int8_t _tdma_get_add_confirmation(uint16_t* mac, uint8_t* num_slots,
		uint16_t* slots) {
	uint8_t i, j;
	uint16_t tmp_mac;
	i = TDMA_BEACON_LENGTH_MIN;

	while (i < last_beacon_info.header_length) {
		tmp_mac = (((uint16_t) ((uint8_t*) &last_beacon_info)[i + 1]) << 8)
				| ((uint16_t) ((uint8_t*) &last_beacon_info)[i]);

		/* Check whether we are parents of this added mac */
		/* @TODO: Actually, we might have more than 1 added children... */
		for (j = 0; j < TDMA_MAX_FWD_CLIENTS; j++) {
			if (tdma_client_pending_requests[j] == tmp_mac) {
				*mac = tmp_mac;
				tdma_client_pending_requests[j] = 0;
				tdma_client_pending_requests_num--;
				*num_slots = ((uint8_t*) &last_beacon_info)[i + 2];
				memcpy(slots, &((uint8_t*) &last_beacon_info)[i + 3],
						*num_slots * 2);
				return NRK_OK;
			}
		}
		i += 3 + 2 * (((uint8_t*) &last_beacon_info)[i + 2]);
	}
	return NRK_ERROR;
}

/**
 * @brief Fill in the buffer passed in with relevant header information for clients
 * @param fd The buffer to be filled in
 * @return NRK_OK on success and NRK_ERROR on failure
 */
int8_t _tdma_fill_header(void * fd) {
	uint8_t i;
	uint16_t added_mac, added_slots[TDMA_MAX_HOPS];
	uint8_t added_slot_num;
	tdma_schedule_event event;
	nrk_time_t curr_time;

	switch (tdma_schedule[tdma_schedule_i].type) {
	case SYNC: {

#ifdef DEBUG_ERROR
		printf("_tdma_fill_header: Trying to send packet in SYNC slot");
#endif

		return NRK_ERROR;
		break;
	}
	case TX: {
#ifdef DEBUG-NRK
		printf("_tdma_fill_header: TX REQUEST\r\n");
#endif
		tdma_data_info* header = fd;

		header->slot = tdma_schedule[tdma_schedule_i].slot;
		header->cycle_size = last_beacon_info.cycle_size;
		header->level = tdma_level;
		header->slot_len_ms = last_beacon_info.slot_len_ms;
		header->original_src = tdma_my_mac;

		header->clients = tdma_client_num_children;
		header->timestamp = 0; /* @TODO */
		header->header_length = TDMA_DATA_LENGTH_MIN;
		header->type = DATA;
		header->seq = seq;
		seq++;

		/* Forward a contention request through us*/
		if (last_contention_fresh) {
			if (tdma_client_pending_requests_num < TDMA_MAX_FWD_CLIENTS) {
				for (i = 0; i < TDMA_MAX_FWD_CLIENTS; i++) {
					if (tdma_client_pending_requests[i]
							== last_contention_info.src) {
						header->add_node_level = last_contention_level;
						header->add_node_mac = last_contention_info.src;

						//printf("FWD Added nodes for %u, level %u\r\n",
						//header->add_node_mac, header->add_node_level);
						header->header_length += 3;
						header->type = DATA_ADD_REQUEST;
						last_contention_fresh = 0;
						break;
					}
				}
				if (last_contention_fresh) {
					for (i = 0; i < TDMA_MAX_FWD_CLIENTS; i++) {
						if (tdma_client_pending_requests[i] == 0) {
							tdma_client_pending_requests[i] =
									last_contention_info.src;
							tdma_client_pending_requests_num++;

							header->add_node_level = last_contention_level;
							header->add_node_mac = last_contention_info.src;
							//printf("FWD Added nodes for %u, level %u\r\n",
							//header->add_node_mac,
							//header->add_node_level);
							header->header_length += 3;
							header->type = DATA_ADD_REQUEST;
							last_contention_fresh = 0;
							break;
						}
					}
				}
			} else {
#ifdef DEBUG_ERROR
				nrk_kprintf(PSTR ("ERROR: Too many pending add requests\r\n"));
#endif
			}
		}

		/* Include in the contention requests that just got acknowledged from host */
		if (_tdma_get_add_confirmation(&added_mac, &added_slot_num,
				added_slots) == NRK_OK) {
			//printf("got confirmation for %u, level %u\r\n", added_mac,
			//added_slot_num);
			/* Add a new FWD_TX event for our sync node */
			event.slot = added_slots[0];
			event.type = FWD_TX;
			_tdma_schedule_add_event(&event, NULL);

			/* Add a new FWD_RX event for our sync node */
			event.slot = added_slots[1];
			event.type = FWD_RX;
			_tdma_schedule_add_event(&event, NULL);

			/* Update number of clients */
			tdma_client_num_children++;
			header->clients = tdma_client_num_children;

			/* Build the header to include this new contention request */
			*((uint16_t *) &(((uint8_t *) fd)[header->header_length])) =
					added_mac;
			header->header_length += 2;

			memcpy(&((uint8_t *) fd)[header->header_length], &added_slots[1],
					(added_slot_num - 1) * 2);
			header->header_length += (added_slot_num - 1) * 2;
		}

		/* Copy the application data to the back of the header */
		for (i = 0; i < tx_pkt_buf_length; i++)
			tdma_rfTxInfo.pPayload[i + header->header_length] = tx_pkt_buf[i];
		tdma_rfTxInfo.length = tx_pkt_buf_length + header->header_length;
		tx_pkt_buf_length = 0;

		/* If no useful information to be sent, skip this cycle
		 * @TODO: What will happen to those who use us as sync node */
		if (tdma_rfTxInfo.length == TDMA_DATA_LENGTH_MIN) {
			nrk_gpio_toggle(NRK_SCK);
			tdma_rfTxInfo.length = 0;
		}

		return NRK_OK;
		break;
	}
	case FWD_TX: {
#ifdef DEBUG-NRK
		printf("_tdma_fill_header: FWD_TX REQUEST\r\n");
#endif
		/* Check whether we have a packet to be forwarded */
		//printf("FWD_TX tdma_schedule_i = %u\r\n", tdma_schedule_i);
		if (tdma_schedule[tdma_schedule_i].fwd_pkt_i >= 0) {

			memcpy(fd, fwd_pkts, fwd_pkt_len);
			tdma_rfTxInfo.length = fwd_pkt_len;

			tdma_data_info* header = fd;
			header->level--;
			header->clients = tdma_client_num_children;
			header->slot = tdma_schedule[tdma_schedule_i].slot;
			header->slot_len_ms = tdma_slot_len_ms;
			header->cycle_size = tdma_slots_per_cycle;

			tdma_schedule[tdma_schedule_i].fwd_pkt_i = -1;
			return NRK_OK;
		} else {
#ifdef DEBUG_ERROR
			//printf("no FWD_TX\r\n");
			//printf(
			//"_tdma_fill_header: could not find a packet to be forwarded in FWD_TX slot %u\r\n",
			//tdma_schedule[tdma_schedule_i].slot);
#endif
			return NRK_ERROR;

		}
		break;
	}
	case CONTENTION: {
#ifdef DEBUG-NRK
		printf("_tdma_fill_header: CONTENTION REQUEST\r\n");
#endif
		/* While we are not sync-ed, keep trying */
		if (sync_node.added == 0) {
			//printf("next_backoff = %u\r\n", sync_node.next_backoff);
			if (sync_node.next_backoff <= 0) {
				tdma_contention_add_request_info* header = fd;
				header->type = ADD_REQUEST;
				header->src = tdma_my_mac;
				header->sync_src = sync_node.mac;

				if (sync_node.backoff_exp <= TDMA_MAX_BACKOFF) {
					/* Reset the next backoff */
					sync_node.backoff_exp *= 2;
					nrk_time_get(&curr_time);
					sync_node.next_backoff = (curr_time.nano_secs
							% sync_node.backoff_exp) + (sync_node.level + 1) * 2;
					return NRK_OK;
				} else {
					/* Reset the sync node to choose the next sync node */
#ifdef DEBUG_ERROR
					printf(
							"_tdma_fill_header: Reached backoff max limit, try to find a new sync node\r\n");
#endif
					_tdma_reset_schedule();
					return NRK_ERROR;
				}
			} else {
				sync_node.next_backoff--;
				return NRK_ERROR;
			}
		} else {
			return NRK_ERROR;
		}
		break;
	}
	default:

#ifdef DEBUG_ERROR
		printf("_tdma_fill_header: DEFAULT CASE\r\n");
#endif
		return NRK_ERROR;
		break;
	}
	return NRK_ERROR;
}

/**
 * @brief A function to fill in the packet before transmitting it
 */
int8_t _tdma_parse_tx() {
	uint8_t header_length, i;
	tdma_rfTxInfo.pPayload = tdma_tx_buf;
	tdma_beacon_info* header = (tdma_beacon_info*) tdma_rfTxInfo.pPayload;

	if (_tdma_fill_header(tdma_rfTxInfo.pPayload) == NRK_ERROR) {
		tdma_rfTxInfo.length = 0;
		return NRK_ERROR;
	}

	return NRK_OK;
}

/**
 * @brief Called by application layer to transmit any application data
 *
 * @return NRK_OK on success and NRK_ERROR on failure
 */
int8_t tdma_send(tdma_info * fd, uint8_t * buf, uint8_t len, uint8_t flags) {
	uint32_t mask;
	uint8_t i;
	uint8_t header_length = 0;

#ifdef DEBUG-NRK
	printf("tdma_send: len = %u\r\n", len);
#endif

	if (tx_pkt_buf_length > 0)
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
		nrk_signal_register(tdma_tx_pkt_done_signal);

#ifdef DEBUG
	nrk_kprintf (PSTR ("Waiting for tx done signal\r\n"));
#endif

	for (i = 0; i < len; i++)
		tx_pkt_buf[i] = buf[i];
	tx_pkt_buf_length = len;

	if (flags == TDMA_BLOCKING) {
		nrk_gpio_toggle(NRK_MISO);
		mask = nrk_event_wait(SIG(tdma_tx_pkt_done_signal));
		if (mask == 0)
			nrk_kprintf(PSTR("TDMA TX: Error calling event wait\r\n"));
		if ((mask & SIG(tdma_tx_pkt_done_signal)) == 0)
			nrk_kprintf(PSTR("TDMA TX: Woke up on wrong signal\r\n"));
		return NRK_OK;
	}

	return NRK_OK;
}

static int8_t _tdma_check_added_nodes() {
	uint8_t i;
	uint16_t temp;
	uint16_t slot = 0;
	tdma_schedule_event event;

	switch (tdma_rfRxInfo.pPayload[0]) {
	case BEACON: {
		tdma_beacon_info *beacon = (tdma_beacon_info *) tdma_rfRxInfo.pPayload;
		if (beacon->header_length > TDMA_BEACON_LENGTH_MIN) {
			i = TDMA_BEACON_LENGTH_MIN;
			while (i < beacon->header_length) {
				temp = (((uint16_t) ((uint8_t*) beacon)[i + 1]) << 8)
						| ((uint16_t) ((uint8_t*) beacon)[i]);

				/* For beacon, there is always one slot (since we are one hop away) */
				if (temp == tdma_my_mac) {
					slot = (((uint16_t) ((uint8_t*) beacon)[i + 4]) << 8)
							| ((uint16_t) ((uint8_t*) beacon)[i + 3]);
					break;
				}
				i += ((uint8_t*) beacon)[i + 2] * 2 + 3;
			}
			if (slot > 0)
				break;
			return NRK_ERROR;
		}
		return NRK_ERROR;
		break;
	}

	case DATA: {
		//printf("Checking added nodes in DATA\r\n");
		tdma_data_info *header = (tdma_data_info *) tdma_rfRxInfo.pPayload;
		if (header->header_length > TDMA_DATA_LENGTH_MIN) {
			//printf(
			//"got mac %u\r\n",
			//*((uint16_t *) &(((uint8_t *) header)[TDMA_DATA_LENGTH_MIN])));
			if (*((uint16_t *) &(((uint8_t *) header)[TDMA_DATA_LENGTH_MIN]))
					== tdma_my_mac) {
				slot =
						*((uint16_t *) &(((uint8_t *) header)[TDMA_DATA_LENGTH_MIN
								+ 2]));
				break;
			}
		}
		return NRK_ERROR;
		break;
	}

	case DATA_ADD_REQUEST: {
		tdma_data_info *header = (tdma_data_info *) tdma_rfRxInfo.pPayload;
		if (header->header_length > TDMA_DATA_ADD_REQUEST_LENGTH_MIN) {
			if (header->added_node_mac == tdma_my_mac) {
				slot = header->added_node_slots[0];
				break;
			}
		}
		return NRK_ERROR;
		break;
	}

	default:
		return NRK_ERROR;
		break;
	}

	/* Add a new event to TX at the slot obtained */
	event.slot = slot;
#ifdef DEBUG-NRK
	printf("_tdma_check_added_nodes: event.slot = %u\r\n", event.slot);
#endif
	event.type = TX;
	return _tdma_schedule_add_event(&event, NULL);
}

/**
 * @brief If beacon from our sync node, copy it for usage later
 */
static int8_t _tdma_parse_beacon() {
	tdma_beacon_info *beacon = (tdma_beacon_info *) tdma_rfRxInfo.pPayload;
	if (tdma_schedule[tdma_schedule_i].type == SYNC) {
		memcpy(&last_beacon_info, beacon, beacon->header_length);
	}
	return NRK_OK;
}

/**
 * @brief Receive a data packet.
 * 		  For master, do not need any header information
 * 		  For clients, if SYNC event - copy pseudo beacon
 * 		               if FWD_RX event - copy packet to be forwarded
 */
static int8_t _tdma_parse_data() {
	tdma_data_info *header = (tdma_data_info *) tdma_rfRxInfo.pPayload;

	if (tdma_mode == TDMA_HOST) { /* Do nothing for master */
		return NRK_OK;
	} else {
		/* When we are at SYNC event, copy the pseudo-beacon */
		if (tdma_schedule[tdma_schedule_i].type == SYNC
				&& tdma_rfRxInfo.srcAddr == sync_node.mac) {
			memcpy(&last_beacon_info, header, TDMA_BEACON_LENGTH_MIN);
			last_beacon_info.header_length = TDMA_BEACON_LENGTH_MIN;

			/* Currently assumes only at most 1 node being added */
			if (header->header_length > TDMA_DATA_LENGTH_MIN) {
				/* Copy Mac of the node being added */
				*(uint16_t*) &last_beacon_info.added_nodes[0] =
						*((uint16_t *) &(((uint8_t *) header)[TDMA_DATA_LENGTH_MIN]));

				/* Copy number of slots being added */
				last_beacon_info.added_nodes[2] = (header->header_length
						- TDMA_DATA_LENGTH_MIN - 2) / 2;

				/* Copy the slots being added */
				memcpy(&last_beacon_info.added_nodes[3],
						&(((uint8_t *) header)[TDMA_DATA_LENGTH_MIN + 2]),
						last_beacon_info.added_nodes[2] * 2);

				/* Update the length of the beacon */
				last_beacon_info.header_length +=
						(last_beacon_info.added_nodes[2] * 2) + 3;

			}
		}

		/* If we are at FWD_RX event, copy the packet to be transmitted */
		if (tdma_schedule[tdma_schedule_i].type == FWD_RX) {
			//printf("RX FWD PKT at slot %u\r\n", tdma_schedule_i);
			memcpy(fwd_pkts, header, tdma_rfRxInfo.length);
			fwd_pkt_len = tdma_rfRxInfo.length;
			tdma_schedule[tdma_schedule_i + 1].fwd_pkt_i = 0; /* Next slot is FWD_TX */
			tdma_schedule[tdma_schedule_i].missed_pkts = 0;
			return NRK_OK;
		}
	}
	return NRK_ERROR;
}

/**
 * @brief Receives a contention request.
 *        For master, add it onto waiting queue to be scheduled
 *        For client, if contention to be forwarded through us, keep it for TX later.
 */
static int8_t _tdma_parse_contention() {
	tdma_contention_add_request_info *cont =
			(tdma_contention_add_request_info *) tdma_rfRxInfo.pPayload;

	/* If someone wants us to be their forward node */
	//printf("cont->sync_src = %u, tdma_my_mac = %u\r\n", cont->sync_src,
	//tdma_my_mac);
	if (cont->sync_src == tdma_my_mac) {
		if (last_contention_fresh == 0) {
			if (tdma_mode == TDMA_HOST) {
				if (tdma_host_pending_requests_i < TDMA_BEACON_MAX_ADDITIONS) {
					tdma_host_pending_requests[tdma_host_pending_requests_i].mac =
							cont->src;
					tdma_host_pending_requests[tdma_host_pending_requests_i].level =
							1;
					tdma_host_pending_requests_i++;
				}
			} else {
				last_contention_fresh = 1;
				last_contention_level = tdma_level + 1;
				memcpy(&last_contention_info, cont,
						sizeof(tdma_contention_add_request_info));
			}
			return NRK_OK;
		}
	}
	return NRK_ERROR;
}

/**
 * @brief Receives a data_add request.
 * 		  If master, grab the add request to be scheduled later
 * 		  If client, if at SYNC event, copy pseudo beacon
 * 		             if at FWD_RX event, copy packet to be FWDed later
 */
static int8_t _tdma_parse_data_add_request() {
	tdma_data_info *header = (tdma_data_info *) tdma_rfRxInfo.pPayload;
	if (tdma_mode == TDMA_HOST) {
		//printf("data->add_node = %u\r\n", header->add_node_mac);
		if (tdma_host_pending_requests_i < TDMA_BEACON_MAX_ADDITIONS) {
			tdma_host_pending_requests[tdma_host_pending_requests_i].mac =
					header->add_node_mac;
			tdma_host_pending_requests[tdma_host_pending_requests_i].level =
					header->add_node_level;
			tdma_host_pending_requests_i++;
		}
	} else {
		if (tdma_schedule[tdma_schedule_i].type == SYNC
				&& tdma_rfRxInfo.srcAddr == sync_node.mac) {
			memcpy(&last_beacon_info, header, TDMA_BEACON_LENGTH_MIN);
			last_beacon_info.header_length = TDMA_BEACON_LENGTH_MIN;

			if (header->header_length > TDMA_DATA_ADD_REQUEST_LENGTH_MIN) {
				// Copy Mac
				*(uint16_t*) &last_beacon_info.added_nodes[0] =
						header->added_node_mac;

				// Copy Number of slots
				last_beacon_info.added_nodes[2] = (header->header_length
						- TDMA_DATA_ADD_REQUEST_LENGTH_MIN - 2) / 2;

				// Copy Slot numbers
				memcpy(&last_beacon_info.added_nodes[3],
						&header->added_node_slots,
						last_beacon_info.added_nodes[2] * 2);
				last_beacon_info.header_length +=
						(last_beacon_info.added_nodes[2] * 2) + 3;

			}
		}

		/* If we are at FWD_RX event, copy the packet to be transmitted */
		if (tdma_schedule[tdma_schedule_i].type == FWD_RX) {
			memcpy(fwd_pkts, header, tdma_rfRxInfo.length);
			fwd_pkt_len = tdma_rfRxInfo.length;
			tdma_schedule[tdma_schedule_i + 1].fwd_pkt_i = 0; /* Next slot is FWD_TX */
			tdma_schedule[tdma_schedule_i].missed_pkts = 0;

			if (last_contention_fresh == 0) {
				last_contention_info.src = header->add_node_mac;
				last_contention_info.sync_src = sync_node.mac;
				last_contention_info.type = CONTENTION;
				last_contention_level = header->add_node_level;
				last_contention_fresh = 1;
				return NRK_OK;
			}

			return NRK_OK;
		}
	}
	return NRK_ERROR;
}

int8_t _tdma_parse_rx() {
	uint8_t err = NRK_ERROR;

	/* Process header of the packet received */
	switch (((uint8_t *) tdma_rfRxInfo.pPayload)[0]) {
	case BEACON:
		//printf("tdma_recv: BEACON RX\r\n");
		err = _tdma_parse_beacon();
		break;
	case DATA:
		//printf("tdma_recv: DATA RX\r\n");
		err = _tdma_parse_data();
		break;
	case ADD_REQUEST:
		//printf("tdma_recv: ADD_REQUEST RX\r\n");
		err = _tdma_parse_contention();
		break;
	case DATA_ADD_REQUEST:
		//printf("tdma_recv: DATA_ADD_REQUEST RX\r\n");
		err = _tdma_parse_data_add_request();
		break;
	default:
		//printf("tdma_recv: DEFAULT RX\r\n");
		return NRK_ERROR;
		break;
	}

	/* Process data of the packet received */
	if (tdma_mode == TDMA_HOST && tdma_rfRxInfo.pPayload[0] != ADD_REQUEST) {
		tdma_data_info *header = (tdma_data_info *) tdma_rfRxInfo.pPayload;

		/* Copy header info */
		last_pkt_info.src = header->original_src;
		last_pkt_info.dst = tdma_my_mac;
		last_pkt_info.cycle_size = header->cycle_size;
		last_pkt_info.rssi = tdma_rfRxInfo.rssi;
		last_pkt_info.seq_num = header->seq;
		last_pkt_info.slot = header->slot;
		last_pkt_info.slot_len_ms = header->slot_len_ms;
		last_pkt_info.temp = tdma_rfRxInfo.srcAddr;
		last_pkt_info.level = header->level;

		/* Copy data */
		last_pkt_buf_length = tdma_rfRxInfo.length - header->header_length;
		memcpy(last_pkt_buf, &tdma_rfRxInfo.pPayload[header->header_length],
				last_pkt_buf_length);
		nrk_event_signal(tdma_rx_pkt_signal);

		//printf("_tdma_parse_rx: last_pkt_info.slot = %u, slot = %u\r\n",
		//last_pkt_info.slot, host_slot);
	}
	return err;
}

int8_t tdma_recv(tdma_info *fd, uint8_t *buf, uint8_t *len, uint8_t flags) {
	nrk_sig_mask_t event;
	uint8_t i;

	// First, verify that we have a received packet ready and signal the radio
	// if it is not ready and flags == TDMA_BLOCKING
	if (flags == TDMA_BLOCKING) {
		if (last_pkt_buf_length == 0) {
			nrk_signal_register(tdma_rx_pkt_signal);
			//printf("tdma_recv: Waiting for Signal RX\r\n");
			event = nrk_event_wait(SIG(tdma_rx_pkt_signal));
			if (last_pkt_buf_length == 0)
				return NRK_ERROR;
			//printf("tdma_recv: Got signal tdma_rx_pkt_signal\r\n");
		}
	} else if (last_pkt_buf_length == 0)
		return NRK_ERROR;

	*len = last_pkt_buf_length;

	if (last_pkt_buf_length != 0) {
		for (i = 0; i < *len; i++)
			buf[i] = last_pkt_buf[i];
		memcpy(fd, &last_pkt_info, sizeof(tdma_info));
		last_pkt_buf_length = 0;
	}

	// Check to see if it was a timeout
	if (flags == TDMA_BLOCKING)
		if ((event & SIG(tdma_rx_pkt_signal)) == 0) {
			//printf("tdma_recv: Signal RX timeout\r\n");
			return NRK_ERROR;
		}

	tdma_rx_buf_empty = 1;
	return NRK_OK;
}

int8_t tdma_rx_pkt_set_buffer(uint8_t * buf, uint8_t size) {
	if (buf == NULL)
		return NRK_ERROR;
	tdma_rfRxInfo.pPayload = buf;
	tdma_rfRxInfo.max_length = size;
	tdma_rx_buf_empty = 1;
	return NRK_OK;
}

int8_t tdma_init(uint8_t mode, uint8_t chan, uint16_t my_mac) {
	tx_reserve = -1;
	tdma_rx_failure_cnt = 0;
	tdma_mode = mode;
	tdma_tx_slots = 0;

	sync_status = 0;
	sync_node.clients = 255;
	sync_node.level = 255;
	sync_node.rssi = -30000;
	sync_node.slot = 0;

	_tdma_slot_time.nano_secs = TDMA_DEFAULT_SLOT_MS * NANOS_PER_MS;
	_tdma_slot_time.secs = 0;
	tdma_slot_len_ms = TDMA_DEFAULT_SLOT_MS;

	memset(host_schedule, 0,
			sizeof(tdma_host_schedule) * TDMA_DEFAULT_SLOTS_PER_CYCLE);

	//Set callback function for time sync on RX_FRAME_START interrupt
	if (tdma_mode != TDMA_HOST)
		rx_start_callback(&_tdma_rx_time_sync);

	tdma_contention_slots = TDMA_DEFAULT_CONTENSION_SLOTS;
	tdma_slots_per_cycle = TDMA_DEFAULT_SLOTS_PER_CYCLE
			+ TDMA_DEFAULT_CONTENSION_SLOTS;

	tdma_rx_pkt_signal = nrk_signal_create();
	if (tdma_rx_pkt_signal == NRK_ERROR) {
		nrk_kprintf(PSTR("TDMA ERROR: creating rx signal failed\r\n"));
		nrk_kernel_error_add(NRK_SIGNAL_CREATE_ERROR,
				nrk_cur_task_TCB->task_ID);
		return NRK_ERROR;
	}

	tdma_tx_pkt_done_signal = nrk_signal_create();
	if (tdma_tx_pkt_done_signal == NRK_ERROR) {
		nrk_kprintf(PSTR("TDMA ERROR: creating tx signal failed\r\n"));
		nrk_kernel_error_add(NRK_SIGNAL_CREATE_ERROR,
				nrk_cur_task_TCB->task_ID);
		return NRK_ERROR;
	}

	tdma_enable_signal = nrk_signal_create();
	if (tdma_enable_signal == NRK_ERROR) {
		nrk_kprintf(PSTR("TDMA ERROR: creating enable signal failed\r\n"));
		nrk_kernel_error_add(NRK_SIGNAL_CREATE_ERROR,
				nrk_cur_task_TCB->task_ID);
		return NRK_ERROR;
	}

	// Set the one main rx buffer
	tdma_rx_pkt_set_buffer(tdma_rx_buf, TDMA_MAX_PKT_SIZE);
	tdma_rx_buf_empty = 1;
	tx_data_ready = 0;

	// Setup the radio
	rf_init(&tdma_rfRxInfo, chan, 0xffff, my_mac);
	tdma_chan = chan;
	tdma_my_mac = my_mac;

	// default cca thresh of -45
	rf_set_cca_thresh(-45);

	asm volatile ("":::"memory");
	tdma_running = 1;
	tdma_is_enabled = 1;
	return NRK_OK;
}

nrk_sig_t tdma_get_rx_pkt_signal() {
	nrk_signal_register(tdma_rx_pkt_signal);
	return (tdma_rx_pkt_signal);
}

nrk_sig_t tdma_get_tx_done_signal() {
	nrk_signal_register(tdma_tx_pkt_done_signal);
	return (tdma_tx_pkt_done_signal);
}

uint8_t *tdma_rx_pkt_get(uint8_t * len, int8_t * rssi) {
	if (tdma_rx_buf_empty == 1) {
		*len = 0;
		*rssi = 0;
		return NULL;
	}
	*len = tdma_rfRxInfo.length;
	*rssi = tdma_rfRxInfo.rssi;
	return tdma_rfRxInfo.pPayload;
}

int8_t tdma_rx_pkt_release(void) {
	tdma_rx_buf_empty = 1;
	return NRK_OK;
}

void tdma_disable() {
	tdma_is_enabled = 0;
	rx_start_callback(NULL);
	rf_power_down();
}

void tdma_enable() {
	tdma_is_enabled = 1;
	if (tdma_mode != TDMA_HOST)
		rx_start_callback(&_tdma_rx_time_sync);
	rf_power_up();
	nrk_event_signal(tdma_enable_signal);
}

/**
 * @brief Verify that the packet received by the host is valid and 
 *        update the host schedule table appropriately and reset the slot
 *        allocated if needed
 */
uint8_t _tdma_host_verify(uint16_t src) {
	uint16_t i;
	if (host_schedule[host_slot].listen == 0)
		return NRK_OK;

	if (src == 0) {
		host_schedule[host_slot].miss_count++;
		if (host_schedule[host_slot].miss_count >= TDMA_MAX_MISSED_PKTS) {
			src = host_schedule[host_slot].mac;
			for (i = 1; i < tdma_slots_per_cycle - 1; i++) {
				if (host_schedule[i].mac == src) {
					host_schedule[i].miss_count = 0;
					host_schedule[i].mac = 0;
					host_schedule[i].listen = 0;
				}
			}

		}
		return NRK_ERROR;
	}

	if (host_slot == tdma_slots_per_cycle - 1) { /* Contention slot */
		if (tdma_rfRxInfo.pPayload[0] == ADD_REQUEST) {
			return NRK_OK;
		} else {
#ifdef DEBUG_ERROR
			printf(
					"ERROR: Received non-contention msg at contention slot from %u\r\n",
					src);
#endif
			return NRK_ERROR;
		}
	}

	/* Slot is not reserved, shouldn't receive a msg */
	if (host_schedule[host_slot].mac == 0) {
#ifdef DEBUG_ERROR
		printf("ERROR: Received msg from %u at empty slot %u\r\n", src,
				host_slot);
#endif
		return NRK_ERROR;
	}

	if (src == host_schedule[host_slot].mac) {
		if (tdma_rfRxInfo.pPayload[0] == DATA
				|| tdma_rfRxInfo.pPayload[0] == DATA_ADD_REQUEST) {
			host_schedule[host_slot].miss_count = 0;
			return NRK_OK;
		}
	}

	host_schedule[host_slot].miss_count++;
	if (host_schedule[host_slot].miss_count >= TDMA_MAX_MISSED_PKTS) {
		for (i = 1; i < tdma_slots_per_cycle - 1; i++) {
			if (host_schedule[i].mac == src) {
				host_schedule[i].miss_count = 0;
				host_schedule[i].mac = 0;
				host_schedule[i].listen = 0;
			}
		}
	}
	return NRK_ERROR;
}

/**
 * @brief Try to schedule a node with mac and level given and add the information
 *        into the beacon passed in.
 */
void _tdma_schedule_add_node(tdma_beacon_info *beacon, uint16_t mac,
		uint8_t level) {
	uint8_t i = 0;
	int8_t j = 0;
	uint8_t found_space = 0;
	uint8_t *buf8;
	uint16_t *buf16;
	//printf("Master adding mac %u, level %u\r\n", mac, level);
	/* First slot for beacon, last slot for contention */
	for (i = 1; i < tdma_slots_per_cycle - 1; i++) {
		found_space = 1;
		/* Try to find level number of contiguous spaces starting from slot i */
		for (j = 0; j < level; j++) {
			if (host_schedule[i + j].mac > 0) {
				i = i + j;
				found_space = 0;
				break;
			}
		}

		if (found_space) {
			// Fill in added node's mac
			buf16 = (uint16_t*) &(((uint8_t*) beacon)[beacon->header_length]);
			buf16[0] = mac;
			beacon->header_length += 2;

			// Fill in number of slots added
			buf8 = &(((uint8_t*) beacon)[beacon->header_length]);
			buf8[0] = level;
			beacon->header_length += 1;

			// Fill in the slots added
			for (j = level - 1; j >= 0; j--) {
				host_schedule[i + j].mac = mac;
				host_schedule[i + j].listen = 0;
				buf16 =
						(uint16_t*) &(((uint8_t*) beacon)[beacon->header_length]);
				buf16[0] = i + j;
				beacon->header_length += 2;
			}
			host_schedule[i + level - 1].listen = 1;

			break;
		}
	}
}

/**
 * @brief Used by host to fill the tdma_rfTxInfo.pPayload with any new nodes
 * 		  that have been added into the schedule
 */
int8_t _tdma_fill_beacon_added_nodes() {
	tdma_beacon_info* beacon = (tdma_beacon_info *) tdma_rfTxInfo.pPayload;
	uint8_t i = 0;
	uint16_t mac;
	uint8_t level;
	uint8_t flag;

	for (i = 0; i < tdma_host_pending_requests_i; i++) {
		flag = 0;
		mac = tdma_host_pending_requests[i].mac;
		level = tdma_host_pending_requests[i].level;

		for (i = 1; i < tdma_slots_per_cycle - 1; i++) {
			if (host_schedule[i].mac == mac) {
				flag = 1;
				break;
			}
		}
		if (flag == 0)
			_tdma_schedule_add_node(beacon, mac, level);
	}

	print_beacon(beacon);
	tdma_host_pending_requests_i = 0;

	return NRK_OK;
}

void print_beacon(tdma_beacon_info* beacon) {
#ifdef DEBUG-NRK
	uint8_t i;
	printf("print_beacon: length = %u\r\n", beacon->header_length);
	printf("print_beacon: cycle_size = %u\r\n", beacon->cycle_size);
	printf("print_beacon: slot_length_ms = %u\r\n", beacon->slot_len_ms);
	if (beacon->header_length > TDMA_BEACON_LENGTH_MIN) {
		printf("print_beacon: added_nodes = ");
		for (i = 0; i < beacon->header_length - TDMA_BEACON_LENGTH_MIN; i++)
		printf("%u ", beacon->added_nodes[i]);
		printf("\r\n");
	}
#endif
}

void tdma_nw_task() {
	int8_t v;
	uint8_t i;
	uint16_t count = 0;
	int16_t tmp, tmp2;
	tdma_beacon_info* beacon;
	tdma_data_info* data;
	nrk_time_t t, t2;
#ifdef DEBUG-NRK
	printf("tdma_nw_task: In Task\r\n");
#endif
	do {
		nrk_wait_until_next_period();
	} while (!tdma_started());

	//register the signal after bmac_init has been called
	v = nrk_signal_register(tdma_enable_signal);
	host_slot = 0;
	//rf_set_rx (&tdma_rfRxInfo, tdma_chan);

	while (1) {
		if (tdma_mode == TDMA_HOST) {
			sync_status = 1; // HOST is always synced

			// This is the downstream transmit slot
			if (host_slot == 0) {
				//rf_rx_off();
				tdma_rfTxInfo.pPayload = tdma_tx_buf;
				beacon = (tdma_beacon_info*) tdma_rfTxInfo.pPayload;
				beacon->type = BEACON;
				beacon->cycle_size = tdma_slots_per_cycle;
				beacon->slot_len_ms = tdma_slot_len_ms;
				beacon->header_length = TDMA_BEACON_LENGTH_MIN;

				/*TODO*/
				beacon->timestamp = 0;
				tdma_rfTxInfo.destAddr = 0xffff;
				tdma_rfTxInfo.ackRequest = 0;
				tdma_rfTxInfo.cca = 0;
				_tdma_fill_beacon_added_nodes();
				tdma_rfTxInfo.length = beacon->header_length;
				for (i = 0; i < beacon->header_length; i++)
					tdma_rfTxInfo.pPayload[i] = ((uint8_t*) beacon)[i];

				_tdma_tx();
				rf_rx_on();
			} else {
				// Upstream data slot
				v = _tdma_rx();
				if (v == 1) {
					data = (tdma_data_info*) tdma_rfRxInfo.pPayload;
					if (_tdma_host_verify(data->original_src) == NRK_OK) {
						_tdma_parse_rx();
					} else
						nrk_event_signal(tdma_rx_pkt_signal);
					tdma_last_tx_slot = host_slot;
				} else { /* Did not receive any packet this slot */
					_tdma_host_verify(0);
				}

			}

			//nrk_gpio_toggle(NRK_UART1_RXD);
			host_slot++;
			if (host_slot >= tdma_slots_per_cycle)
				host_slot = 0;

			//printf( "nw1=%lu %lu \r\n",_tdma_next_wakeup.secs,_tdma_next_wakeup.nano_secs/NANOS_PER_MS);
			nrk_time_add(&_tdma_next_wakeup, _tdma_next_wakeup,
					_tdma_slot_time);
			nrk_time_compact_nanos(&_tdma_next_wakeup);
			//printf( "nw2=%lu %lu \r\n",_tdma_next_wakeup.secs,_tdma_next_wakeup.nano_secs/NANOS_PER_MS);
			//printf("nw_task: slot = %u, _tdma_next_wakeup = %u:%u\r\n", slot, _tdma_next_wakeup.secs, _tdma_next_wakeup.nano_secs/NANOS_PER_MS);
			//printf("nw_tase: _tdma_slot_time = %u:%u\r\n",_tdma_slot_time.secs, _tdma_slot_time.nano_secs/NANOS_PER_MS)

			nrk_wait_until(_tdma_next_wakeup);
			nrk_gpio_toggle(NRK_UART1_TXD);
		} else { /* Clients */

			rf_power_up();
			rf_rx_on();

			_tdma_client_process_event();

			rf_rx_off();
			rf_power_down();

			//printf( "tt=%lu %lu\r\n",tmp_time.secs,tmp_time.nano_secs/NANOS_PER_MS);
			//printf( "nwnrk_led_toggle(ORANGE_LED);=%lu %lu\r\n",_tdma_next_wakeup.secs,_tdma_next_wakeup.nano_secs/NANOS_PER_MS);

			//Find next slot
			//printf("%u, %u, %u\r\n", tdma_schedule[0].slot, tdma_schedule[1].slot, tdma_schedule[2].slot);
			//printf("%u, %u, %u\r\n", tdma_schedule[0].type, tdma_schedule[1].type, tdma_schedule[2].type);
			if (tdma_schedule_i == tdma_schedule_length - 1) {
				tmp = 0;
				tmp2 = tdma_schedule[0].slot + tdma_slots_per_cycle
						- tdma_schedule[tdma_schedule_i].slot;
				tdma_schedule_i = 0;
			} else {
				tmp = tdma_schedule[tdma_schedule_i].slot;
				tdma_schedule_i++;
				tmp2 = tdma_schedule[tdma_schedule_i].slot;
			}

			//printf( "nw1=%lu %lu, tmp=%u, tmp2=%u\r\n",_tdma_next_wakeup.secs,_tdma_next_wakeup.nano_secs/NANOS_PER_MS, tmp, tmp2);

			while (tmp < tmp2) {
				nrk_time_add(&_tdma_next_wakeup, _tdma_next_wakeup,
						_tdma_slot_time);
				tmp++;
			}

			nrk_time_compact_nanos(&_tdma_next_wakeup);

			if (tdma_schedule[tdma_schedule_i].type == SYNC
					|| tdma_schedule[tdma_schedule_i].type == FWD_RX
					|| (tdma_schedule[tdma_schedule_i].type == CONTENTION
							&& sync_node.added == 1)) {
				t.nano_secs = TDMA_TIME_OFFSET * NANOS_PER_MS;
				t.secs = 0;
				nrk_time_sub(&t2, _tdma_next_wakeup, t);
				nrk_time_compact_nanos(&t);
				nrk_wait_until(t2);
			} else
				nrk_wait_until(_tdma_next_wakeup);

			//printf( "nw2=%lu %lu\r\n",_tdma_next_wakeup.secs,_tdma_next_wakeup.nano_secs/NANOS_PER_MS);

			nrk_gpio_toggle(NRK_UART1_TXD);
			/*nrk_time_get(&t);

			 if (nrk_time_cmp(t, _tdma_next_wakeup) > 0)
			 nrk_time_sub(&t, t, _tdma_next_wakeup);
			 else
			 nrk_time_sub(&t, _tdma_next_wakeup, t);
			 nrk_time_compact_nanos(&t);

			 t2.nano_secs = 0;
			 t2.secs = 0;
			 count = 0;

			 while(nrk_time_cmp(t, t2) < 0){
			 nrk_time_add(&t2, t2, _tdma_slot_time);
			 count++;
			 }
			 if(count != 0){

			 }
			 */
		}
	}
}

int8_t _tdma_reset_schedule() {
	memset(
			tdma_schedule,
			0,
			(2 * TDMA_MAX_FWD_CLIENTS + TDMA_DEFAULT_CONTENSION_SLOTS + 2)
					* sizeof(tdma_schedule_event));
	tdma_schedule_length = 0;
	sync_status = 0;

	//Reset sync_node parameters
	sync_node.clients = 255;
	sync_node.level = 255;
	sync_node.rssi = -30000;
	sync_node.slot = 0;
	sync_node.added = 0;
	sync_node.mac = 0;
	sync_node.backoff_exp = 1;
	sync_node.next_backoff = 0;
	return NRK_OK;
}

int8_t _tdma_client_process_event() {
	int8_t v;
	uint8_t event_type;
	nrk_time_t search_start_time, tmp_time;
	tdma_schedule_event event;
	uint16_t tmp, src, sync;
	tdma_beacon_info* beacon;
	tdma_data_info* data;

	/* Decide which event we are at */
	if (sync_status == 0)
		event_type = SYNC;
	else
		event_type = tdma_schedule[tdma_schedule_i].type;

#ifdef DEBUG-NRK
	printf("%u, %u, %u\r\n", tdma_schedule[0].slot, tdma_schedule[1].slot,
			tdma_schedule[2].slot);
#endif
	switch (event_type) {
	case SYNC: {
		//printf("SYNC event\r\n");
		sync = 0;
		nrk_time_get(&search_start_time);

		do {
			v = _tdma_rx();

			if (v == NRK_OK) {

				src = tdma_rfRxInfo.srcAddr;

				// Ignore Master
				/*if (src == 1) {
					v = NRK_ERROR;
					continue;
				}*/

				/* Grab (pseudo)-beacon if packet from sync node */

				if ((src == sync_node.mac || sync_status == 0)
						&& tdma_rfRxInfo.pPayload[0] != ADD_REQUEST) {

					tdma_schedule[tdma_schedule_i].missed_pkts = 0;
					if (src == sync_node.mac && sync_node.added == 0) {
						if (_tdma_check_added_nodes() == NRK_OK) {
							sync_node.added = 1;
						}
					}
					tmp = tdma_slots_per_cycle;

					beacon = (tdma_beacon_info*) tdma_rfRxInfo.pPayload;
					tdma_slots_per_cycle = beacon->cycle_size;

					if (tdma_slot_len_ms != beacon->slot_len_ms
							|| tmp != tdma_slots_per_cycle) {
						tdma_slot_len_ms = beacon->slot_len_ms;
						tdma_cycle_len_ms = tdma_slots_per_cycle
								* tdma_slot_len_ms;
					}

					_tdma_slot_time.nano_secs = tdma_slot_len_ms * NANOS_PER_MS;
					_tdma_slot_time.secs = 0;
					if (sync_status == 1)
						_tdma_parse_rx();
				}
				/* Otherwise we have lost sync */
				else {

					_tdma_reset_schedule();
				}

				if (sync_status == 0) {
					nrk_led_clr(GREEN_LED);
					nrk_led_set(RED_LED);
					//If we received a beacon from the host, use it as the sync node
					//and skip searching for a better one
					if (tdma_rfRxInfo.pPayload[0] == BEACON) {
						sync_node.level = 0;
						sync_node.slot = 0;
						sync_node.added = 0;
						sync_node.mac = tdma_rfRxInfo.srcAddr;

						event.slot = 0;
						event.type = SYNC;
						v = _tdma_schedule_add_event(&event, &tdma_schedule_i);

						// Add contention slot since we aren't added to the host's schedule yet
						event.slot = tdma_slots_per_cycle - 1;
						event.type = CONTENTION;
						v = _tdma_schedule_add_event(&event, NULL);

						sync_node.next_backoff = 0;
						sync_node.backoff_exp = 1;
						sync_status = 1;
					} else if (tdma_rfRxInfo.pPayload[0] == DATA
							|| tdma_rfRxInfo.pPayload[0] == DATA_ADD_REQUEST) { /* @TODO */
						nrk_time_sub(&tmp_time, _tdma_next_wakeup,
								search_start_time);
						data = (tdma_data_info*) tdma_rfRxInfo.pPayload;
						//printf("Got DATA from %u, slot=%u\r\n",
						//tdma_rfRxInfo.srcAddr, data->slot);
						//printf("%u:",search_start_time.secs);
						//printf("%u\r\n",search_start_time.nano_secs);
						//printf("%u\r\n",_tdma_next_wakeup.nano_secs);
						//printf("%u\r\n",tmp_time.nano_secs);

						//printf("%u\r\n", (tmp_time.secs * 1000) + (tmp_time.nano_secs >> 20));
						//printf("%u\r\n", TDMA_HOST_TIMEOUT*tdma_cycle_len_ms);

						//Determine best sync node within TDMA_HOST_TIMEOUT cycles
						//Divide nano_secs by 2^20 instead of 1,000,000 for faster computation
						if ((tmp_time.secs * 1000) + (tmp_time.nano_secs >> 20)
								< TDMA_HOST_TIMEOUT * tdma_cycle_len_ms) {
							nrk_led_set(RED_LED);
							nrk_led_clr(GREEN_LED);
							_tdma_compare_sync_node(&sync_node, &tdma_rfRxInfo);
							v = NRK_ERROR;
							sync_status = 0;
							tdma_rx_buf_empty = 1;
						} else {
							//Add the sync node to the schedule and update the current slot to this
							tdma_level = sync_node.level + 1;
							event.slot = sync_node.slot;
							event.type = SYNC;
							v = _tdma_schedule_add_event(&event,
									&tdma_schedule_i);
							//printf("added sync slot %u, schedule_i = %u\r\n",
							//event.slot, tdma_schedule_i);
							printf("sync node = %u, level %u\r\n", sync_node.mac, tdma_level);
							event.slot = tdma_slots_per_cycle - 1;
							event.type = CONTENTION;
							v = _tdma_schedule_add_event(&event, NULL);
							sync_node.next_backoff = 0;
							sync_node.backoff_exp = 1;
							sync_status = 1;
							//printf("added contention slot %u\r\n", event.slot);
						}
					}
				}
			} else {
				if (sync_status == 1) {
					tdma_schedule[tdma_schedule_i].missed_pkts++;
					if (tdma_error_callback != NULL) {
						tdma_error_callback();
					}

					if (tdma_schedule[tdma_schedule_i].missed_pkts
							>= TDMA_MAX_MISSED_PKTS) {
						//printf("resetting schedule after missed pkts\r\n");
						_tdma_reset_schedule();
					} else if (tdma_schedule_length > 0) {
						break;
					}
				}
			}
		} while (v != NRK_OK);
		nrk_led_clr(RED_LED);
		nrk_led_set(GREEN_LED);
		break;
	}

	case TX: {
		tdma_rfTxInfo.destAddr = 0xffff;
		tdma_rfTxInfo.ackRequest = 0;
		tdma_rfTxInfo.cca = 0;
		if (_tdma_parse_tx() == NRK_OK) {
			_tdma_tx();
		}
		break;
	}

	case CONTENTION: {

		if (sync_node.added == 0) {
			tdma_rfTxInfo.pPayload = tdma_tx_buf;
			tdma_rfTxInfo.length = sizeof(tdma_contention_add_request_info);
			tdma_rfTxInfo.destAddr = 0xffff;
			tdma_rfTxInfo.ackRequest = 0;
			tdma_rfTxInfo.cca = 0;
			if (_tdma_parse_tx() == NRK_OK) {
				//printf("process contention\r\n");
				_tdma_tx();
			}
		} else {
			if (_tdma_rx() == NRK_OK) {
				_tdma_parse_rx();
			}
		}
		break;
	}

	case FWD_RX: {
		if (_tdma_rx() == NRK_OK) {
			_tdma_parse_rx();
		} else {
			tdma_schedule[tdma_schedule_i].missed_pkts++;

			/* Too many missed packet for this child node, remove it */
			if (tdma_schedule[tdma_schedule_i].missed_pkts
					>= TDMA_MAX_MISSED_PKTS) {
				_tdma_schedule_remove_event(
						tdma_schedule[tdma_schedule_i].slot);
				tdma_client_num_children--;
			}
		}
		break;
	}

	case FWD_TX: {
		tdma_rfTxInfo.destAddr = 0xffff;
		tdma_rfTxInfo.ackRequest = 0;
		tdma_rfTxInfo.cca = 0;
		if (_tdma_parse_tx() == NRK_OK) {
			_tdma_tx();
		}
		break;
	}

	default:
		return NRK_ERROR;
		break;
	}
	return NRK_OK;
}

uint8_t _tdma_compare_sync_node(tdma_sync_node *node0, RF_RX_INFO *node1) {
	if (node1->pPayload[0] == DATA || node1->pPayload[0] == DATA_ADD_REQUEST) {
		tdma_data_info *header = (tdma_data_info *) tdma_rfRxInfo.pPayload;

		//printf("node0slot = %u, node1slot = %u\r\n", node0->slot, header->slot);
		//Always favor nodes that are at a lower level
		if (header->level < node0->level) {
			_tdma_update_sync_node(node0, node1);
			return 1;
		}
		//If nodes are at the same level, prefer the node with less clients
		else if (header->level == node0->level) {
			if (header->clients < node0->clients) {
				_tdma_update_sync_node(node0, node1);
				return 1;
				//If nodes are at the same level, and have the same amount of
				//clients, prefer the node with the better rssi value
			} else if (header->clients == node0->clients)
				if (node1->rssi > node0->rssi) {
					_tdma_update_sync_node(node0, node1);
					return 1;
				}
		}
	}
	return 0;
}

int8_t _tdma_update_sync_node(tdma_sync_node *node0, RF_RX_INFO *node1) {
	if (node1->pPayload[0] == DATA || node1->pPayload[0] == DATA_ADD_REQUEST) {
		tdma_data_info *header = (tdma_data_info *) tdma_rfRxInfo.pPayload;
		node0->level = header->level;
		node0->rssi = node1->rssi;
		node0->clients = header->clients;
		node0->slot = header->slot;
		node0->mac = node1->srcAddr;
		return NRK_OK;
	}
	return NRK_ERROR;
}

//Insertion sort for tdma schedule according to slot number
int8_t _tdma_schedule_sort() {
	uint8_t i;
	int16_t j;
	tdma_schedule_event tmp;
	for (i = 1; i < tdma_schedule_length; i++) {
		tmp = tdma_schedule[i];
		j = i - 1;
		while (tmp.slot < tdma_schedule[j].slot && j >= 0) {
			tdma_schedule[j + 1] = tdma_schedule[j];
			j = j - 1;
		}
		tdma_schedule[j + 1] = tmp;
	}
	return NRK_OK;
}

int8_t _tdma_schedule_add_event(tdma_schedule_event *event,
		uint8_t *inserted_slot) {
	uint8_t i = 0;
	uint16_t tmp;
	if (tdma_schedule_length
			< TDMA_MAX_FWD_CLIENTS + TDMA_DEFAULT_CONTENSION_SLOTS) {
		tdma_schedule[tdma_schedule_length] = *event;
		tdma_schedule[tdma_schedule_length].missed_pkts = 0;
		tdma_schedule[tdma_schedule_length].fwd_pkt_i = -1;

		tdma_schedule_length++;
		tmp = tdma_schedule[tdma_schedule_i].slot;
		_tdma_schedule_sort();

		//Find the slot we were at after sorting the schedule
		tdma_schedule_i = 0;
		while (tdma_schedule[tdma_schedule_i].slot != tmp) {
			tdma_schedule_i++;
		}

		//If desired, find the index of the slot the event was inserted into
		if (inserted_slot != NULL) {
			while (tdma_schedule[i].slot != event->slot)
				i++;
			*inserted_slot = i;
		}
		return NRK_OK;
	}

#ifdef DEBUG_ERROR
	nrk_kprintf(
			PSTR("ERROR: Cannot add slot to TDMA schedule, maximum slot number reached.\r\n"));
#endif

	return NRK_ERROR;
}

int8_t _tdma_schedule_remove_event(uint16_t slot) {
	uint8_t i = 0;
	while (tdma_schedule[i].slot != slot && i < tdma_schedule_length)
		i++;
	if (i == tdma_schedule_length) {
#ifdef DEBUG_ERROR
		printf("tdma_schedule does not have slot %u to be removed\r\n", slot);
#endif
		return NRK_ERROR;
	}

	/* If we are removing the slot we are currently at, set tdma_schedule_i
	 to the slot before the current one */
	if (tdma_schedule[tdma_schedule_i].slot == slot) {
		if (tdma_schedule_i == 0)
			tdma_schedule_i = tdma_schedule_length - 1;
		else
			tdma_schedule_i--;
	}

	//Remove the slot and close the resulting gap in the array
	while (i < tdma_schedule_length - 1) {
		tdma_schedule[i] = tdma_schedule[i + 1];
		i++;
	}
	tdma_schedule_length--;
	return NRK_OK;
}

int8_t tdma_started() {
	return tdma_running;
}

int8_t _tdma_rx() {
	int8_t v, i;
	v = 0;

	for (i = 0; i < tdma_slot_len_ms * 7; i++) { /* Listen for 80% of the time of the slot */
		nrk_gpio_toggle(NRK_UART1_RXD);
		v = rf_rx_packet_nonblock();
		if (v == 1) {
#ifdef DEBUG-NRK
			printf("_tdma_rx: tdma_rfRxInfo.length = %u\r\n",
					tdma_rfRxInfo.length);
			printf("_tdma_rx: tdma_rfRxInfo.src = %u\r\n",
					tdma_rfRxInfo.srcAddr);
#endif
			if (tdma_rfRxInfo.length
					>= sizeof(tdma_contention_add_request_info)) {
				tdma_rx_buf_empty = 0;

				switch (tdma_rfRxInfo.pPayload[0]) {
				case BEACON:
					nrk_gpio_toggle(NRK_SCK);
					break;
				case DATA:
					nrk_gpio_toggle(NRK_MOSI);
					break;
				case ADD_REQUEST:
					nrk_gpio_toggle(NRK_MISO);
					break;
				case DATA_ADD_REQUEST:
					//nrk_gpio_toggle(NRK_MISO);
					break;
				default:
					//printf("default RX\r\n");
					//nrk_gpio_toggle(NRK_UART1_RXD);
					break;
				}
			}
			break;
		}
		nrk_spin_wait_us(100);
	}
	return v;
}

int8_t _tdma_tx() {
	int8_t v;
	uint8_t checksum, i;
	uint8_t *data_start, *frame_start = &TRXFBST;
	tdma_beacon_info *beacon;

	if (tdma_rfTxInfo.length == 0)
		return NRK_ERROR;

	/*Calculate and append checksum for backwards compatibility to FF2*/
	checksum = 0;
	for (i = 0; i < tdma_rfTxInfo.length; i++)
		checksum += tdma_rfTxInfo.pPayload[i];

	data_start = frame_start + 9 + 1 + tdma_rfTxInfo.length;

	memcpy(data_start, &checksum, sizeof(uint8_t));
	switch (tdma_rfTxInfo.pPayload[0]) {
	case BEACON:
		nrk_gpio_toggle(NRK_ADC_INPUT_0);
		break;
	case DATA:
		nrk_gpio_toggle(NRK_ADC_INPUT_1);
		break;
	case ADD_REQUEST:
		nrk_gpio_toggle(NRK_ADC_INPUT_2);
		break;
	case DATA_ADD_REQUEST:
		nrk_gpio_toggle(NRK_ADC_INPUT_1);
		break;
	default:
		//printf("default TX\r\n");
		break;
	}
	//nrk_spin_wait_us(TDMA_TIME_OFFSET*1000);
	v = rf_tx_packet(&tdma_rfTxInfo);

	tdma_rfTxInfo.length = 0;
	nrk_event_signal(tdma_tx_pkt_done_signal);
	return NRK_OK;
}

void tdma_task_config() {
	nrk_task_set_entry_function(&tdma_task, tdma_nw_task);
	nrk_task_set_stk(&tdma_task, tdma_task_stack, TDMA_STACKSIZE);
	tdma_task.prio = TDMA_TASK_PRIORITY;
	tdma_task.FirstActivation = TRUE;
	tdma_task.Type = BASIC_TASK;
	tdma_task.SchType = PREEMPTIVE;
	tdma_task.period.secs = 0;
	tdma_task.period.nano_secs = 20 * NANOS_PER_MS;
	tdma_task.cpu_reserve.secs = 0; //PCF_TDMA_TIMEOUT; // bmac reserve , 0 to disable
	tdma_task.cpu_reserve.nano_secs = 0;
	tdma_task.offset.secs = 0;
	tdma_task.offset.nano_secs = 0;
	nrk_activate_task(&tdma_task);
}

void _tdma_rx_time_sync() {
	nrk_time_t t;
	t.nano_secs = TDMA_TIME_OFFSET * NANOS_PER_MS;
	t.secs = 0;
	if (tdma_schedule[tdma_schedule_i].slot == sync_node.slot
			|| sync_status == 0) {
		nrk_gpio_toggle(NRK_ADC_INPUT_0);
		nrk_time_get(&_tdma_next_wakeup);
		//nrk_time_sub(&_tdma_next_wakeup, _tdma_next_wakeup, t);
	}
}
