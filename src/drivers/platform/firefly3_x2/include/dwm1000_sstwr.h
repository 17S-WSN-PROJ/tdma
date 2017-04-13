/*
 * dwm1000_sstwr.h - library interface for dwm1000 single sided two way ranging
 */

#ifndef _DWM100_SSTWR_H_
#define _DWM100_SSTWR_H_

#include <stdint.h>

#define MAX_WAIT_COUNT  5

typedef struct ranging_info {
    uint32_t request_id;
    uint64_t anchor_tx_mark;
    uint64_t anchor_rx_mark;
    uint64_t responder_tx_mark;
    uint64_t responder_rx_mark;
    uint8_t done;
} ranging_info;

typedef struct ranging_packet {
    uint8_t  is_request;
    uint16_t anchor_mac;
    uint16_t responder_mac;
    uint64_t responder_rx_mark;
    uint64_t responder_tx_mark;
} ranging_packet;

uint64_t dwm_sstwr_get_last_ranging(void);
void dwm_sstwr_blockin_collect(void);
void dwm_sstwr_request_ranging(uint16_t their_mac);
void dwm_sstwr_init(void (*user_rx_callback)(uint64_t));

#endif

