// tdma_asap_scheduler.h
// Contains graph coloring and schedule establishment

#ifndef TDMA_SCHEDULER_H
#define TDMA_SCHEDULER_H

#include <tdma_asap.h>

#define TDMA_SCHEDULE_MAX_SIZE		64

// Typedefs
typedef uint16_t node_addr_t;
typedef int16_t tdma_slot_t;

// type to determine whether a slot is intended to
// send data to parent (data) or child (time sync)
// or RX
/*
typedef enum {
    TDMA_RX,
    TDMA_TX_PARENT,
    TDMA_TX_CHILD
} tdma_slot_type_t;
*/

typedef enum {
    TDMA_RXSYNC=0, //0b00
    TDMA_RXDATA=1, //0b01
    TDMA_TXSYNC=2, //0b10
    TDMA_TXDATA=3, //0b11
} tdma_slot_type_t;

typedef struct {
    tdma_slot_t slot;
    tdma_slot_type_t type;
    int8_t priority;
} tdma_schedule_entry_t;

tdma_schedule_entry_t tdma_schedule[TDMA_SCHEDULE_MAX_SIZE] ;
uint8_t tdma_schedule_size;
uint8_t cur_schedule_entry;

inline uint8_t is_rx_slot(tdma_slot_type_t st);
inline uint8_t is_sync_slot(tdma_slot_type_t st);

void tdma_schedule_print();
int8_t tdma_schedule_add(tdma_slot_t slot, tdma_slot_type_t type, int8_t priority);
tdma_schedule_entry_t tdma_schedule_get_next(tdma_slot_t slot);

int8_t nrk_time_cmp(nrk_time_t t1, nrk_time_t t2);
#endif
