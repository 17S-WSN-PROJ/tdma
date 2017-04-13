//#include <nrk_timer.h>
#include <include.h>
#include <ulib.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <nrk_eeprom.h>
#include <basic_rf.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <nrk.h>
#include <nrk_events.h>
#include <nrk_timer.h>
#include <nrk_error.h>
#include <nrk_reserve.h>
#include "tdma_asap_stats.h"

/*
inline void _tdma_timer4_setup()
{
  TCCR4A=0;  
  TCCR4B=BM(CS10);  // clk I/O no prescale
  TCNT4=0;  // 16 bit
  GTCCR |= BM(PSRASY);              // reset prescaler
  GTCCR |= BM(PSRSYNC);              // reset prescaler
}

inline void _tdma_timer4_start()
{
    TCCR4B = BM(CS10); // no prescale
}

inline void _tdma_timer4_reset()
{
    GTCCR |= BM(PSRSYNC);
    TCNT4 = 0;
}

inline void _tdma_timer4_stop()
{
    TCCR4B = 0;
}

inline uint16_t _tdma_timer4_get()
{
    uint16_t tmp;
    tmp = TCNT4;
    return tmp;
}
*/

inline void _tdma_timer5_setup()
{
  TCCR5A=0;  
  TCCR5B=BM(CS10);  // clk I/O no prescale
  TCNT5=0;  // 16 bit
  GTCCR |= BM(PSRASY);              // reset prescaler
  GTCCR |= BM(PSRSYNC);              // reset prescaler
}

inline void _tdma_timer5_start()
{
    TCCR5A = 0;
    TCCR5B = BM(CS10); // no prescale
}

inline void _tdma_timer5_reset()
{
    GTCCR |= BM(PSRSYNC);
    TCNT5 = 0;
}

inline void _tdma_timer5_stop()
{
    TCCR5A = 0;
}

inline uint16_t _tdma_timer5_get()
{
    volatile uint16_t tmp;
    tmp = TCNT5;
    return tmp;
}

void tdma_stats_setup(uint8_t pid)
{

    // used to grab the stats structure
    tdma_pid = pid;

    //steal_rdo_running = 0;
    //reg_rdo_running = 0;
    rdo_running = 0;
    ticks_rdo = 0;
    //reg_micros_rdo = 0;
    //steal_micros_rdo = 0;
    steal_cnt=0;
    backoff_cnt = 0;
    dbl_pkt_cnt = 0;
    sync_rx_cnt = 0;
    sync_slot_cnt = 0;
    //recording_awk_reg = 0;
    //recording_awk_steal =0;
    //reg_micros_awk =0;
    //steal_micros_awk =0;

    #ifdef MPKT_LOG
    mpkt_log_curbuf=0;
    #endif

    tree_creation_time.secs = 0;
    tree_creation_time.nano_secs = 0;

    //_tdma_timer4_setup();
    _tdma_timer5_setup();
}

// radio stats : use timer 5

void stats_start_rdo()
{
    rdo_running = 1;
    _tdma_timer5_reset();
    _tdma_timer5_start();
}

void stats_stop_rdo()
{
    if (rdo_running)
    {
        rdo_running = 0;
        ticks_rdo += _tdma_timer5_get();
        //printf("r%20"PRIu32" %d\r\n", ticks_rdo, _tdma_timer5_get() );
        _tdma_timer5_stop();
    }
}

// records radio while running without stopping it
void stats_record_rdo()
{
    if (rdo_running)
    {
        ticks_rdo+= _tdma_timer5_get();
        _tdma_timer5_reset();
    }
}

/*
void steal_start_rdo()
{
    steal_rdo_running=1;
    _tdma_timer5_reset();
    _tdma_timer5_start();
}

void steal_stop_rdo()
{
    if (steal_rdo_running)
    {
        steal_rdo_running = 0;
        steal_micros_rdo += _tdma_timer5_get();
        _tdma_timer5_stop();
    }
}

void steal_cca_check()
{
    steal_micros_rdo+=250;
}

void reg_start_rdo()
{
    reg_rdo_running=1;
    _tdma_timer5_reset();
    _tdma_timer5_start();
}

void reg_stop_rdo()
{
    if (reg_rdo_running)
    {
        steal_rdo_running = 0;
        reg_micros_rdo += _tdma_timer5_get() / 4;
        _tdma_timer5_stop();
    }
}
*/

inline void steal_log()
{
    steal_cnt++;
}

inline void steal_backoff_log()
{
    backoff_cnt++;
}

#ifdef MPKT_LOG
void tdma_stats_mpkt_dump()
{
    uint8_t i = 0;
    //printf("MD %d\r\n", mpkt_log_curbuf);
    while (i < mpkt_log_curbuf)
    {
        printf("%s\r\n", mpkt_log[i]);
        i++;
    }
    mpkt_log_curbuf = 0;
}
#endif

void tdma_stats_add_osa(nrk_time_t t)
{
    nrk_time_add(&osa_time, osa_time, t);
    nrk_time_compact_nanos(&osa_time);
}

void tdma_stats_record_tree_creation()
{
#ifdef NRK_STATS_TRACKER
    nrk_stats_get(tdma_pid, &tdma_stat_struct);
    tree_creation_time = _nrk_ticks_to_time(tdma_stat_struct.total_ticks);
#endif
}

void tdma_stats_ts_packet(uint8_t * tx_buf)
{
    nrk_time_t old, cur, lat;
    nrk_time_get(&cur);
    nrk_time_compact_nanos(&cur);

    //memcpy(&lat, &tx_buf[TDMA_DATA_START+8],  sizeof(lat));
    memcpy(&lat, &tx_buf[TDMA_DATA_START+8],  sizeof(lat));

    //memcpy(&old, &tx_buf[TDMA_DATA_START+16],  sizeof(old));
    memcpy(&old, &tx_buf[TDMA_DATA_START+16],  sizeof(old));

    //printf("MTTS %lu %lu %lu %lu\r\n", lat.secs, lat.nano_secs, old.secs, old.nano_secs);
    
    // subtract current time from arrival time (old)
    if (nrk_time_sub(&cur, cur, old) == NRK_ERROR)
    {
        cur.secs = 0;
        cur.nano_secs = 0;
    }
    // add that difference to the total latency
    nrk_time_add(&lat, lat, cur);


    // finally, add the cumulative latency back in
    memcpy(&tx_buf[TDMA_DATA_START+8], &lat, sizeof(lat));
}

void tdma_stats_dump()
{

    nrk_time_t ct;
    nrk_time_get(&ct);
  // PRINTS OUT...
  //
  // My MAC address
  // time tdma task has spent awake (secs)
  // time tdma task has spent awake (nanosecs)
  // time tdma task used radio
  // number of slot steals
  // number of backoffs
  // number of rx syncs
  // number of sync slots

#ifdef NRK_STATS_TRACKER
  nrk_stats_get(tdma_pid, &tdma_stat_struct);

  tdma_cpu_time = _nrk_ticks_to_time(tdma_stat_struct.total_ticks);
    //printf("TCT %lu %lu %lu %lu\r\n", tdma_cpu_time.secs, tdma_cpu_time.nano_secs,
    //                tree_creation_time.secs, tree_creation_time.nano_secs);
  nrk_time_sub(&tdma_cpu_time, tdma_cpu_time, tree_creation_time);
#endif


  printf("SD %d %lu %lu %lu %lu %lu %u %u\r\n",
    tdma_mac_get(),
    tdma_cpu_time.secs,
    tdma_cpu_time.nano_secs,
    ticks_rdo,
    steal_cnt,
    backoff_cnt,
    sync_rx_cnt,
    sync_slot_cnt);

 printf("OSA %lu %lu %lu %lu\r\n",
        osa_time.secs, osa_time.nano_secs,
        ct.secs, ct.nano_secs);


#ifdef SYNC_HIST
  printf("SH %d %d %d %d %d %d %d %d %d %d %d\r\n",
    tdma_mac_get(),
    sync_hist[0],
    sync_hist[1],
    sync_hist[2],
    sync_hist[3],
    sync_hist[4],
    sync_hist[5],
    sync_hist[6],
    sync_hist[7],
    sync_hist[8],
    sync_hist[9]);
    
#endif
}


#ifdef SYNC_HIST
void sync_rx_log()
{
    // if I've missed any syncs, record how many
    // there were up to this point
    if (sync_consec_miss > 0)
    {
        sync_hist[sync_consec_miss-1]++;
    }
    // reset the miss ctr
    sync_consec_miss = 0;
}


void sync_miss_log()
{
    if (sync_consec_miss < 10)
    {
        sync_consec_miss++;
        if (sync_consec_miss > 5)
        {
            nrk_time_t cur_time;
            nrk_time_get(&cur_time);
            nrk_time_compact_nanos(&cur_time);
            printf("SM %d %d %lu %lu\r\n", tdma_mac_get(), sync_consec_miss, cur_time.secs, cur_time.nano_secs);
        }
    }
}
#endif

void steal_dbl_pkt()
{
    dbl_pkt_cnt++;
}



/*
// awake stats = use timer 4
void stats_begin_awk_steal()
{
    recording_awk_steal=1;
    _tdma_timer4_reset();
    _tdma_timer4_start();
}

void stats_end_awk_steal()
{

    if (recording_awk_steal)
    {
        recording_awk_steal = 0;
        steal_micros_awk += _tdma_timer4_get() / 4;
        _tdma_timer5_stop();
    }
}

void stats_begin_awk_reg()
{
    recording_awk_reg=1;
    _tdma_timer4_reset();
    _tdma_timer4_start();

}

void stats_end_awk_reg()
{

    if (recording_awk_reg)
    {
        recording_awk_reg = 0;
        reg_micros_awk += _tdma_timer4_get() / 4;
        _tdma_timer5_stop();
    }
}

// decide not to record this stat
void stats_cancel_awk_reg()
{
    recording_awk_reg = 0;
    _tdma_timer5_stop();
    _tdma_timer5_reset();
}

// decide not to record this stat
void stats_cancel_awk_steal()
{
    recording_awk_steal = 0;
    _tdma_timer5_stop();
    _tdma_timer5_reset();
}

void stats_record_failed_rx_reg()
{
    reg_micros_awk += 2000; // 2MS for TDMA_RX_WAIT_TIME_MS
}
*/
