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
*******************************************************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <pcf_tdma.h>
#include <slip.h>
#include <tdma_cons.h>

//CHANGE THESE AS NECESSARY
//SLOT NUMBER: number of clients + 1
//SLOT_LEN: as low as possible, 2 seems to be fine

#define SLOT_LEN 5
#define SLOT_NO 6

// 0 - gateway
// 1 - bike, cadence
// 2 - handlebars
// 3 - rider chest
// 4 - helmet
// 5 - car hood


NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type rx_task_info;
void rx_task (void);

void nrk_create_taskset ();

tdma_info rx_tdma_fd;

uint8_t rx_buf[TDMA_MAX_PKT_SIZE];
uint8_t serial_buf[128];

uint8_t aes_key[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee, 0xff}; 

int main ()
{
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_230K4);
  nrk_init ();

  // init tdma with -1 mac address
  //tdma_init (TDMA_HOST, DEFAULT_CHANNEL, -1);
  tdma_init (TDMA_HOST, 16, -1);

  // init slipstream
  slip_init (stdin, stdout, 0, 0);

  // Change these parameters at runtime...
  tdma_set_slot_len_ms (SLOT_LEN);
  tdma_set_slots_per_cycle (SLOT_NO);
  tdma_ttl_set(1);
  printf("%d ms slots, %d slots per cycle\r\n",SLOT_LEN,SLOT_NO);


  //tdma_aes_setkey(aes_key);
  //tdma_aes_enable();



  nrk_led_clr (ORANGE_LED);
  nrk_led_set (BLUE_LED);
  nrk_led_clr (GREEN_LED);
  nrk_led_clr (RED_LED);

  nrk_time_set (0, 0);
  nrk_create_taskset ();
  nrk_start ();

  return 0;
}


void rx_task ()
{
  int8_t v;
  uint8_t len, i,j;
  uint8_t pkts,datagram_len;

  while (!tdma_started () || !slip_started())
    nrk_wait_until_next_period ();


  while (1) {
    v = tdma_recv (&rx_tdma_fd, &rx_buf, &len, TDMA_NONBLOCKING);
    if (v == NRK_OK && len > 0) {
      nrk_led_set(RED_LED);

      pkts=rx_buf[len-2];
      datagram_len=rx_buf[len-1];

      for(i=0; i<pkts; i++)
	{
		// copy the datagram into the serial buf
    for(j=0; j<datagram_len; j++ )
      serial_buf[j]=rx_buf[i*datagram_len+j];

    // add rssi to end of buf
    *(int16_t *)(&serial_buf[datagram_len]) = rx_tdma_fd.rssi;

    //send the packet to the SLIPstream client
    slip_tx (serial_buf,datagram_len+2);
  }

      nrk_led_clr(RED_LED);
    }
  }
}

void nrk_create_taskset ()
{
  nrk_task_set_entry_function (&rx_task_info, rx_task);
  nrk_task_set_stk (&rx_task_info, rx_task_stack, NRK_APP_STACKSIZE);
  rx_task_info.prio = 1;
  rx_task_info.FirstActivation = TRUE;
  rx_task_info.Type = BASIC_TASK;
  rx_task_info.SchType = PREEMPTIVE;
  rx_task_info.period.secs = 0;
  rx_task_info.period.nano_secs = 25 * NANOS_PER_MS;
  rx_task_info.cpu_reserve.secs = 0;
  rx_task_info.cpu_reserve.nano_secs = 0 * NANOS_PER_MS;
  rx_task_info.offset.secs = 1;
  rx_task_info.offset.nano_secs = 0;
  nrk_activate_task (&rx_task_info);

  tdma_task_config ();

}
