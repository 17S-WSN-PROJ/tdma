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
#include <nrk_eeprom.h>
#include <slip.h>
#include <nrk_eeprom.h>

#define HOST_MAC 0x0
#define DEFAULT_CHANNEL   26  

#define SLIPSTREAM_ACK

uint32_t mac_address;

NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type rx_task_info;
void rx_task(void);

NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type tx_task_info;
void tx_task(void);

void nrk_create_taskset();

uint8_t aes_key[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee, 0xff}; 


tdma_info tx_tdma_fd;
tdma_info rx_tdma_fd;

uint8_t slip_rx_buf[TDMA_MAX_PKT_SIZE];
uint8_t slip_tx_buf[TDMA_MAX_PKT_SIZE];

uint16_t last_seq_num,last_mac;

int
main ()
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);

  nrk_init();

  nrk_led_clr(ORANGE_LED);
  nrk_led_clr(BLUE_LED);
  nrk_led_clr(GREEN_LED);
  nrk_led_clr(RED_LED);
 
  nrk_time_set(0,0);
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}

void tx_task()
{
int8_t v,i;
uint8_t len,cnt;
uint8_t ack_buf[2];
  
printf( "Gateway Tx Task PID=%u\r\n",nrk_get_pid());

  // send at startup to cope with empty network and rebooting
  ack_buf[0]='N';
  // slip_tx(ack_buf, 1);
cnt=0;

  while(1) {
  // This is simply a place holder in case you want to add Host -> Client Communication
  // v = slip_rx ( slip_rx_buf, TDMA_MAX_PKT_SIZE);
  // if (v > 0) {
       
  //      ack_buf[0]='A';
  //      nrk_kprintf (PSTR ("Sending data: "));
  //      for (i = 0; i < v; i++)
  //       printf ("%d ", slip_rx_buf[i]);
  //       printf ("\r\n");

       
  //       v=tdma_send(&tx_tdma_fd, &slip_rx_buf, v, TDMA_BLOCKING );  
  // } else ack_buf[0]='N';
  // slip_tx(ack_buf, 1);
  sprintf(&slip_rx_buf,"Test\r\n");
  len = strlen(slip_rx_buf)+1;
  v=tdma_send(&tx_tdma_fd, &slip_rx_buf, len, TDMA_BLOCKING ); 
  nrk_led_toggle(BLUE_LED);
  }
}

void rx_task()
{
nrk_time_t t;
uint16_t cnt;
int8_t v;
uint8_t len,i,chan;


cnt=0;
nrk_kprintf( PSTR("Nano-RK Version ") );
printf( "%d\r\n",NRK_VERSION );

  
printf( "Gateway Task PID=%u\r\n",nrk_get_pid());
t.secs=30;
t.nano_secs=0;

// setup a software watch dog timer
nrk_sw_wdt_init(0, &t, NULL);
nrk_sw_wdt_start(0);

//for(i=0; i<32; i++ )
//nrk_eeprom_write_byte(i,0xff);


  chan = DEFAULT_CHANNEL;
  mac_address = HOST_MAC;

  printf ("MAC ADDR: %x\r\n", mac_address & 0xffff);
  printf ("chan = %d\r\n", chan);
  len=0;
  for(i=0; i<16; i++ ) { len+=aes_key[i]; }
  printf ("AES checksum = %d\r\n", len);


tdma_init(TDMA_HOST, chan, mac_address);

// Change these parameters anytime you want...
tdma_set_slot_len_ms(10);
tdma_set_slots_per_cycle(7);
//TODO figure out what ttl_set is...
tdma_ttl_set(3);

tdma_aes_setkey(aes_key);
tdma_aes_enable();

slip_init (stdin, stdout, 0, 0);
last_seq_num=0;
last_mac=0;

while(!tdma_started()) nrk_wait_until_next_period();
  nrk_led_set(GREEN_LED);
  
while(1) {
  v=tdma_recv(&rx_tdma_fd, &slip_tx_buf, &len, TDMA_BLOCKING ); 
  nrk_led_set(ORANGE_LED);
  if(v==NRK_OK)
  {
    /*printf ("src: %u\r\nrssi: %d\r\n", rx_tdma_fd.src, rx_tdma_fd.rssi);
    printf ("slot: %u\r\n", rx_tdma_fd.slot);
    printf ("cycle len: %u\r\n", rx_tdma_fd.cycle_size);
    */
    
    printf("%u, ",rx_tdma_fd.slot); 
    for (i = 0; i < len; i++){
      char cur; 
      cur = slip_tx_buf[i]; 
      if((cur >= '0' && cur <= '9') || cur == ',' || cur == '-' || cur =='.')
        printf ("%c", cur);
    }
    printf ("\r\n");
    nrk_led_set(BLUE_LED);
            
    tdma_rx_pkt_release();
    nrk_led_clr(ORANGE_LED);
    nrk_sw_wdt_update(0);   
   }
  }
}

void
nrk_create_taskset()
{
  nrk_task_set_entry_function( &rx_task_info, rx_task);
  nrk_task_set_stk( &rx_task_info, rx_task_stack, NRK_APP_STACKSIZE);
  rx_task_info.prio = 50;
  rx_task_info.FirstActivation = TRUE;
  rx_task_info.Type = BASIC_TASK;
  rx_task_info.SchType = PREEMPTIVE;
  
  /*TODO tighten these up- spend more time in RX and way less in TX!*/
  rx_task_info.period.secs = 0;
  rx_task_info.period.nano_secs = 250*NANOS_PER_MS;
  rx_task_info.cpu_reserve.secs = 0;
  rx_task_info.cpu_reserve.nano_secs = 200*NANOS_PER_MS;//100*NANOS_PER_MS;//250*NANOS_PER_MS;
 /* 
  rx_task_info.period.secs = 0;
  rx_task_info.period.nano_secs = 250*NANOS_PER_MS;
  rx_task_info.cpu_reserve.secs = 0;
  rx_task_info.cpu_reserve.nano_secs = 500*NANOS_PER_MS;
 */
  rx_task_info.offset.secs = 0; 
  rx_task_info.offset.nano_secs= 0;
  nrk_activate_task (&rx_task_info);

  nrk_task_set_entry_function( &tx_task_info, tx_task);
  nrk_task_set_stk( &tx_task_info, tx_task_stack, NRK_APP_STACKSIZE);
  tx_task_info.prio = 1;
  tx_task_info.FirstActivation = TRUE;
  tx_task_info.Type = BASIC_TASK;
  tx_task_info.SchType = PREEMPTIVE;
  
  tx_task_info.period.secs = 0;
  tx_task_info.period.nano_secs = 250*NANOS_PER_MS;
  tx_task_info.cpu_reserve.secs = 0;
  tx_task_info.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  /*
  tx_task_info.period.secs = 1;
  tx_task_info.period.nano_secs = 250*NANOS_PER_MS;
  tx_task_info.cpu_reserve.secs = 1;
  tx_task_info.cpu_reserve.nano_secs = 500*NANOS_PER_MS;
  */
  tx_task_info.offset.secs = 0;
  tx_task_info.offset.nano_secs= 0;
  nrk_activate_task (&tx_task_info);

  tdma_task_config();

}


