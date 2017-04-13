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
#include <avr/sleep.h>
#include <hal.h>
#include <pcf_tdma.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_eeprom.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <nrk_ext_int.h>
#include <ff_basic_sensor.h>


// if SET_MAC is 0, then read MAC from EEPROM
// otherwise use the coded value
#define DEFAULT_CHANNEL  	26
#define MAC_ADDRESS		0x2	


uint8_t sbuf[4];
tdma_info tx_tdma_fd;
tdma_info rx_tdma_fd;

uint8_t rx_buf[TDMA_MAX_PKT_SIZE];
uint8_t tx_buf[TDMA_MAX_PKT_SIZE];

uint32_t mac_address;


nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);


nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();

uint8_t aes_key[] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee, 0xff};

// This function gets called in a loop if sync is lost.
// It is passed a counter indicating how long it has gone since the last synchronization.
int8_t tdma_error(uint16_t cons_err_cnt)
{

  if(tdma_sync_ok()==0 && cons_err_cnt>50 )  nrk_led_set(RED_LED);
  else nrk_led_clr(RED_LED);

  // If there has been enough cycles without sync then snooze  
  if(cons_err_cnt>400) {
	//nrk_kprintf(PSTR("Entering TDMA snooze...\r\n" ));
	nrk_wait_until_next_period();
	return NRK_OK; 
	}

return NRK_ERROR;
}

int main ()
{
  uint8_t ds;
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  nrk_time_set (0, 0);

  tdma_set_error_callback(&tdma_error);
  tdma_task_config();

  nrk_create_taskset ();
  nrk_start ();

  return 0;
}


void rx_task ()
{
	nrk_time_t t;
	uint16_t cnt;
	int8_t v;
	uint8_t len, i;
	uint8_t chan;


	cnt = 0;
	nrk_kprintf (PSTR ("Nano-RK Version "));
	printf ("%d\r\n", NRK_VERSION);


	printf ("RX Task PID=%u\r\n", nrk_get_pid ());
	t.secs = 5;
	t.nano_secs = 0;

	chan = DEFAULT_CHANNEL;
	mac_address = MAC_ADDRESS;

	printf ("MAC ADDR: %x\r\n", mac_address & 0xffff);
	printf ("chan = %d\r\n", chan);
	len=0;
	for(i=0; i<16; i++ ) { len+=aes_key[i]; }
	printf ("AES checksum = %d\r\n", len);



	tdma_init (TDMA_CLIENT, chan, mac_address);

	tdma_aes_setkey(aes_key);
	tdma_aes_enable();


	while (!tdma_started ())
	nrk_wait_until_next_period ();

	// Mask off lower byte of MAC address for TDMA slot
	v = tdma_tx_slot_add (mac_address&0xFFFF);

	if (v != NRK_OK)
	nrk_kprintf (PSTR ("Could not add slot!\r\n"));

	while (1) {
		v = tdma_recv (&rx_tdma_fd, &rx_buf, &len, TDMA_BLOCKING);
		if (v == NRK_OK) {

		printf("ActualRssi:  %d, energyDetectionLevel:  %d, linkQualityIndication:  %d\r\n",
		     rx_tdma_fd.actualRssi,
		     rx_tdma_fd.energyDetectionLevel,
		     rx_tdma_fd.linkQualityIndication);
		printf ("raw len: %u\r\nraw buf: ", len);
		for (i = 0; i < len; i++)
		printf ("%c ", rx_buf[i]);
		printf ("\r\n");
		
		}
	}	

	//  nrk_wait_until_next_period();
}


void tx_task ()
{
	uint8_t j, i, val, cnt;
	int8_t len;
	int8_t v,fd;
	nrk_sig_mask_t ret;
	nrk_time_t t;


	printf ("tx_task PID=%d\r\n", nrk_get_pid ());

	while (!tdma_started ())
		nrk_wait_until_next_period ();

	while (1) {
		nrk_led_clr(RED_LED);
		sprintf (tx_buf, "Dogrusoz Emre Node MAC: %u\n", MAC_ADDRESS);
    	len = strlen (tx_buf) + 1;
		v = tdma_send (&tx_tdma_fd, &tx_buf, len, TDMA_BLOCKING);
		if (v == NRK_OK) {
			printf("packet sent len=%d \r\n",len);
		}
		else { nrk_kprintf(PSTR("Pkt tx error\r\n")); nrk_wait_until_next_period();}
	}
}



void nrk_create_taskset ()
{


  RX_TASK.task = rx_task;
	nrk_task_set_stk (&RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 2;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
	RX_TASK.period.secs = 0;
  RX_TASK.period.nano_secs = 250* NANOS_PER_MS;
	RX_TASK.cpu_reserve.secs = 1;
  RX_TASK.cpu_reserve.nano_secs = 50* NANOS_PER_MS;
	//  RX_TASK.period.secs = 1;
  //  RX_TASK.period.nano_secs = 0;
	//  RX_TASK.cpu_reserve.secs = 2;
  //  RX_TASK.cpu_reserve.nano_secs = 0;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk (&TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 3;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  
  TX_TASK.period.secs = 0;
  TX_TASK.period.nano_secs = 250* NANOS_PER_MS;
  TX_TASK.cpu_reserve.secs = 1;
  TX_TASK.cpu_reserve.nano_secs = 50* NANOS_PER_MS;
  /*
  TX_TASK.period.secs = 1;
  TX_TASK.period.nano_secs = 0;
  TX_TASK.cpu_reserve.secs = 1;
  TX_TASK.cpu_reserve.nano_secs = 0;
  */
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);

}

