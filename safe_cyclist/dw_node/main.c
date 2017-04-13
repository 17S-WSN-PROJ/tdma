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
#include <nrk_ext_int.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <pcf_tdma.h>
#include <TWI_Master.h>
#include <tdma_cons.h>
#include <dwm1000.h>

NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);

void nrk_create_taskset();

void callback(void)
{
    nrk_led_toggle(RED_LED);
}

int
main (void)
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);

  nrk_init();

  nrk_led_clr(ORANGE_LED);
  nrk_led_clr(BLUE_LED);
  nrk_led_clr(GREEN_LED);
  nrk_led_clr(RED_LED);

  dwm_sstwr_init(callback);

  nrk_time_set(0,0);
  nrk_create_taskset ();
  nrk_start();
  return 0;
}

void print_40bit(uint64_t x)
{
  uint8_t *y = (uint8_t*) &x;
  printf("%x %x %x %x %x\r\n", y[4], y[3], y[2], y[1], y[0]);
}

/* @brief send a packet with a short that increments
 */
void Task1(void)
{
    uint32_t data;
    uint64_t dwm_stat;
    dwm_rx_mode();
    while(1)
    {
        dwm_stat = dwm_get_sys_status();
        printf("status: ");print_40bit(dwm_get_sys_status());
        if ((dwm_stat >> 24)  )
        {
            dwm_sstwr_init(callback);
            dwm_rx_mode();
        }
        if ((dwm_stat >> 13 & 0xf))
        {
            dwm_sstwr_init(callback);
            dwm_rx_mode();
        }
        nrk_wait_until_next_period();
    }
}

void
nrk_create_taskset()
{
  nrk_task_set_entry_function( &TaskOne, Task1);
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 250*NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 250*NANOS_PER_MS;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

}

