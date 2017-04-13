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
#include <TWI_Master.h>
#include <tdma_cons.h>

// Constants for the accelrometer
//There are 6 data registers, they are sequential starting 
//with the LSB of X.  We'll read all 6 in a burst and won't
//address them individually
#define ADXL345_REGISTER_XLSB 0x32
#define ADXL_REGISTER_DTFMT 0x31
//Need to set power control bit to wake up the adxl345
#define ADXL_REGISTER_PWRCTL 0x2D
#define ADXL_REGISTER_FIFOCTL 0x38
#define ADXL_FIFOCTL_STREAM 1<<7
#define ADXL_PWRCTL_MEASURE 1 << 3
#define ADXL_16G_DTFMT 0x0F
#define ADXL_PWRCTL_STBY 0
#define ADXL345_ADDRESS 0xA6
#define ADXL_SIZE 6


//Constants for the gyroscope
#define ITG3200_ADDRESS 0xD0
//request burst of 6 bytes from this address
#define ITG3200_REGISTER_XMSB 0x1D
#define ITG3200_REGISTER_DLPF 0x16
#define ITG3200_FULLSCALE 0x03 << 3
#define ITG3200_42HZ 0x03
#define ITG3200_SIZE 6


#define HMC5843_ADDRESS 0x3C
//First data address of 6 is XMSB.  Also need to set a configuration register for
//continuous measurement
#define HMC5843_REGISTER_XMSB 0x03
#define HMC5843_REGISTER_MEASMODE 0x02
#define HMC5843_MEASMODE_CONT 0x00
#define HMC5843_SIZE 6


tdma_info tx_tdma_fd;
tdma_info rx_tdma_fd;

uint8_t i2c_buf[16];
uint8_t tx_buf[TDMA_MAX_PKT_SIZE];
uint8_t pkt[TDMA_MAX_PKT_SIZE];
uint8_t tx_len;
unsigned int sequenceNo; 
bool packetReady;
uint16_t mac_address;

uint8_t aes_key[] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee, 0xff};

NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void task_imu(void);

NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type rx_task_info;
void rx_task (void);

NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
nrk_task_type tx_task_info;
void tx_task (void);


void init_adxl345(void);
void init_itg3200(void);
void init_hmc5843(void);

void nrk_create_taskset();

int main ()
{ 

  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);
  nrk_init();

  TWI_Master_Initialise();
  sei();
  init_adxl345();
  init_itg3200();
  init_hmc5843();
  /* initialize sequence number, used to sync with master */
  sequenceNo = 0; 

  /* initialize tx_buf ready flag */
  packetReady = false;
  

  mac_address = CLIENT_MAC;
  printf("mac = %d\r\n",mac_address);
  // default channel (13) for bike
  //tdma_init (TDMA_CLIENT, DEFAULT_CHANNEL, CLIENT_MAC);
  // channel 16 for car
  tdma_init (TDMA_CLIENT, 16, CLIENT_MAC);

  //tdma_aes_setkey(aes_key);
  //tdma_aes_enable();

  tdma_tx_slot_add (mac_address&0xFFFF);

  nrk_led_clr(ORANGE_LED);
  nrk_led_clr(BLUE_LED);
  nrk_led_clr(GREEN_LED);
  nrk_led_clr(RED_LED);
 
  nrk_time_set(0,0);
  nrk_create_taskset();
  nrk_start();
  
  return 0;
}

void init_itg3200() {
    /* put in standby mode while we change fifo control bits */
  i2c_buf[0] = ITG3200_ADDRESS | FALSE<<TWI_READ_BIT;
  i2c_buf[1] = ITG3200_REGISTER_DLPF;
  i2c_buf[2] = ITG3200_FULLSCALE | ITG3200_42HZ;
  TWI_Start_Transceiver_With_Data(i2c_buf, 3);
}

void init_hmc5843() {
    /* put in standby mode while we change fifo control bits */
  i2c_buf[0] = HMC5843_ADDRESS | FALSE<<TWI_READ_BIT;
  i2c_buf[1] = HMC5843_REGISTER_MEASMODE;
  i2c_buf[2] = HMC5843_MEASMODE_CONT;
  TWI_Start_Transceiver_With_Data(i2c_buf, 3);
}


void init_adxl345() {
  unsigned int read = 0;

  /* put in standby mode while we change fifo control bits */
  i2c_buf[0] = ADXL345_ADDRESS | FALSE<<TWI_READ_BIT;
  i2c_buf[1] = ADXL_REGISTER_PWRCTL;
  i2c_buf[2] = ADXL_PWRCTL_STBY;
  TWI_Start_Transceiver_With_Data(i2c_buf, 3);

  /* set the fifo mode to stream */
  i2c_buf[0] = ADXL345_ADDRESS | FALSE<<TWI_READ_BIT;
  i2c_buf[1] = ADXL_REGISTER_FIFOCTL;
  i2c_buf[2] = ADXL_FIFOCTL_STREAM;
  TWI_Start_Transceiver_With_Data(i2c_buf,3);

  /* set data format to full resolution +-16g */
  i2c_buf[0] = ADXL345_ADDRESS | FALSE<<TWI_READ_BIT;
  i2c_buf[1] = ADXL_REGISTER_DTFMT;
  i2c_buf[2] = ADXL_16G_DTFMT;
  TWI_Start_Transceiver_With_Data(i2c_buf,3);

  /* set to measure mode */
  i2c_buf[0] = ADXL345_ADDRESS | FALSE<<TWI_READ_BIT;
  i2c_buf[1] = ADXL_REGISTER_PWRCTL;
  i2c_buf[2] = ADXL_PWRCTL_MEASURE;
  TWI_Start_Transceiver_With_Data(i2c_buf, 3);
}

void task_imu(){
  unsigned int i;
  unsigned int count;
  int v;
  uint8_t datagram,cycles;
 
  cycles=0;
  datagram=0;
 
  while (!tdma_started())
    nrk_wait_until_next_period ();

  while(1){
    i = 0;
    tx_buf[i++] = NODE_ADDR;
    tx_buf[i++] = sequenceNo++;
    
    i2c_buf[0] = (ADXL345_ADDRESS) | (FALSE<<TWI_READ_BIT);
    i2c_buf[1] = ADXL345_REGISTER_XLSB;
    TWI_Start_Transceiver_With_Data(i2c_buf, 2);


    /* Read first byte */
    i2c_buf[0] = (ADXL345_ADDRESS) | (TRUE<<TWI_READ_BIT);

    TWI_Start_Transceiver_With_Data(i2c_buf, 7);
    TWI_Get_Data_From_Transceiver(i2c_buf,7);
    for (count = 0; count < ADXL_SIZE; count++){
      tx_buf[i++] = i2c_buf[count+1];
    }

    i2c_buf[0] = (ITG3200_ADDRESS) | (FALSE<<TWI_READ_BIT);
    i2c_buf[1] = ITG3200_REGISTER_XMSB;
    TWI_Start_Transceiver_With_Data(i2c_buf, 2);

    /* Read first byte */
    i2c_buf[0] = (ITG3200_ADDRESS) | (TRUE<<TWI_READ_BIT);
    TWI_Start_Transceiver_With_Data(i2c_buf, 7);
    TWI_Get_Data_From_Transceiver(i2c_buf,7);
    for (count = 0; count < ITG3200_SIZE; count++){
      tx_buf[i++] = i2c_buf[count+1];
    }

    i2c_buf[0] = (HMC5843_ADDRESS) | (FALSE<<TWI_READ_BIT);
    i2c_buf[1] = HMC5843_REGISTER_XMSB;
    TWI_Start_Transceiver_With_Data(i2c_buf, 2);

    /* Read first byte */
    i2c_buf[0] = (HMC5843_ADDRESS) | (TRUE<<TWI_READ_BIT);
    TWI_Start_Transceiver_With_Data(i2c_buf, 7);
    TWI_Get_Data_From_Transceiver(i2c_buf,7);
    for (count = 0; count < HMC5843_SIZE; count++){
      tx_buf[i++] = i2c_buf[count+1];
    }
    tx_len = i;
    packetReady = false;
  

    datagram=tx_len;


    if(cycles>1)
    {
    // Copy second half of packet to front
    // This takes the datagram inserted on the last cycle
    // and makes it first.
    	for (int i = 0; i < tx_len; i++)
    		pkt[i]=pkt[i+datagram];

    // Add new datagram the end of the packet
    	for (int i = 0; i < tx_len; i++)
    		pkt[i+datagram]=tx_buf[i];
	// second to last byte is the number of datagrams in the packet
	// last byte is the size of the datagram 
	pkt[i+datagram]=2;
	pkt[i+datagram+1]=datagram;
	tx_len+=2;
    	tx_len=tx_len+datagram;

    } else
	{
	// If there is only 1 datagram just pack in 1
    	for (int i = 0; i < tx_len; i++)
    		pkt[i]=tx_buf[i];
	// second to last byte is the number of datagrams in the packet
	// last byte is the size of the datagram 
	pkt[i]=1;
	pkt[i+1]=datagram;
	tx_len+=2;
	cycles++;
	}
    packetReady = true;
     
    v = tdma_send (&tx_tdma_fd, &pkt, tx_len, TDMA_BLOCKING);
  }
}


void tx_task ()
{
  int8_t v;
  uint8_t cnt;
  nrk_time_t t;


  printf ("Tx Task PID=%u\r\n", nrk_get_pid ());
  t.secs = 5;
  t.nano_secs = 0;

  // // setup a software watch dog timer
  nrk_sw_wdt_init(0, &t, NULL);
  nrk_sw_wdt_start(0);



  cnt = 0;

  while (1) {
    // Update watchdog timer
    nrk_sw_wdt_update(0);
    nrk_led_set(RED_LED);

    // if sensor data hasn't been gathered yet
    if (!packetReady){
       continue;
    }
    nrk_led_clr(RED_LED);
    v = tdma_send (&tx_tdma_fd, &pkt, tx_len, TDMA_BLOCKING);
    v = tdma_send (&tx_tdma_fd, &pkt, tx_len, TDMA_BLOCKING);
    v = tdma_send (&tx_tdma_fd, &pkt, tx_len, TDMA_BLOCKING);
    }
}



void
nrk_create_taskset()
{
  nrk_task_set_entry_function( &TaskOne, task_imu);
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 25 * NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 0;
  TaskOne.offset.secs = 1;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);
/*
  nrk_task_set_entry_function (&tx_task_info, tx_task);
  nrk_task_set_stk (&tx_task_info, tx_task_stack, NRK_APP_STACKSIZE);
  tx_task_info.prio = 1;
  tx_task_info.FirstActivation = TRUE;
  tx_task_info.Type = BASIC_TASK;
  tx_task_info.SchType = PREEMPTIVE;
  tx_task_info.period.secs = 0;
  tx_task_info.period.nano_secs = 0 * NANOS_PER_MS;
  tx_task_info.cpu_reserve.secs = 0;
  tx_task_info.cpu_reserve.nano_secs = 0 * NANOS_PER_MS;
  tx_task_info.offset.secs = 1;
  tx_task_info.offset.nano_secs = 0;
  nrk_activate_task (&tx_task_info);
*/
  tdma_task_config ();
}



