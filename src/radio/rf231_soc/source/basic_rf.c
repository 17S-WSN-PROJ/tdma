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
*  Maxim Buevich
*  Anthony Rowe
*******************************************************************************/

#include <include.h>
#include <basic_rf.h>
#include <ulib.h>
#include <nrk.h>
#include <nrk_events.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_cpu.h>
#include <avr/wdt.h>

#define OSC_STARTUP_DELAY	1000
//#define RADIO_CC2591
//#define GLOSSY_TESTING

//#define RADIO_VERBOSE
#ifdef RADIO_VERBOSE
	#define vprintf(...)		printf(__VA_ARGS__)
#else
	#define vprintf(...) 	
#endif


typedef struct ieee_mac_fcf{
	uint8_t frame_type: 		3;
	uint8_t sec_en: 				1;
	uint8_t frame_pending: 	1;
	uint8_t ack_request: 		1;
	uint8_t intra_pan: 			1;

	uint8_t res:						3;	

	uint8_t dest_addr_mode:	2;
	uint8_t frame_version: 	2;
	uint8_t src_addr_mode:	2;
} ieee_mac_fcf_t;

typedef struct ieee_mac_frame_header{	
	ieee_mac_fcf_t fcf;
	uint8_t seq_num;

	uint16_t dest_pan_id;
	uint16_t dest_addr;
	//uint16_t src_pan_id;
	uint16_t src_addr;
	/*uint16_t sec_header; */
} ieee_mac_frame_header_t;



static void rf_cmd(uint8_t cmd);
void rf_glossy_interrupt();

#ifdef GLOSSLY_TESTING
	void clear_packet_flags();
#endif

nrk_sem_t *radio_sem;
//uint8_t auto_ack_enable;
//uint8_t security_enable;
//uint8_t last_pkt_encrypted;
uint16_t mdmctrl0;
uint8_t tx_ctr[6];
uint8_t rx_ctr[4];

volatile RF_SETTINGS rfSettings;
uint8_t rf_ready;
volatile uint8_t rx_ready;
volatile uint8_t tx_done;
uint8_t use_glossy;


volatile void (*rx_start_func)(void) = 0;
volatile void (*rx_end_func)(void) = 0;

/* AES encryption and decryption key buffers */
uint8_t ekey[16];
uint8_t dkey[16];

/* wireless programming reset */
uint8_t wireless_prog = 0;
uint8_t reset_val[] = {0x43, 0x15, 0xa6, 0xd9, 0x3d, 0x31, 0x82, 0xf1, 0x8c, 0xa7, 0x4f, 0xc5, 0x99, 0x97, 0x04, 0xac};


void set_wireless_prog(uint8_t val)
{
   wireless_prog = val;
}

void rf_enable_glossy()
{
	use_glossy = 1;
}

void rf_disable_glossy()
{
	use_glossy = 0;
}

void rf_power_down()
{
	uint8_t status;

	while((TRX_STATUS & 0x1F) == STATE_TRANSITION_IN_PROGRESS)
		continue;

	/* For some reason comparing to SLEEP doesn't work, but 0 does */
	status = (TRX_STATUS & 0x1F);
	if((status == 0) || (status == 0xF))
		return;
	/* Disable TRX if it is enabled */
	if((TRX_STATUS & 0x1F) != TRX_OFF){
		rf_cmd(TRX_OFF);
		do{
			status = (TRX_STATUS & 0x1F);
		}while(status != TRX_OFF);
	}

	TRXPR |= (1 << SLPTR);
	do{
		status = (TRX_STATUS & 0x1F);
	}while((status != 0) && (status != 0xF));
}

void rf_power_up()
{
	uint8_t status;

	while((TRX_STATUS & 0x1F) == STATE_TRANSITION_IN_PROGRESS)
		continue;
	/* For some reason comparing to SLEEP doesn't work, but 0 does */
	status = (TRX_STATUS & 0x1F);
	if((status != 0) && (status != 0xF))
		return;

	/* Wake up */
	TRXPR &= ~(1 << SLPTR);
	while((TRX_STATUS & 0x1F) != TRX_OFF)
		continue;
}


/* Safely change the radio state */
static void rf_cmd(uint8_t cmd)
{
	while((TRX_STATUS & 0x1F) == STATE_TRANSITION_IN_PROGRESS)
		continue;
	TRX_STATE = cmd;
}

void rf_pll_on()
{
   rf_cmd(PLL_ON);
}


void rf_tx_power(uint8_t pwr)
{
	PHY_TX_PWR &= 0xF0;
	PHY_TX_PWR |= (pwr & 0xF);
}

void rf_addr_decode_enable()
{
	XAH_CTRL_1 &= ~(1 << AACK_PROM_MODE);
}


void rf_addr_decode_disable()
{
	XAH_CTRL_1 |= (1 << AACK_PROM_MODE);
}


void rf_auto_ack_enable()
{
	CSMA_SEED_1 &= ~(1 << AACK_DIS_ACK);
}

void rf_auto_ack_disable()
{
	CSMA_SEED_1 |= (1 << AACK_DIS_ACK);
}


void rf_addr_decode_set_my_mac(uint16_t my_mac)
{
	/* Set short MAC address */
	SHORT_ADDR_0 = (my_mac & 0xFF); 
	SHORT_ADDR_1 = (my_mac >> 8);
	rfSettings.myAddr = my_mac;
}


void rf_set_rx(RF_RX_INFO *pRRI, uint8_t channel )
{
	rfSettings.pRxInfo = pRRI;
	PHY_CC_CCA &= ~(0x1F);
	PHY_CC_CCA |= (channel << CHANNEL0);
}

void rx_start_callback(void (*func)(void)){
	rx_start_func = func;
}

void rx_end_callback(void (*func)(void)){
	rx_end_func = func;
}


void rf_init(RF_RX_INFO *pRRI, uint8_t channel, uint16_t panId, uint16_t myAddr)
{ 
/*
	 uint8_t n;
   int8_t v;

#ifdef RADIO_PRIORITY_CEILING
    radio_sem = nrk_sem_create(1,RADIO_PRIORITY_CEILING);
    if (radio_sem == NULL)
      nrk_kernel_error_add (NRK_SEMAPHORE_CREATE_ERROR, nrk_get_pid ());

  v = nrk_sem_pend (radio_sem);
  if (v == NRK_ERROR) {
    nrk_kprintf (PSTR ("CC2420 ERROR:  Access to semaphore failed\r\n"));
  }
#endif
	

#ifdef RADIO_PRIORITY_CEILING
  v = nrk_sem_post (radio_sem);
  if (v == NRK_ERROR) {
    nrk_kprintf (PSTR ("CC2420 ERROR:  Release of semaphore failed\r\n"));
    _nrk_errno_set (2);
  }
#endif

*/


	/* Turn on auto crc calculation */
	TRX_CTRL_1 = (1 << TX_AUTO_CRC_ON);
	/* Set PA buffer lead time to 6 us and TX power to 3.0 dBm (maximum) */
	PHY_TX_PWR = (1 << PA_BUF_LT1) | (1 << PA_BUF_LT0) | (0 << TX_PWR0);
	/* CCA Mode and Channel selection */
	PHY_CC_CCA = (0 << CCA_MODE1) | (1 << CCA_MODE0) | (channel << CHANNEL0);
	/* Set CCA energy threshold */
	CCA_THRES = 0xC5;
	/* Start of frame delimiter */
	SFD_VALUE = 0xA7;
	/* Dynamic buffer protection on and data rate is 250 kb/s */
	TRX_CTRL_2 = (1 << RX_SAFE_MODE) | (0 << OQPSK_DATA_RATE1) | (0 << OQPSK_DATA_RATE0);
	
	/* Set short MAC address */
	SHORT_ADDR_0 = (myAddr & 0xFF); SHORT_ADDR_1 = (myAddr >> 8);
	/* Set PAN ID */
	PAN_ID_0 = (panId & 0xFF); PAN_ID_1 = (panId >> 8);
	
	/* 2-bit random value generated by radio hardware */
	#define RADIO_RAND ((PHY_RSSI >> RND_VALUE0) & 0x3)
	/* Set random csma seed */
	CSMA_SEED_0 = (RADIO_RAND << 6) | (RADIO_RAND << 4) 
			| (RADIO_RAND << 2) | (RADIO_RAND << 0);
	/* Will ACK received frames with version numbers of 0 or 1 */
	CSMA_SEED_1 = (0 << AACK_FVN_MODE1) | (1 << AACK_FVN_MODE0) 
			| (RADIO_RAND << CSMA_SEED_11) | (RADIO_RAND << CSMA_SEED_10);

	/* don't re-transmit frames or perform cca multiple times, slotted op is off */
	XAH_CTRL_0 = (0 << MAX_FRAME_RETRIES0) | (0 << MAX_CSMA_RETRIES0)
			| (0 << SLOTTED_OPERATION);
   
   /* Enable all radio interrupts */
	IRQ_MASK = (1 << AWAKE_EN) | (1 << TX_END_EN) | (1 << AMI_EN) | (1 << CCA_ED_DONE_EN)
			| (1 << RX_END_EN) | (1 << RX_START_EN) | (1 << PLL_UNLOCK_EN) | (1 << PLL_LOCK_EN);

	/* Initialize settings struct */
	rfSettings.pRxInfo = pRRI;
	rfSettings.txSeqNumber = 0;
	rfSettings.ackReceived = 0;
	rfSettings.panId = panId;
	rfSettings.myAddr = myAddr;
	rfSettings.receiveOn = 0;

	rf_ready = 1;
   rx_ready = 0;
   tx_done = 0;

	use_glossy = 0;

} // rf_init() 


//-------------------------------------------------------------------------------------------------------
//  void rf_rx_on(void)
//
//  DESCRIPTION:
//      Enables the CC2420 receiver and the FIFOP interrupt. When a packet is received through this
//      interrupt, it will call rf_rx_callback(...), which must be defined by the application
//-------------------------------------------------------------------------------------------------------
void rf_rx_on(void) 
{
/*
#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_pend (radio_sem);
#endif
	rfSettings.receiveOn = TRUE;

#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_post(radio_sem);
#endif
*/

#ifdef RADIO_CC2591
	rf_cc2591_rx_on();
#endif
#ifdef GLOSSY_TESTING
	clear_packet_flags();
#endif
	rf_cmd(RX_AACK_ON);
}

void rf_polling_rx_on(void)
{
/*#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_pend (radio_sem);
#endif
  rfSettings.receiveOn = TRUE;


#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_post(radio_sem);
#endif
*/

#ifdef RADIO_CC2591
	rf_cc2591_rx_on();
#endif

	rf_cmd(RX_AACK_ON);
}


//-------------------------------------------------------------------------------------------------------
//  void rf_rx_off(void)
//
//  DESCRIPTION:
//      Disables the CC2420 receiver and the FIFOP interrupt.
//-------------------------------------------------------------------------------------------------------
void rf_rx_off(void)
{
/*
#ifdef RADIO_PRIORITY_CEILING
  nrk_sem_pend (radio_sem);
#endif
	// XXX
  //SET_VREG_INACTIVE();	
	rfSettings.receiveOn = FALSE;

#ifdef RADIO_PRIORITY_CEILING
  nrk_sem_post(radio_sem);
#endif
  //	DISABLE_FIFOP_INT();
*/
   rf_cmd(TRX_OFF);
   rx_ready = 0;
}



//-------------------------------------------------------------------------------------------------------
//  BYTE rf_tx_packet(RF_TX_INFO *pRTI)
//
//  DESCRIPTION:
//		Transmits a packet using the IEEE 802.15.4 MAC data packet format with short addresses. CCA is
//		measured only once before packet transmission (not compliant with 802.15.4 CSMA-CA).
//		The function returns:
//			- When pRTI->ackRequest is FALSE: After the transmission has begun (SFD gone high)
//			- When pRTI->ackRequest is TRUE: After the acknowledgment has been received/declared missing.
//		The acknowledgment is received through the FIFOP interrupt.
//
//  ARGUMENTS:
//      RF_TX_INFO *pRTI
//          The transmission structure, which contains all relevant info about the packet.
//
//  RETURN VALUE:
//		uint8_t
//			Successful transmission (acknowledgment received)
//-------------------------------------------------------------------------------------------------------


uint8_t rf_tx_packet(RF_TX_INFO *pRTI)
{
	/*
	#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_pend(radio_sem);
	#endif

	#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_post(radio_sem);
	#endif
	//return success;
	*/

	uint8_t trx_status, trx_error, *data_start, *frame_start = &TRXFBST;
	uint16_t i;

	if(!rf_ready) 
		return NRK_ERROR;
	
	ieee_mac_frame_header_t *machead = frame_start + 1;
	ieee_mac_fcf_t fcf;

	/* TODO: Setting FCF bits is probably slow. Optimize later. */
	fcf.frame_type = 1;
	fcf.sec_en = 0;
	fcf.frame_pending = 0;
	fcf.ack_request = pRTI->ackRequest;
	fcf.intra_pan = 1;
	fcf.res = 0;
	fcf.dest_addr_mode = 2;
	fcf.frame_version = 0;
	fcf.src_addr_mode = 2;
	
	/* Build the rest of the MAC header */
	rfSettings.txSeqNumber++;
	machead->fcf = fcf;
	if (use_glossy) {
		machead->seq_num = 0xFF;
		machead->src_addr = 0xAAAA;
		machead->dest_addr = 0xFFFF;
		machead->dest_pan_id = (PAN_ID_1 << 8) | PAN_ID_0;
	} else {
		machead->seq_num = rfSettings.txSeqNumber;
		machead->src_addr = (SHORT_ADDR_1 << 8) | SHORT_ADDR_0;
		machead->dest_addr = pRTI->destAddr;
		machead->dest_pan_id = (PAN_ID_1 << 8) | PAN_ID_0;
	}
	//machead->src_pan_id = (PAN_ID_1 << 8) | PAN_ID_0;
	
	/* Copy data payload into packet */
	data_start = frame_start + sizeof(ieee_mac_frame_header_t) + 1;
	memcpy(data_start, pRTI->pPayload, pRTI->length);
	/* Set the size of the packet */
	*frame_start = sizeof(ieee_mac_frame_header_t) + pRTI->length + 2;
	
	vprintf("packet length: %d bytes\r\n", *frame_start);

	/* Wait for radio to be in a ready state */
	do{
		trx_status = (TRX_STATUS & 0x1F);
	}while((trx_status == BUSY_TX) || (trx_status == BUSY_RX)
			|| (trx_status == BUSY_RX_AACK) || (trx_status == BUSY_TX_ARET)
			|| (trx_status == STATE_TRANSITION_IN_PROGRESS));
	
	/* Return error if radio not in a tx-ready state */
	if((trx_status != TRX_OFF) && (trx_status != RX_ON) 
			&& (trx_status != RX_AACK_ON) && (trx_status != PLL_ON)){
		return NRK_ERROR;
	}

	rf_cmd(RX_AACK_ON);

	/* Perform CCA if requested */
	if(pRTI->cca){
		PHY_CC_CCA |= (1 << CCA_REQUEST);
		while(!(TRX_STATUS & (1 << CCA_DONE)))
			continue;
		if(!(TRX_STATUS & (1 << CCA_STATUS)))
			return NRK_ERROR;
	}

	rf_cmd(PLL_ON);
	if(pRTI->ackRequest)
		rf_cmd(TX_ARET_ON);
	
#ifdef RADIO_CC2591
		rf_cc2591_tx_on();
#endif

   tx_done = 0;
   // Send packet. 0x2 is equivalent to TX_START
   rf_cmd(0x2);
   for(i=0; (i<65000) && !tx_done; i++){
      continue;
   }

   /* note error if ACK requested and not received */
	trx_error = ((pRTI->ackRequest && 
			(((TRX_STATE >> TRAC_STATUS0) & 0x7) != 0))
			|| (i == 65000)) ? NRK_ERROR : NRK_OK;
	rf_cmd(trx_status);

#ifdef RADIO_CC2591
	if (trx_error == NRK_ERROR) rf_cc2591_rx_on();
#endif

	return trx_error;
}


/* Resends the packet in the buffer */
uint8_t rf_tx_packet_resend()
{
   uint8_t trx_error;
   uint16_t i;

   tx_done = 0;
   // Send packet. 0x2 is equivalent to TX_START
   rf_cmd(0x2);
   for(i=0; (i<65000) && !tx_done; i++)
      continue;
   trx_error = (i == 65000) ? NRK_ERROR : NRK_OK;

   return trx_error;
}


/* Returns 1 if the channel is clear
 * Returns 0 if the channel is being used
 */
int8_t rf_cca_check()
{
	uint8_t trx_status, cca_value;

	if(!rf_ready)
		return NRK_ERROR;

	/* Wait for radio to be in a ready state */
	do{
		trx_status = (TRX_STATUS & 0x1F);
	}while((trx_status == BUSY_TX) || (trx_status == BUSY_RX)
			|| (trx_status == BUSY_RX_AACK) || (trx_status == BUSY_TX_ARET)
			|| (trx_status == STATE_TRANSITION_IN_PROGRESS)); 

	/* Return error if radio not in a tx-ready state */
	if((trx_status != TRX_OFF) && (trx_status != RX_ON) 
			&& (trx_status != RX_AACK_ON))
		return NRK_ERROR;
	
	rf_cmd(RX_AACK_ON);

	PHY_CC_CCA |= (1 << CCA_REQUEST);
	while(!(TRX_STATUS & (1 << CCA_DONE)))
		continue;
	cca_value = (TRX_STATUS & (1 << CCA_STATUS)) ? 1 : 0;
	rf_cmd(trx_status);

	return cca_value;
}



int8_t rf_rx_packet_nonblock()
{
	/*
	#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_pend(radio_sem);
	#endif
	
	#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_post(radio_sem);
	#endif
	*/
	
	uint8_t *frame_start = &TRXFBST;

	if(!rf_ready)
		return NRK_ERROR;

   if(!rx_ready)
      return 0;
   else if((TST_RX_LENGTH - 2) > rfSettings.pRxInfo->max_length)
		return NRK_ERROR;


	ieee_mac_frame_header_t *machead = frame_start;

	rfSettings.pRxInfo->seqNumber = machead->seq_num;
	rfSettings.pRxInfo->srcAddr = machead->src_addr;
	rfSettings.pRxInfo->length = TST_RX_LENGTH - sizeof(ieee_mac_frame_header_t) - 2;

	if((rfSettings.pRxInfo->length > rfSettings.pRxInfo->max_length)
			|| (rfSettings.pRxInfo->length < 0)){
		rx_ready = 0;
      TRX_CTRL_2 &= ~(1 << RX_SAFE_MODE);
		TRX_CTRL_2 |= (1 << RX_SAFE_MODE);
		return NRK_ERROR;
	}

	memcpy(rfSettings.pRxInfo->pPayload, frame_start 
			+ sizeof(ieee_mac_frame_header_t), rfSettings.pRxInfo->length);

   /* if reset packet received, perform reset */
   if(wireless_prog && (rfSettings.pRxInfo->length == 16)){
      if(strncmp(reset_val, rfSettings.pRxInfo->pPayload, 4) == 0){
         if(strncmp(reset_val, rfSettings.pRxInfo->pPayload, 16) == 0){
            wdt_enable(WDTO_500MS);
            nrk_led_set(0);
            nrk_led_set(1);
            nrk_led_set(2);
            nrk_led_set(3);
            while(1)
               continue;
         }
      }
   }

	/* I am assuming that ackRequest is supposed to
	 * be set, not read, by rf_basic */
	rfSettings.pRxInfo->ackRequest = machead->fcf.ack_request;
	//rfSettings.pRxInfo->rssi = *(frame_start + TST_RX_LENGTH);
	rfSettings.pRxInfo->rssi = PHY_ED_LEVEL;
	rfSettings.pRxInfo->actualRssi = PHY_RSSI >> 3;
	rfSettings.pRxInfo->energyDetectionLevel = PHY_ED_LEVEL;
	rfSettings.pRxInfo->linkQualityIndication = *(frame_start + TST_RX_LENGTH);

	/* Reset frame buffer protection */
	rx_ready = 0;
   TRX_CTRL_2 &= ~(1 << RX_SAFE_MODE);
	TRX_CTRL_2 |= (1 << RX_SAFE_MODE);

	return NRK_OK;
}


SIGNAL(TRX24_RX_END_vect)
{	
	uint8_t i, *byte_ptr = &TRXFBST;

	/* Verbose mode print block */
	vprintf("RX_END IRQ!\r\n");	
	for(i=0; i<TST_RX_LENGTH; i++){
		vprintf("0x%02x ", byte_ptr[i]);
		if(((i+1) % 16) == 0)
			vprintf("\r\n");
	}
	vprintf("\r\n");

   if((PHY_RSSI >> RX_CRC_VALID) & 0x1){
      rx_ready = 1;
   } else {
      printf("RX end failed checksum!\r\n");
   }
   IRQ_STATUS = (1 << RX_END);
	
	if((PHY_RSSI >> RX_CRC_VALID) & 0x1) {
		if (use_glossy) rf_glossy_interrupt();
	}

	if(rx_end_func)
		rx_end_func();

	return;
}


/* These interrupt handlers are useful for finding
 * out the exact order of events during a transmission */

SIGNAL(TRX24_AWAKE_vect)
{
	vprintf("RADIO AWAKE IRQ!\r\n");
	IRQ_STATUS = (1 << AWAKE);

	return;
}

SIGNAL(TRX24_TX_END_vect)
{
	vprintf("TX_END IRQ!\r\n");
	tx_done = 1;
   IRQ_STATUS = (1 << TX_END);

#ifdef RADIO_CC2591
	rf_cc2591_rx_on();
#endif

	return;
}

SIGNAL(TRX24_XAH_AMI_vect)
{
	vprintf("AMI IRQ!\r\n");
	IRQ_STATUS = (1 << AMI);

	return;
}

SIGNAL(TRX24_CCA_ED_DONE_vect)
{
	vprintf("CCA_ED_DONE IRQ!\r\n");
	IRQ_STATUS = (1 << CCA_ED_DONE);

	return;
}

SIGNAL(TRX24_RX_START_vect)
{
	vprintf("RX_START IRQ!\r\n");
	IRQ_STATUS = (1 << RX_START);

	if(rx_start_func)
		rx_start_func();

	return;
}

SIGNAL(TRX24_PLL_UNLOCK_vect)
{
	vprintf("PLL_UNLOCK IRQ!\r\n");
	IRQ_STATUS = (1 << PLL_UNLOCK);

	return;
}

SIGNAL(TRX24_PLL_LOCK_vect)
{
	vprintf("PLL_LOCK IRQ!\r\n");
	IRQ_STATUS = (1 << PLL_LOCK);

	return;
}



void rf_set_cca_thresh(int8_t t)
{
	CCA_THRES &= 0xF0;
	CCA_THRES |= (t & 0xF);
	return;
}


// Returns 1 if the last packet was encrypted, 0 otherwise
uint8_t rf_security_last_pkt_status()
{
	//return last_pkt_encrypted;
	return NRK_ERROR;
}


void rf_security_set_ctr_counter(uint8_t *counter)
{
	return;
}


void rf_security_set_key(uint8_t *key)
{
	return;
}



void rf_security_disable()
{
	return;
}



/**********************************************************
 ******************* NOT IMPLEMENTED **********************
 **********************************************************/


uint8_t rf_tx_tdma_packet(RF_TX_INFO *pRTI, uint16_t slot_start_time, uint16_t tx_guard_time) {
//    return success;
	return NRK_ERROR;
}


nrk_sem_t* rf_get_sem()
{
return radio_sem;
}



inline void rf_flush_rx_fifo()
{
}

uint8_t rf_busy()
{
//return SFD_IS_1;
return 1;
}

/* Implement */
uint8_t rf_rx_check_fifop()
{
//return FIFOP_IS_1;
return 1;
}


uint8_t rf_rx_check_sfd()
{
//return SFD_IS_1;
return 1;
}



/**********************************************************
 * start sending a carrier pulse
 * assumes wdrf_radio_test_mode() was called before doing this
 */
void rf_carrier_on()
{
/*
#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_pend(radio_sem);
#endif
     
#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_post(radio_sem);
#endif
*/
}



/**********************************************************
 * stop sending a carrier pulse; set the radio to idle state
 */
void rf_carrier_off()
{
/*
#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_pend(radio_sem);
#endif
     
#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_post(radio_sem);
#endif
*/
}



void rf_test_mode()
{
/*
#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_pend(radio_sem);
#endif

#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_post(radio_sem);
#endif
*/
}


/**********************************************************
 * set the radio into "normal" mode (buffered TXFIFO) and go into (data) receive */
void rf_data_mode()
{
/*
#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_pend(radio_sem);
#endif

#ifdef RADIO_PRIORITY_CEILING
	nrk_sem_post(radio_sem);
#endif
*/
}





/**********************************************************
 * Set the radio into serial unbuffered RX mode
 * RX data is received through sampling the FIFO pin, timing is done using FIFOP 
 * Use rf_rx_on() to start rcv, then wait for SFD / FIFOP. Sample during each high edge of FIFOP
 * This can be undone by using rf_data_mode()
 */
void rf_rx_set_serial()
{
}

/**********************************************************
 * Put the radio in serial TX mode, where data is sampled from the FIFO
 * pin to send after SFD, and timing is done using FIFOP
 * use rf_carrier_on() to start, set FIFO to first bit, then wait for it
 * to go up and down, then set next bit etc.
 * NOTE: You must set the FIFO pin to output mode in order to do this!
 * This can be undone by calling rf_data_mode()
 */
void rf_tx_set_serial()
{
}

/**********************************************************
 * Specifies the number of symbols to be part of preamble
 * arg is equal to number of bytes - 1.
 * (3 bytes is 802.15.4 compliant, so length arg would be 2)
 * Length arg supports values 0 to 15. See the datasheet of course for more details
 */
void rf_set_preamble_length(uint8_t length)
{
}


void rf_set_cca_mode(uint8_t mode)
{
}


void rf_cc2591_tx_on()
{
/*
	DPDS1	|= 0x3; 
	DDRG	|= 0x1;
	PORTG	|= 0x1;
	DDRE	|= 0xE0;
	PORTE	|= 0xE0;
*/
	
	DDRG	&= ~(0x1);  // Set RXTX as input
	DDRE |= 0xE0;
	// PAEN=1  EN=0  HGM=x RXTX=NC
	// PAEN = PE.7
	// EN = PE.6
	// HGM= PE.5
	PORTE |= 0xA0;


}

void rf_cc2591_rx_on()
{
/*
	DPDS1	|= 0x3; 
	DDRG	|= 0x1;
	PORTG	&= ~(0x1);
	DDRE	|= 0xE0;
	PORTE	|= 0xE0;
*/
	DDRG	&= ~(0x1);  // Set RXTX as input
	DDRE |= 0xE0;
	// PAEN=0  EN=1  HGM=1 RXTX=NC
	// PAEN = PE.7
	// EN = PE.6
	// HGM= PE.5
	PORTE |= 0x60;

}


#ifdef GLOSSY_TESTING
uint8_t packet_flags[128];
uint8_t delay_flags[26];
uint16_t curr_count;

int calculate_checksum(uint8_t * buf) {
	uint8_t i, checksum = 0;
	for (i = 1; i < 99; i++) checksum ^= buf[i];
	return checksum == buf[99];
}

uint16_t calculate_reception_rate()
{
	uint8_t i, j;
	uint16_t packets_received = 0;
	for (i = 0; i < 128; i++) {
		for (j = 0; j < 8; j++) {
			packets_received += (packet_flags[i] >> j) & 1;
		}
	}
	return packets_received;
}

void mark_delay(uint8_t delay)
{
	delay_flags[delay/8] |= 1 << (delay % 8);
}

uint8_t check_delay(uint8_t delay)
{
	return ((delay_flags[delay/8]) >> (delay % 8)) & 1;
}

void mark_recieved(uint16_t packet_id, uint8_t delay)
{
	if (check_delay(delay)) return;
	if (packet_id > 999) {
		mark_delay(delay);
		uint16_t reception_rate = calculate_reception_rate();
		printf("\n\n%4d\t%4d\r\n\n\n", delay, reception_rate);
		clear_packet_flags();
	} else {
		packet_flags[packet_id/8] |= 1 << (packet_id % 8);
	}
}
void clear_packet_flags()
{
	int i;
	for (i = 0; i < 128; i++) {
		packet_flags[i] = 0;
	}
	curr_count = 0;
}
#endif

void rf_glossy_interrupt()
{
	nrk_int_disable();

	RF_RX_INFO rfRxInfo = *(rfSettings.pRxInfo);
	RF_TX_INFO rfTxInfo;

	/* Grab packet */
	int err = rf_rx_packet_nonblock();
	if (err < 1) {
		printf("rf_rx_packet_nonblock failed]\r\n");
		nrk_int_enable();
		return;
	}

	/* TTL should be the first byte of the payload */
	uint8_t ttl = rfRxInfo.pPayload[0];
	if (ttl == 0) {
#ifndef GLOSSY_TESTING
		printf("Packet is done bouncing around!\r\n\n");
#endif
		nrk_int_enable();
		return;
	} else if (ttl == 5) {
#ifndef GLOSSY_TESTING
		printf("\n");
#endif
	}

	/* Print packet information */
#ifndef GLOSSY_TESTING
	int8_t rssi = rfRxInfo.rssi;
	uint8_t snum = rfRxInfo.seqNumber;
	printf("SEQ:%4u\tTTL:%2d\tRSSI:%4d\tPayload: [%s]\r\n", 
					snum, ttl, rssi, rfRxInfo.pPayload + 1);
#endif
	/* Copy pointer to payload and length */
	rfTxInfo.pPayload = rfRxInfo.pPayload;
	rfTxInfo.pPayload[0] = ttl - 1;
	rfTxInfo.length = rfRxInfo.length;
	rfTxInfo.cca = 0;
	rfTxInfo.ackRequest = 0;
	rfTxInfo.destAddr = 0xFFFF;

	/* Code for testing glossy reception rate*/
#ifdef GLOSSY_TESTING
	uint16_t send_num = (((uint16_t)rfRxInfo.pPayload[1]) << 8) + rfRxInfo.pPayload[2];
	uint8_t us_jam = rfRxInfo.pPayload[3];
	if (calculate_checksum(rfRxInfo.pPayload)) {
		mark_recieved(send_num, us_jam);
		if (send_num < 1000) {
			curr_count++;
			printf("SEND NUM   %04d   %04d\r", send_num, curr_count);
			fflush(stdout);
			//nrk_spin_wait_us(((uint16_t)us_jam) * 10);
		}
		nrk_int_enable();
		rf_tx_packet(&rfTxInfo);
	} else {
		nrk_int_enable();
	}
#else
	nrk_int_enable();
	rf_tx_packet(&rfTxInfo);
#endif
	return;
}


/* AES encryption and decryption */

void aes_setkey(uint8_t *key)
{
   uint8_t i;

   for(i=0; i<16; i++){
      ekey[i] = key[i];
      AES_KEY = key[i];
   }
   for(i=0; i<16; i++){
      AES_STATE = 0x00;
   }
   AES_CTRL = (1 << AES_REQUEST);

   while(!(AES_STATUS & (1 << AES_DONE))){
      continue;
   }
   for(i=0; i<16; i++){
      dkey[i] = AES_KEY;
   }
}


uint8_t aes_encrypt(uint8_t *data, uint8_t len)
{
   uint8_t i, j;

   if(len==0 || len%16!=0)
      return 1;

   for(i=0; i<16; i++)
      AES_KEY = ekey[i];

   for(i=0; 16*i<len; i++){ 
      if(i==0)
         AES_CTRL = (0 << AES_MODE) | (0 << AES_DIR);
      else
         AES_CTRL = (1 << AES_MODE) | (0 << AES_DIR);
      
      for(j=0; j<16; j++)
         AES_STATE = data[16*i+j];
      AES_CTRL |= (1 << AES_REQUEST);
      while(!(AES_STATUS & (1 << AES_DONE)))
         continue;
      for(j=0; j<16; j++)
         data[16*i+j] = AES_STATE;
   }
   return 0;
}

uint8_t aes_decrypt(uint8_t *data, uint8_t len)
{
   int8_t i;
   uint8_t j;

   if(len==1 || len%16!=0)
      return 1;

   for(i=0; i<16; i++)
      AES_KEY = dkey[i];

   for(i=(len/16)-1; i>=0; i--){ 
      AES_CTRL = (0 << AES_MODE) | (1 << AES_DIR);
      
      for(j=0; j<16; j++)
         AES_STATE = data[16*i+j];
      AES_CTRL |= (1 << AES_REQUEST);
      while(!(AES_STATUS & (1 << AES_DONE)))
         continue;
      for(j=0; j<16; j++){
         data[16*i+j] = AES_STATE;
         if(i!=0)
            data[16*i+j] ^= data[16*(i-1)+j];
      }
   }
   return 0;
}

