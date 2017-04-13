/*
 * dwm1000_sstwr.c - Single sided two way ranging lib for dw1000
 */
#include <math.h>
#include <stdint.h>
#include "dwm1000_sstwr.h"
#include "dwm1000_platform.h"
#include "dwm1000_spi.h"
#include "dwm1000.h"
#include "dwm1000_regs.h"

#define DELAYED_ANSWER_US_OFFSET   ((uint64_t)700)
#define DELAYED_ANSWER_TIMEUNITS_OFFSET   (((uint64_t)500*(uint64_t)128)*(DELAYED_ANSWER_US_OFFSET))

#define DWM_TU_TO_US(tu) ((((uint64_t)(tu))/((uint64_t)128))/(uint64_t)500)

static ranging_info _current_ranging;
static volatile uint64_t last_prop_time;

void (*_user_rx_callback) (uint64_t);

void print_timestamp(uint64_t timestamp)
{
  uint8_t *ts = (uint8_t*) &timestamp;
  printf("%x %x %x %x %x\r\n", ts[4], ts[3], ts[2], ts[1], ts[0]);
}

void restart_everything(void)
{
    dwm_sstwr_init(_user_rx_callback);
}

/* @brief tx callback for sstwr
 */
void dwm_sstwr_tx_callback(void)
{
}

/* @brief rx callback for sstwr
 */
void dwm_sstwr_rx_callback(void)
{
  dwm1000_status status;
  ranging_packet rx_packet;
  uint32_t rx_len;
  uint64_t rx_timestamp, tx_timestamp;

  rx_packet.is_request = 0;
  rx_packet.responder_mac = 0;
  rx_packet.responder_tx_mark = 0;
  rx_packet.responder_rx_mark = 0;

  rx_timestamp = dwm_get_raw_rx_timestamp();
  tx_timestamp = rx_timestamp + DELAYED_ANSWER_TIMEUNITS_OFFSET;

  status = dwm_rx_pkt_nonblocking((uint8_t*)&rx_packet, sizeof(rx_packet), &rx_len);

  if (status != DWM_SUCCESS)
  {
    printf("error: rx error = %x\r\n", status);
    restart_everything();
  }
  else if (rx_len - DWM_CRC_LEN != sizeof(rx_packet))
  {
    printf("rx len invalid %d\r\n", rx_len);
  }
  else if (rx_packet.is_request == 1)
  {
      // this is a ranging request, just send him our timestamps
      // mac stuff will change
      rx_packet.is_request = 0;
      rx_packet.responder_mac = 1;
      rx_packet.responder_rx_mark = rx_timestamp;
      rx_packet.responder_tx_mark = tx_timestamp;

      nrk_led_toggle(RED_LED);

      dwm_tx_delayed_pkt_nonblocking((uint8_t*)&rx_packet, sizeof(rx_packet), tx_timestamp);
      dwm_host_delay_us(2);
      dwm_rx_mode();
  }
  else if (rx_packet.is_request == 0)
  {
      // this is a response to a request
    uint64_t t_prop, t_round, t_reply;
    _current_ranging.anchor_rx_mark = dwm_get_raw_rx_timestamp();
    t_round = _current_ranging.anchor_rx_mark - _current_ranging.anchor_tx_mark;
    t_reply = rx_packet.responder_tx_mark - rx_packet.responder_rx_mark;
    t_prop = t_round-t_reply;

    //printf(">> t_prop:"); print_timestamp(t_prop);

    if (_user_rx_callback)
    {
      _user_rx_callback(t_prop);
    }

    _current_ranging.done = 1;
  }
  else
  {
      printf("CORRUPT PACKAGE\r\n");
  }

}

uint64_t dwm_sstwr_get_last_ranging(void)
{
    return last_prop_time;
}

/* @brief collects responses to the ranging's sent
 */
void dwm_sstwr_blocking_collect(void)
{
  dwm1000_status status;
  ranging_packet rx_packet;
  uint32_t rx_len;
  uint64_t rx_timestamp, tx_timestamp;

  rx_packet.is_request = 0;
  rx_packet.responder_mac = 0;
  rx_packet.responder_tx_mark = 0;
  rx_packet.responder_rx_mark = 0;

  rx_timestamp = dwm_get_raw_rx_timestamp();
  tx_timestamp = rx_timestamp + DELAYED_ANSWER_TIMEUNITS_OFFSET;

  status = dwm_rx_pkt_blocking((uint8_t*)&rx_packet, sizeof(rx_packet), &rx_len);

  if (status != DWM_SUCCESS)
  {
    printf("error: rx error = %x\r\n", status);
    return;
  }

  // check that we have a packet of the correct size
  if (rx_len - DWM_CRC_LEN != sizeof(rx_packet))
  {
    printf("rx len invalid %d\r\n", rx_len);
    return;
  }


  // this is a ranging request, just send him our timestamps
  if (rx_packet.is_request == 1)
  {
      // mac stuff will change
      rx_packet.is_request = 0;
      rx_packet.responder_mac = 1;
      rx_packet.responder_rx_mark = rx_timestamp;
      rx_packet.responder_tx_mark = tx_timestamp;

      dwm_tx_delayed_pkt_nonblocking((uint8_t*)&rx_packet, sizeof(rx_packet), tx_timestamp);
      printf("RESPONDED!\r\n");
  }


  // this is a response to a request
  else if (rx_packet.is_request == 0)
  {
    uint64_t prop_time;

    _current_ranging.anchor_rx_mark = dwm_get_raw_rx_timestamp();

    prop_time = _current_ranging.anchor_rx_mark - _current_ranging.anchor_tx_mark;
    prop_time -= rx_packet.responder_tx_mark - rx_packet.responder_rx_mark;
    prop_time /= 2;

    last_prop_time = prop_time;

    if (_user_rx_callback)
    {
      _user_rx_callback(prop_time);
    }
    _current_ranging.done = 1;
    //printf(">> prop_time: ");print_timestamp(prop_time);
  }

  else
  {
      printf("CORRUPT PACKAGE\r\n");
  }
}

/* @brief request ranging from designated node
 */
void dwm_sstwr_request_ranging(uint16_t their_mac)
{
  uint16_t my_mac = 1;
  ranging_packet pkt;
  uint64_t tx_timestamp;
/*
  if (!_current_ranging.done)
  {
    if(_current_ranging.wait_count++ < MAX_WAIT_COUNT)
    {
      printf("wait_count %d\r\n", _current_ranging.wait_count);
      return;
    }
  }
  _current_ranging.done = 0;
  _current_ranging.wait_count = 0;
*/
  pkt.is_request = 1;
  pkt.anchor_mac = my_mac;
  pkt.responder_mac = their_mac;
  pkt.responder_rx_mark = 0;
  pkt.responder_tx_mark = 0;

  tx_timestamp = dwm_get_raw_curr_timestamp() + DELAYED_ANSWER_TIMEUNITS_OFFSET;
  _current_ranging.anchor_tx_mark = tx_timestamp;

  if (dwm_tx_pkt_blocking((uint8_t*)&pkt, sizeof(pkt)) != DWM_SUCCESS)
  {
      printf("couldn't transmit!!\r\n");
  }
  dwm_rx_mode();

}

/* @brief init dwm_sstwr_init
 */
void dwm_sstwr_init(void (*user_rx_callback)(uint64_t) )
{
  if (user_rx_callback)
  {
      _user_rx_callback = user_rx_callback;
  }

  dwm_spi_init();
  dwm_init();
  dwm_configure_IRQ(dwm_sstwr_rx_callback, NULL);
  dwm_enable_autorx();
  dwm_enable_rx_timestamp();
  dwm_GPIO_enable_leds();
  printf("dwm_sstwr_init done!\r\n");

  // now make sure spi is fast again
  dwm_fast_spi();
}
