/*
 * dwm1000.h - library interface for dwm1000
 */

#ifndef _DWM100_H_
#define _DWM100_H_

#include <stdint.h>

// used to check if the device is connected over SPI correctly in dwm_init()
#define DWM_DEV_ID 0xDECA0130

typedef enum _dwm1000_status {
  DWM_SUCCESS = (uint32_t) 0x0U,               // generic success
  DWM_ERROR = (uint32_t) 0x1U,                 // generic error
  DWM_INVALID_CONFIG = (uint32_t) 0x2U,        // invalid configuration settings
  DWM_ERR_RXPHE = ((uint32_t) 0x1U << 12),     // Receiver PHY Header Error
  DWM_ERR_RXFCE = ((uint32_t) 0x1U << 15),     // Receiver CRC Error
  DWM_ERR_RXRFSL = ((uint32_t) 0x1U << 16),    // Receiver Reed Solomon Frame Sync Loss
  DWM_ERR_LDEERR = ((uint32_t) 0x1U << 18),    // Leading edge detection processing error
  DWM_ERR_RXPTO = ((uint32_t) 0x1U << 21),     // Preamble detection timeout
  DWM_ERR_RFPLL_LL = ((uint32_t) 0x1U << 24),  // RF PLL Losing Lock
  DWM_ERR_CLKPLL_LL = ((uint32_t) 0x1U << 25), // Clock PLL Losing Lock
  DWM_ERR_RXSFDTO = ((uint32_t) 0x1U << 26),   // Receive SFD timeout
  DWM_ERR_AFFREJ = ((uint32_t) 0x1U << 29),    // Automatic Frame Filtering rejection
} dwm1000_status;

typedef enum _dwm1000_channel {
  CHANNEL_1 = 1U,
  CHANNEL_2 = 2U,
  CHANNEL_3 = 3U,
  CHANNEL_4 = 4U,
  CHANNEL_5 = 5U,
  CHANNEL_7 = 7U,
} dwm1000_channel;

typedef enum _dwm1000_bitrate {
  RF_110KBps = 0x0U, // todo: lots of edge cases not implemented with this...
  RF_850KBps = 0x1U,
  RF_6800KBps = 0x2U,
} dwm1000_bitrate;

typedef enum _dwm1000_prf {
  PRF_16MHZ = 0x1U,
  PRF_64MHZ = 0x2U,
} dwm1000_prf;

typedef enum _dwm1000_psr {
  PSR_64 = 0x1U,
  PSR_128 = 0x5U,
  PSR_256 = 0x9U,
  PSR_512 = 0xDU,
  PSR_1024 = 0x2U,
  PSR_1536 = 0x6U,
  PSR_2048 = 0xAU,
  PSR_4096 = 0x3U,
} dwm1000_psr;

typedef enum _dwm1000_preamble_code {
  PREAMBLE_CODE_1 = 1U,
  PREAMBLE_CODE_2 = 2U,
  PREAMBLE_CODE_3 = 3U,
  PREAMBLE_CODE_4 = 4U,
  PREAMBLE_CODE_5 = 5U,
  PREAMBLE_CODE_6 = 6U,
  PREAMBLE_CODE_7 = 7U,
  PREAMBLE_CODE_8 = 8U,
  PREAMBLE_CODE_9 = 9U,
  PREAMBLE_CODE_10 = 10U,
  PREAMBLE_CODE_11 = 11U,
  PREAMBLE_CODE_12 = 12U,
  PREAMBLE_CODE_17 = 17U,
  PREAMBLE_CODE_18 = 18U,
  PREAMBLE_CODE_19 = 19U,
  PREAMBLE_CODE_20 = 20U,
} dwm1000_preamble_code;

typedef enum _dwm1000_clock {
  AUTO_CLK = 0U,
  XTI_9MHZ = 1U,
  PLL_125MHZ = 2U,
} dwm1000_clock;

typedef struct _dwm1000_config {
  dwm1000_preamble_code preamble;
  dwm1000_bitrate rate;
  dwm1000_prf prf;
  dwm1000_psr psr;
} _dwm1000_config;

// Radio lengths predefined by dw platform
#define DWM_MAX_RX_LEN  127
#define DWM_CRC_LEN     2

// todo: add function signatures with descriptions
// todo: need way to register unique delay function

void print_settings(void);
void print_status(dwm1000_status status);

uint64_t dwm_get_sys_status(void);
uint64_t dwm_get_raw_curr_timestamp(void);
uint64_t dwm_get_raw_rx_timestamp(void);
uint64_t dwm_get_raw_tx_timestamp(void);

dwm1000_status dwm_init(void);
dwm1000_status dwm_trigger_IRQ(void);
dwm1000_status dwm_enable_rx_IRQ(void);
dwm1000_status dwm_disable_rx_IRQ(void);
dwm1000_status dwm_enable_autorx(void);
dwm1000_status dwm_enable_rx_timestamp(void);
dwm1000_status dwm_disable_rx_timestamp(void);
dwm1000_status dwm_rx_mode(void);
dwm1000_status dwm_idle_mode(void);
dwm1000_status dwm_sleep_mode(void);
dwm1000_status dwm_set_channel(dwm1000_channel chan);
dwm1000_status dwm_get_rx_timestamp(double *timestamp);
dwm1000_status dwm_get_tx_timestamp(double *timestamp);
dwm1000_status dwm_rx_pkt_blocking(uint8_t *data, uint32_t len, uint32_t *rx_len);
dwm1000_status dwm_rx_pkt_nonblocking(uint8_t *buf, uint8_t buf_len, uint32_t *rx_len);
dwm1000_status dwm_tx_pkt_nonblocking(uint8_t *data, uint32_t len);
dwm1000_status dwm_tx_pkt_blocking(uint8_t *data, uint32_t len);
dwm1000_status dwm_tx_delayed_pkt_nonblocking(uint8_t *data, uint32_t len, uint64_t timestamp);
dwm1000_status dwm_configure_IRQ(void *rx_callback, void *tx_callback);
dwm1000_status dwm_rx_config(dwm1000_bitrate rate, dwm1000_preamble_code code, dwm1000_prf prf, dwm1000_psr psr);
dwm1000_status dwm_tx_config(dwm1000_bitrate rate, dwm1000_preamble_code code, dwm1000_prf prf, dwm1000_psr psr);

#endif
