/*
 * dwm1000.c - High level dw1000 driver
 */
#include <math.h>
#include <stdint.h>

#include "dwm1000.h"
#include "dwm1000_spi.h"
#include "dwm1000_regs.h"
#include "dwm1000_platform.h"

// DWM1000 timer resolution in microseconds
#define TIME_RES_US   ((double)0.0000156500400641025)
// DWM timestamp overflow (40 bit timestamps)
#define TIME_OVERFLOW ((double)1099511627776)

// RX and tX IRQ callbacks
#define NULL ((void *)0)
static void (*_tx_callback)(void) = NULL;
static void (*_rx_callback)(void) = NULL;

/*
 * array of default configurations indexed by a _dwm1000_channel.
 */
_dwm1000_config dwm_default_config[8] = {
  {0, 0, 0, 0}, // channel 0 is invalid
  { // channel 1
    .preamble = PREAMBLE_CODE_10,
    .rate = RF_6800KBps,
    .prf = PRF_64MHZ,
    .psr = PSR_128,
  },
  { // channel 2
    .preamble = PREAMBLE_CODE_10,
    .rate = RF_6800KBps,
    .prf = PRF_64MHZ,
    .psr = PSR_128,
  },
  { // channel 3
    .preamble = PREAMBLE_CODE_6,
    .rate = RF_850KBps,
    .prf = PRF_16MHZ,
    .psr = PSR_1024,
  },
  { // channel 4
    .preamble = PREAMBLE_CODE_18,
    .rate = RF_6800KBps,
    .prf = PRF_64MHZ,
    .psr = PSR_128,
  },
  { // channel 5
    .preamble = PREAMBLE_CODE_4,
    .rate = RF_6800KBps,
    .prf = PRF_16MHZ,
    .psr = PSR_128,
  },
  {0, 0, 0, 0}, // channel 6 is invalid
  { // channel 7
    .preamble = PREAMBLE_CODE_18,
    .rate = RF_6800KBps,
    .prf = PRF_64MHZ,
    .psr = PSR_128,
  },
};

/**
 * @brief take an array of bytes and make a 16 bit unsigned number.
 *
 * @param x the array of bytes
 * @return an unsigned 16 bit number
 */
static inline uint16_t bytes_to_16bit(uint8_t *x) {
  return ((uint16_t)x[1] << 8) | ((uint16_t)x[0]);
}


/**
 * @brief take an array of bytes and make a 32 bit unsigned number.
 *
 * @param x the array of bytes
 * @return an unsigned 32 bit number
 */
static inline uint32_t bytes_to_32bit(uint8_t *x) {
  return ((uint32_t)x[3] << 24) | ((uint32_t)x[2] << 16) | ((uint32_t)x[1] << 8) | ((uint32_t)x[0]);
}


/**
 * @brief take an array of bytes and make a 40 bit unsigned number.
 *
 * @param x the array of bytes
 * @return an unsigned 64 bit number
 */
static inline uint64_t bytes_to_40bit(uint8_t *x) {
  return ((uint64_t)x[4] << 32) | ((uint64_t)x[3] << 24) | ((uint64_t)x[2] << 16) |
         ((uint64_t)x[1] << 8) | ((uint64_t)x[0]);
}


/**
 * @brief take an array of bytes and make a 64 bit unsigned number.
 *
 * @param x the array of bytes
 * @return an unsigned 64 bit number
 */
static inline uint64_t bytes_to_64bit(uint8_t *x) {
  return ((uint64_t)x[7] << 56) | ((uint64_t)x[6] << 48) | ((uint64_t)x[5] << 40) |
         ((uint64_t)x[4] << 32) | ((uint64_t)x[3] << 24) | ((uint64_t)x[2] << 16) |
         ((uint64_t)x[1] << 8) | ((uint64_t)x[0]);
}


uint32_t dwm_get_dev_id(void) {
  uint8_t devID[LEN_DEV_ID];
  dwm_spi_read(DEV_ID, 0, LEN_DEV_ID, devID);
  return bytes_to_32bit(devID);
}


uint64_t dwm_get_ip_address(void) {
  uint8_t ipAddr[LEN_EUI];
  dwm_spi_read(EUI, 0, LEN_EUI, ipAddr);
  return bytes_to_64bit(ipAddr);
}


dwm1000_status dwm_set_system_clock(dwm1000_clock clk) {
  uint32_t reg32;
  uint8_t data[LEN_PMSC_CTRL0];

  // DEBUG_PRINT("setting system clock...\n\r");

  // set SYSCLKS in PMSC_CTRL0
  dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, data);
  reg32 = bytes_to_32bit(data);
  reg32 &= ~((uint32_t)0x3);
  reg32 |= clk;
  dwm_spi_write(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, (uint8_t *)(&reg32));
  // dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, data);
  // reg32 = bytes_to_32bit(data);
  // DEBUG_PRINTF("PMSC_CTRL0: 0x%08x\n\r", reg32);

  return DWM_SUCCESS;
}

dwm1000_status dwm_rx_mode(void) {
  uint32_t reg32;
  uint8_t sysCtrl[LEN_SYS_CTRL];

  // set RXENAB in SYS_CTRL
  reg32 = (0x1 << 8);
  dwm_spi_write(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, (uint8_t *)(&reg32));

  // wait until it is cleared to know RX mode reached
  while (1) {
    dwm_spi_read(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, sysCtrl);
    if ((sysCtrl[0] & (0x1 << 8)) == 0x0) {
      break;
    }
  }

  return DWM_SUCCESS;
}

dwm1000_status dwm_idle_mode(void) {
  uint32_t reg32;
  uint8_t sysCtrl[LEN_SYS_CTRL];

  // set TRXOFF in SYS_CTRL
  reg32 = (0x1 << 6);
  dwm_spi_write(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, (uint8_t *)(&reg32));

  // wait until it is cleared to know IDLE mode reached
  while (1) {
    dwm_spi_read(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, sysCtrl);
    if ((sysCtrl[0] & (0x1 << 6)) == 0x0) {
      break;
    }
  }

  return DWM_SUCCESS;
}

dwm1000_status dwm_sleep_mode(void){
    uint32_t reg32;

    dwm_spi_read(AON, AON_CFG1_OFFSET, LEN_AON_CFG1, &reg32);
    reg32 |= (1<<1);
    dwm_spi_write(AON, AON_CFG1_OFFSET, LEN_AON_CFG1, &reg32);

    dwm_spi_read(AON, AON_CFG0_OFFSET, LEN_AON_CFG0, &reg32);
    // set the sleep_en bit
    reg32 |= 1<<0;
    dwm_spi_write(AON, AON_CFG0_OFFSET, LEN_AON_CFG0, &reg32);

    dwm_spi_read(AON, AON_CTRL_OFFSET, LEN_AON_CTRL, &reg32);
    // set upl_cfg
    reg32 |= 1<<2;
    dwm_spi_write(AON, AON_CTRL_OFFSET, LEN_AON_CTRL, &reg32);

    return DWM_SUCCESS;
}


dwm1000_status dwm_reset_rx(void) {
  uint32_t reg32;
  uint8_t data[LEN_PMSC_CTRL0];

  // DEBUG_PRINT("reset RX...\n\r");

  // set SYSCLKS to 0x01 for 19.2 MHz XTI clock
  dwm_set_system_clock(XTI_9MHZ);

  // set SOFTRESET bit to reset RX interface
  dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, data);
  reg32 = bytes_to_32bit(data);
  reg32 &= ~((uint32_t)0x1 << 28);
  dwm_spi_write(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, (uint8_t *)(&reg32));
  // dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, data);
  // reg32 = bytes_to_32bit(data);
  // DEBUG_PRINTF("PMSC_CTRL0: 0x%08x\n\r", reg32);

  // set SOFTRESET back to all 1's
  dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, sizeof(uint32_t), data);
  reg32 = bytes_to_32bit(data);
  reg32 |= ((uint32_t)0xF << 28);
  dwm_spi_write(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, (uint8_t *)(&reg32));
  // dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, data);
  // reg32 = bytes_to_32bit(data);
  // DEBUG_PRINTF("PMSC_CTRL0: 0x%08x\n\r", reg32);

  // resume AUTO clocking
  dwm_set_system_clock(AUTO_CLK);

  // force processor into idle mode
  return dwm_idle_mode();
}


dwm1000_status dwm_soft_reset(void) {
  uint32_t reg32;
  uint8_t data[LEN_PMSC_CTRL0];

  // DEBUG_PRINT("soft reset...\n\r");

  // set SYSCLKS to 0x01 for 19.2 MHz XTI clock
  dwm_set_system_clock(XTI_9MHZ);

  // set SOFTRESET to all 0's (reset IC, TX, RX, and host interface)
  dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, data);
  reg32 = bytes_to_32bit(data);
  reg32 &= ~((uint32_t)0xF << 28);
  dwm_spi_write(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, (uint8_t *)(&reg32));
  // dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, data);
  // reg32 = bytes_to_32bit(data);
  // DEBUG_PRINTF("PMSC_CTRL0: 0x%08x\n\r", reg32);

  // set SOFTRESET back to all 1's
  dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, sizeof(uint32_t), data);
  reg32 = bytes_to_32bit(data);
  reg32 |= ((uint32_t)0xF << 28);
  dwm_spi_write(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, (uint8_t *)(&reg32));
  // dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, data);
  // reg32 = bytes_to_32bit(data);
  // DEBUG_PRINTF("PMSC_CTRL0: 0x%08x\n\r", reg32);

  // resume AUTO clocking
  dwm_set_system_clock(AUTO_CLK);

  // force processor into idle mode
  return dwm_idle_mode();
}


dwm1000_status dwm_get_channel(dwm1000_channel *chan) {
  uint8_t data[LEN_CHAN_CTRL];
  // get TX_CHAN in CHAN_CTRL register
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, data);
  *chan = (dwm1000_channel)(data[0] & 0xF);
  return DWM_SUCCESS;
}


dwm1000_status dwm_set_channel(dwm1000_channel chan) {
  uint32_t reg32;
  uint8_t chanCtrl[LEN_CHAN_CTRL];
  uint8_t fsPllcfg[LEN_FS_PLLCFG];
  uint8_t rfTxctrl[LEN_RF_TXCTRL];
  uint8_t rfRxctrl[LEN_RF_RXCTRLH];
  uint8_t fsPlltune[LEN_FS_PLLTUNE];
  uint8_t tcPgdelay[LEN_TC_PGDELAY];

  DEBUG_PRINT("setting channel...\r\n");

  // set RX_CHAN and TX_CHAN in CHAN_CTRL register
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, chanCtrl);
  reg32 = bytes_to_32bit(chanCtrl);
  reg32 &= ~((uint32_t)0xFF);
  reg32 |= (((uint32_t)chan << 0x4) | chan);
  dwm_spi_write(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, (uint8_t *)(&reg32));
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, chanCtrl);
  reg32 = bytes_to_32bit(chanCtrl);
  DEBUG_PRINTF("CHAN_CTRL: 0x%08x\n\r", reg32);

  // set FS_PLLCFG in FS_CTRL to correct value for channel
  switch (chan) {
    case CHANNEL_1:
      reg32 = 0x09000407U;
      break;
    case CHANNEL_2:
    case CHANNEL_4:
      reg32 = 0x08400508U;
      break;
    case CHANNEL_3:
      reg32 = 0x08401009U;
      break;
    case CHANNEL_5:
    case CHANNEL_7:
      reg32 = 0x0800041DU;
      break;
    default:
      return DWM_INVALID_CONFIG;
  }
  dwm_spi_write(FS_CTRL, FS_PLLCFG_OFFSET, LEN_FS_PLLCFG, (uint8_t *)(&reg32));
  dwm_spi_read(FS_CTRL, FS_PLLCFG_OFFSET, LEN_FS_PLLCFG, fsPllcfg);
  reg32 = bytes_to_32bit(fsPllcfg);
  DEBUG_PRINTF("FS_PLLCFG: 0x%08x\n\r", reg32);

  // set FS_PLLTUNE in FS_CTRL to correct value for channel
  switch (chan) {
    case CHANNEL_1:
      fsPlltune[0] = 0x1EU;
      break;
    case CHANNEL_2:
    case CHANNEL_4:
      fsPlltune[0] = 0x26U;
      break;
    case CHANNEL_3:
      fsPlltune[0] = 0x5EU;
      break;
    case CHANNEL_5:
    case CHANNEL_7:
      fsPlltune[0] = 0xBEU;
      break;
  }
  dwm_spi_write(FS_CTRL, FS_PLLTUNE_OFFSET, LEN_FS_PLLTUNE, fsPlltune);
  dwm_spi_read(FS_CTRL, FS_PLLTUNE_OFFSET, LEN_FS_PLLTUNE, fsPlltune);
  DEBUG_PRINTF("FS_PLLTUNE: 0x%02x\n\r", fsPlltune[0]);

  // set RF_RXCTRLH in RF_CONF to correct value for channel
  switch (chan) {
    case CHANNEL_1:
    case CHANNEL_2:
    case CHANNEL_3:
    case CHANNEL_5:
      rfRxctrl[0] = 0xD8U;
      break;
    case CHANNEL_4:
    case CHANNEL_7:
      rfRxctrl[0] = 0xBCU;
      break;
  }
  dwm_spi_write(RF_CONF, RF_RXCTRLH_OFFSET, LEN_RF_RXCTRLH, rfRxctrl);
  dwm_spi_read(RF_CONF, RF_RXCTRLH_OFFSET, LEN_RF_RXCTRLH, rfRxctrl);
  DEBUG_PRINTF("RF_RXCTRLH: 0x%02x\n\r", rfRxctrl[0]);

  // set RF_TXCTRL in RF_CONF to correct value for channel
  switch (chan) {
    case CHANNEL_1:
      reg32 = 0x00005C40U;
      break;
    case CHANNEL_2:
      reg32 = 0x00045CA0U;
      break;
    case CHANNEL_3:
      reg32 = 0x00086CC0U;
      break;
    case CHANNEL_4:
      reg32 = 0x00045C80U;
      break;
    case CHANNEL_5:
      reg32 = 0x001E3FE0U;
      break;
    case CHANNEL_7:
      reg32 = 0x001E7DE0U;
      break;
  }
  dwm_spi_write(RF_CONF, RF_TXCTRL_OFFSET, LEN_RF_TXCTRL, (uint8_t *)(&reg32));
  dwm_spi_read(RF_CONF, RF_TXCTRL_OFFSET, LEN_RF_TXCTRL, rfTxctrl);
  reg32 = bytes_to_32bit(rfTxctrl);
  DEBUG_PRINTF("RF_TXCTRL: 0x%08x\n\r", reg32);

  // set TC_PGDELAY in TX_CAL
  switch (chan) {
    case CHANNEL_1:
      tcPgdelay[0] = 0xC9U;
      break;
    case CHANNEL_2:
      tcPgdelay[0] = 0xC2U;
      break;
    case CHANNEL_3:
      tcPgdelay[0] = 0xC5U;
      break;
    case CHANNEL_4:
      tcPgdelay[0] = 0x95U;
      break;
    case CHANNEL_5:
      tcPgdelay[0] = 0xC0U;
      break;
    case CHANNEL_7:
      tcPgdelay[0] = 0x93U;
      break;
  }
  dwm_spi_write(TX_CAL, TC_PGDELAY_OFFSET, LEN_TC_PGDELAY, tcPgdelay);
  dwm_spi_read(TX_CAL, TC_PGDELAY_OFFSET, LEN_TC_PGDELAY, tcPgdelay);
  DEBUG_PRINTF("TC_PGDELAY: 0x%02x\n\r", tcPgdelay[0]);

  return DWM_SUCCESS;
}


dwm1000_status dwm_manual_tx_power(dwm1000_prf txprf) {
  uint32_t reg32;
  dwm1000_channel chan;
  uint8_t sysCfg[LEN_SYS_CFG];
  uint8_t txPower[LEN_TX_POWER];

  DEBUG_PRINT("setting tx manual power...\r\n");

  // set DIS_STXP to disable smart TX power
  dwm_spi_read(SYS_CFG, SYS_CFG_OFFSET, LEN_SYS_CFG, sysCfg);
  reg32 = bytes_to_32bit(sysCfg);
  reg32 |= ((uint32_t)0x1 << 18);
  dwm_spi_write(SYS_CFG, SYS_CFG_OFFSET, LEN_SYS_CFG, (uint8_t *)(&reg32));
  dwm_spi_read(SYS_CFG, SYS_CFG_OFFSET, LEN_SYS_CFG, sysCfg);
  reg32 = bytes_to_32bit(sysCfg);
  DEBUG_PRINTF("SYS_CFG: 0x%08x\n\r", reg32);

  // get channel settings
  dwm_get_channel(&chan);

  // set TX_POWER according to prf
  dwm_spi_read(TX_POWER, TX_POWER_OFFSET, LEN_TX_POWER, txPower);
  reg32 = bytes_to_32bit(txPower);
  switch (chan) {
    case CHANNEL_1:
    case CHANNEL_2:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x75757575U;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x67676767U;
      }
      break;
    case CHANNEL_3:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x6F6F6F6FU;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x8B8B8B8BU;
      }
      break;
    case CHANNEL_4:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x5F5F5F5FU;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x9A9A9A9AU;
      }
      break;
    case CHANNEL_5:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x48484848U;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x85858585U;
      }
      break;
    case CHANNEL_7:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x92929292U;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0xD1D1D1D1U;
      }
      break;
    default:
      return DWM_INVALID_CONFIG;
  }
  dwm_spi_write(TX_POWER, TX_POWER_OFFSET, LEN_TX_POWER, (uint8_t *)(&reg32));
  dwm_spi_read(TX_POWER, TX_POWER_OFFSET, LEN_TX_POWER, txPower);
  reg32 = bytes_to_32bit(txPower);
  DEBUG_PRINTF("TX_POWER: 0x%08x\n\r", reg32);

  return DWM_SUCCESS;
}


dwm1000_status dwm_smart_tx_power(dwm1000_prf txprf) {
  uint32_t reg32;
  dwm1000_channel chan;
  uint8_t sysCfg[LEN_SYS_CFG];
  uint8_t txPower[LEN_TX_POWER];

  DEBUG_PRINT("setting tx smart power...\r\n");

  // clear DIS_STXP to enable smart TX power
  dwm_spi_read(SYS_CFG, SYS_CFG_OFFSET, LEN_SYS_CFG, sysCfg);
  reg32 = bytes_to_32bit(sysCfg);
  reg32 &= ~((uint32_t)0x1 << 18);
  dwm_spi_write(SYS_CFG, SYS_CFG_OFFSET, LEN_SYS_CFG, (uint8_t *)(&reg32));
  dwm_spi_read(SYS_CFG, SYS_CFG_OFFSET, LEN_SYS_CFG, sysCfg);
  reg32 = bytes_to_32bit(sysCfg);
  DEBUG_PRINTF("SYS_CFG: 0x%08x\n\r", reg32);

  // get channel settings
  dwm_get_channel(&chan);

  // set TX_POWER according to prf and channel
  switch (chan) {
    case CHANNEL_1:
    case CHANNEL_2:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x15355575U;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x07274767U;
      }
      break;
    case CHANNEL_3:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x0F2F4F6FU;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x2B4B6B8BU;
      }
      break;
    case CHANNEL_4:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x1F1F3F5FU;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x3A5A7A9AU;
      }
      break;
    case CHANNEL_5:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x0E082848U;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x25456585U;
      }
      break;
    case CHANNEL_7:
      if (txprf == PRF_16MHZ) {
        reg32 = 0x32527292U;
      } else if (txprf == PRF_64MHZ) {
        reg32 = 0x5171B1D1U;
      }
      break;
    default:
      return DWM_INVALID_CONFIG;
  }
  dwm_spi_write(TX_POWER, TX_POWER_OFFSET, LEN_TX_POWER, (uint8_t *)(&reg32));
  dwm_spi_read(TX_POWER, TX_POWER_OFFSET, LEN_TX_POWER, txPower);
  reg32 = bytes_to_32bit(txPower);
  DEBUG_PRINTF("TX_POWER: 0x%08x\n\r", reg32);

  return DWM_SUCCESS;
}


dwm1000_status dwm_tx_config(dwm1000_bitrate rate, dwm1000_preamble_code code, dwm1000_prf prf, dwm1000_psr psr) {
  uint32_t reg32;
  uint64_t reg64;
  uint8_t txFctrl[LEN_TX_FCTRL];
  uint8_t chanCtrl[LEN_CHAN_CTRL];

  DEBUG_PRINT("setting TX config...\r\n");

  // set TXBR, TXPRF, and {PE, TXPSR} in TX_FCTRL register
  dwm_spi_read(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, txFctrl);
  reg64 = bytes_to_40bit(txFctrl);
  reg64 &= ~((uint32_t)0x3 << 13);
  reg64 |= ((uint32_t)rate << 13);
  reg64 &= ~((uint32_t)0x3 << 16);
  reg64 |= ((uint32_t)prf << 16);
  reg64 &= ~((uint32_t)0xF << 18);
  reg64 |= ((uint32_t)psr << 18);
  dwm_spi_write(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, (uint8_t *)(&reg64));
  dwm_spi_read(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, txFctrl);
  reg64 = bytes_to_40bit(txFctrl);
  DEBUG_PRINTF("TX_FCTRL: 0x%010llx\n\r", reg64);

  // set TX_PCODE in CHAN_CTRL register
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, chanCtrl);
  reg32 = bytes_to_32bit(chanCtrl);
  reg32 &= ~((uint32_t)0x1F << 22);
  reg32 |= ((uint32_t)code << 22);
  dwm_spi_write(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, (uint8_t *)(&reg32));
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, chanCtrl);
  reg32 = bytes_to_32bit(chanCtrl);
  DEBUG_PRINTF("CHAN_CTRL: 0x%08x\n\r", reg32);

  // Smart TX power control applies at the 6.8 Mbps data rate
  if (rate == RF_6800KBps) {
    return dwm_smart_tx_power(prf);
  }

  return dwm_manual_tx_power(prf);
}


dwm1000_status dwm_get_tx_preamble_code(dwm1000_preamble_code *code) {
  uint32_t reg32;
  uint8_t data[LEN_CHAN_CTRL];
  // get TX_PCODE in CHAN_CTRL register
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, data);
  reg32 = bytes_to_32bit(data);
  *code = (dwm1000_preamble_code)((reg32 & ((uint32_t)0x1FU << 22)) >> 22);
  return DWM_SUCCESS;
}


dwm1000_status dwm_get_tx_bit_rate(dwm1000_bitrate *rate) {
  uint8_t data[LEN_TX_FCTRL];
  // get TXBR in TX_FCTRL register
  dwm_spi_read(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, data);
  *rate = (dwm1000_bitrate)((data[1] >> 5) & 0x3);
  return DWM_SUCCESS;
}


dwm1000_status dwm_get_tx_prf(dwm1000_prf *prf) {
  uint8_t data[LEN_TX_FCTRL];
  // get TXPRF in TX_FCTRL register
  dwm_spi_read(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, data);
  *prf = (dwm1000_prf)(data[2] & 0x03);
  return DWM_SUCCESS;
}


dwm1000_status dwm_get_tx_psr(dwm1000_psr *psr) {
  uint8_t data[LEN_TX_FCTRL];
  // get {PE, TXPSR} in TX_FCTRL register
  dwm_spi_read(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, data);
  *psr = (dwm1000_psr)((data[2] >> 2) & 0xF);
  return DWM_SUCCESS;
}


dwm1000_status dwm_rx_config(dwm1000_bitrate rate, dwm1000_preamble_code code, dwm1000_prf prf, dwm1000_psr psr) {
  uint16_t reg16;
  uint32_t reg32, DWSFD;
  uint8_t chanCtrl[LEN_CHAN_CTRL];
  uint8_t agcTune1[LEN_AGC_TUNE1];
  uint8_t drxTune2[LEN_DRX_TUNE2];
  uint8_t drxTune0b[LEN_DRX_TUNE0b];
  uint8_t drxTune1a[LEN_DRX_TUNE1a];
  uint8_t drxTune1b[LEN_DRX_TUNE1b];
  uint8_t drxTune4h[LEN_DRX_TUNE4h];

  DEBUG_PRINT("setting RX config...\r\n");

  // get DWSFD bit in CHAN_CTRL register to check for standard or non-standard SFD
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, chanCtrl);
  reg32 = bytes_to_32bit(chanCtrl);
  DWSFD = reg32 & (0x1UL << 17); // 0 => standard SFD. 1 => non-standard SFD

  // set RX_PCODE in CHAN_CTRL register
  reg32 &= ~((uint32_t)0x1F << 27);
  reg32 |= ((uint32_t)code << 27);

  // set RXPRF in CHAN_CTRL register
  reg32 &= ~((uint32_t)0x3 << 18);
  reg32 |= ((uint32_t)prf << 18);
  dwm_spi_write(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, (uint8_t *)(&reg32));
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, chanCtrl);
  reg32 = bytes_to_32bit(chanCtrl);
  DEBUG_PRINTF("CHAN_CTRL: 0x%08x\n\r", reg32);

  // set DRX_TUNE0b in DRX_CONF register
  switch (rate) {
    case RF_110KBps:
      if (DWSFD) {
        reg16 = 0x0016U;
      } else {
        reg16 = 0x000AU;
      }
      break;
    case RF_850KBps:
      if (DWSFD) {
        reg16 = 0x0006U;
      } else {
        reg16 = 0x0001U;
      }
      break;
    case RF_6800KBps:
      if (DWSFD) {
        reg16 = 0x0002U;
      } else {
        reg16 = 0x0001U;
      }
      break;
  }
  dwm_spi_write(DRX_CONF, DRX_TUNE0b_OFFSET, LEN_DRX_TUNE0b, (uint8_t *)(&reg16));
  dwm_spi_read(DRX_CONF, DRX_TUNE0b_OFFSET, LEN_DRX_TUNE0b, drxTune0b);
  reg16 = bytes_to_16bit(drxTune0b);
  DEBUG_PRINTF("DRX_TUNE0b: 0x%04x\n\r", reg16);

  // set AGC_TUNE1 in AGC_CTRL
  if (prf == PRF_16MHZ) {
    reg16 = 0x8870U;
  } else if (prf == PRF_64MHZ) {
    reg16 = 0x889BU;
  } else {
    return DWM_INVALID_CONFIG;
  }
  dwm_spi_write(AGC_CTRL, AGC_TUNE1_OFFSET, LEN_AGC_TUNE1, (uint8_t *)(&reg16));
  dwm_spi_read(AGC_CTRL, AGC_TUNE1_OFFSET, LEN_AGC_TUNE1, agcTune1);
  reg16 = bytes_to_16bit(agcTune1);
  DEBUG_PRINTF("AGC_TUNE1: 0x%04x\n\r", reg16);

  // set DRX_TUNE1a in DRX_CONF
  if (prf == PRF_16MHZ) {
    reg16 = 0x0087U;
  } else if (prf == PRF_64MHZ) {
    reg16 = 0x008DU;
  } else {
    return DWM_INVALID_CONFIG;
  }
  dwm_spi_write(DRX_CONF, DRX_TUNE1a_OFFSET, LEN_DRX_TUNE1a, (uint8_t *)(&reg16));
  dwm_spi_read(DRX_CONF, DRX_TUNE1a_OFFSET, LEN_DRX_TUNE1a, drxTune1a);
  reg16 = bytes_to_16bit(drxTune1a);
  DEBUG_PRINTF("DRX_TUNE1a: 0x%04x\n\r", reg16);

  // set DRX_TUNE1b in DRX_CONF
  switch (rate) {
    case RF_110KBps:
      switch (psr) {
        case PSR_1536:
        case PSR_2048:
        case PSR_4096:
          reg16 = 0x0064U;
          break;
        default:
          return DWM_INVALID_CONFIG;
      }
      break;
    case RF_850KBps:
      switch (psr) {
        case PSR_128:
        case PSR_256:
        case PSR_512:
        case PSR_1024:
          reg16 = 0x0020U;
          break;
        default:
          return DWM_INVALID_CONFIG;
      }
      break;
    case RF_6800KBps:
      switch (psr) {
        case PSR_64:
          reg16 = 0x0010U;
          break;
        case PSR_128:
        case PSR_256:
        case PSR_512:
        case PSR_1024:
          reg16 = 0x0020U;
          break;
        default:
          return DWM_INVALID_CONFIG;
      }
      break;
  }
  dwm_spi_write(DRX_CONF, DRX_TUNE1b_OFFSET, LEN_DRX_TUNE1b, (uint8_t *)(&reg16));
  dwm_spi_read(DRX_CONF, DRX_TUNE1b_OFFSET, LEN_DRX_TUNE1b, drxTune1b);
  reg16 = bytes_to_16bit(drxTune1b);
  DEBUG_PRINTF("DRX_TUNE1b: 0x%04x\n\r", reg16);

  // set DRX_TUNE2 in DRX_CONF
  switch (psr) {
    case PSR_64:  // PAC 8
    case PSR_128: // PAC 8
      if (prf == PRF_16MHZ) {
        reg32 = 0x311A002DU;
      } else { // PRF_64MHZ
        reg32 = 0x313B006BU;
      }
      break;
    case PSR_256: // PAC 16
    case PSR_512: // PAC 16
      if (prf == PRF_16MHZ) {
        reg32 = 0x331A0052U;
      } else { // PRF_64MHZ
        reg32 = 0x333B00BEU;
      }
      break;
    case PSR_1024: // PAC 32
      if (prf == PRF_16MHZ) {
        reg32 = 0x351A009AU;
      } else { // PRF_64MHZ
        reg32 = 0x353B015EU;
      }
      break;
    case PSR_1536: // PAC 64
    case PSR_2048: // PAC 64
    case PSR_4096: // PAC 64
      if (prf == PRF_16MHZ) {
        reg32 = 0x371A011DU;
      } else { // PRF_64MHZ
        reg32 = 0x373B0296U;
      }
      break;
  }
  dwm_spi_write(DRX_CONF, DRX_TUNE2_OFFSET, LEN_DRX_TUNE2, (uint8_t *)(&reg32));
  dwm_spi_read(DRX_CONF, DRX_TUNE2_OFFSET, LEN_DRX_TUNE2, drxTune2);
  reg32 = bytes_to_32bit(drxTune2);
  DEBUG_PRINTF("DRX_TUNE2: 0x%08x\n\r", reg32);

  // set DRX_TUNE4h in DRX_CONF
  if (psr == PSR_64) {
    reg16 = 0x0010U;
  } else {
    reg16 = 0x0028U;
  }
  dwm_spi_write(DRX_CONF, DRX_TUNE4h_OFFSET, LEN_DRX_TUNE4h, (uint8_t *)(&reg16));
  dwm_spi_read(DRX_CONF, DRX_TUNE4h_OFFSET, LEN_DRX_TUNE4h, drxTune4h);
  reg16 = bytes_to_16bit(drxTune4h);
  DEBUG_PRINTF("DRX_TUNE4h: 0x%04x\n\r", reg16);

  return DWM_SUCCESS;
}


dwm1000_status dwm_get_rx_pac_size(uint8_t *size) {
  uint32_t reg32;
  uint8_t drxTune2[LEN_DRX_TUNE2];

  // get DRX_TUNE1b in DRX_CONF
  dwm_spi_read(DRX_CONF, DRX_TUNE2_OFFSET, LEN_DRX_TUNE2, drxTune2);
  reg32 = bytes_to_32bit(drxTune2);
  switch (reg32) {
    case 0x311A002DU:
    case 0x313B006BU:
      *size = 8;
      break;
    case 0x331A0052U:
    case 0x333B00BEU:
      *size = 16;
      break;
    case 0x351A009AU:
      *size = 32;
      break;
    case 0x353B015EU:
    case 0x371A011DU:
    case 0x373B0296U:
      *size = 64;
      break;
    default:
      return DWM_INVALID_CONFIG;
  }
  return DWM_SUCCESS;
}


dwm1000_status dwm_get_rx_prf(dwm1000_prf *prf) {
  uint8_t data[LEN_CHAN_CTRL];
  // get RXPRF in CHAN_CTRL register
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, data);
  *prf = (dwm1000_prf)((data[2] & 0x0C) >> 2);
  return DWM_SUCCESS;
}


dwm1000_status dwm_get_rx_preamble_code(dwm1000_preamble_code *code) {
  uint8_t data[LEN_CHAN_CTRL];
  // get RX_PCODE in CHAN_CTRL register
  dwm_spi_read(CHAN_CTRL, CHAN_CTRL_OFFSET, LEN_CHAN_CTRL, data);
  *code = (dwm1000_preamble_code)((data[3] & 0xF8) >> 3);
  return DWM_SUCCESS;
}


dwm1000_status dwm_get_rx_bit_rate(dwm1000_bitrate *rate) {
  uint16_t reg16;
  uint8_t data[LEN_DRX_TUNE0b];
  // get DRX_TUNE0b in DRX_CONF register
  dwm_spi_read(DRX_CONF, DRX_TUNE0b_OFFSET, LEN_DRX_TUNE0b, data);
  reg16 = bytes_to_16bit(data);
  switch (reg16) {
    case 0x000AU:
    case 0x0016U:
      *rate = RF_110KBps;
      break;
    case 0x0006U:
      *rate = RF_850KBps;
      break;
    case 0x0001U: // TODO: could also be 850KBps...
    case 0x0002U:
      *rate = RF_6800KBps;
      break;
    default:
      return DWM_INVALID_CONFIG;
  }
  return DWM_SUCCESS;
}

dwm1000_status dwm_tx_delayed_pkt_nonblocking(uint8_t *data, uint32_t len, uint64_t timestamp) {
  uint64_t reg64;
  uint32_t reg32, maxLen;
  uint8_t sysCtrl[LEN_SYS_CTRL];
  uint8_t txFctrl[LEN_TX_FCTRL];
  uint8_t sysStatus[LEN_SYS_STATUS];

  // check PHR_MODE in SYS_CFG for max frame length
  dwm_spi_read(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, sysCtrl);
  reg32 = (bytes_to_32bit(sysCtrl) & (0x3UL << 16)) >> 16;
  if (reg32 == 0x0) {
    maxLen = 0x7F; // 127 bytes max if PHR_MODE = 00
  } else if (reg32 == 0x3) {
    maxLen = 0x3FF; // 1023 bytes max if PHR_MODE = 11 (long frames mode)
  } else {
    return DWM_ERROR;
  }

  // add CRC length (2 bytes) and check length
  if ((len + 2) > maxLen) {
    return DWM_ERROR;
  }

  // DWM1000 must be in IDLE mode before any RX/TX
  dwm_idle_mode();

  // write data into the TX_BUFFER at offset 0x0
  dwm_spi_write(TX_BUFFER, 0x0, len, data);

  // set TFLEN = (len + 2) and TXBOFFS = 0 in TX_FCTRL register
  dwm_spi_read(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, txFctrl);
  reg64 = bytes_to_40bit(txFctrl);
  reg64 &= ~((uint64_t)0x7F << 0);
  reg64 |= (len + 2); // TFLEN must include CRC length in bytes
  reg64 &= ~((uint64_t)0x3FF << 22);
  dwm_spi_write(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, (uint8_t *)(&reg64));

  dwm_spi_write(DX_TIME, 0, LEN_DX_TIME, (uint8_t*) &timestamp);

  // set TXSTRT and TXDLYS (do not set SFCST to allow for CRC)
  reg32 = (0x1 << 1) | (0x1 << 2);
  dwm_spi_write(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, (uint8_t *)(&reg32));

  return DWM_SUCCESS;
}

dwm1000_status dwm_tx_delayed_pkt_blocking(uint8_t *data, uint32_t len, uint64_t timestamp) {
    uint8_t sysStatus[LEN_SYS_STATUS];
    if (dwm_tx_delayed_pkt_nonblocking(data, len, timestamp) != DWM_SUCCESS)
    {
        return DWM_ERROR;
    }
    // wait until TXFRS in SYS_STATUS is set
    while (1) {
      dwm_spi_read(SYS_STATUS, SYS_STATUS_OFFSET, LEN_SYS_STATUS, sysStatus);
      if (sysStatus[0] & (0x1 << 7)) {
            break;
        }
    }

    return DWM_SUCCESS;
}

dwm1000_status dwm_tx_pkt_blocking(uint8_t *data, uint32_t len) {
  uint64_t reg64;
  uint32_t reg32, maxLen;
  uint8_t sysCtrl[LEN_SYS_CTRL];
  uint8_t txFctrl[LEN_TX_FCTRL];
  uint8_t sysStatus[LEN_SYS_STATUS];

  // check PHR_MODE in SYS_CFG for max frame length
  dwm_spi_read(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, sysCtrl);
  reg32 = (bytes_to_32bit(sysCtrl) & (0x3 << 16)) >> 16;
  if (reg32 == 0x0) {
    maxLen = 0x7F; // 127 bytes max if PHR_MODE = 00
  } else if (reg32 == 0x3) {
    maxLen = 0x3FF; // 1023 bytes max if PHR_MODE = 11 (long frames mode)
  } else {
    return DWM_ERROR;
  }

  // add CRC length (2 bytes) and check length
  if ((len + 2) > maxLen) {
    return DWM_ERROR;
  }

  // DWM1000 must be in IDLE mode before any RX/TX
  dwm_idle_mode();

  // write data into the TX_BUFFER at offset 0x0
  dwm_spi_write(TX_BUFFER, 0x0, len, data);

  // set TFLEN = (len + 2) and TXBOFFS = 0 in TX_FCTRL register
  dwm_spi_read(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, txFctrl);
  reg64 = bytes_to_40bit(txFctrl);
  reg64 &= ~((uint64_t)0x7F << 0);
  reg64 |= (len + 2); // TFLEN must include CRC length in bytes
  reg64 &= ~((uint64_t)0x3FF << 22);
  dwm_spi_write(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, (uint8_t *)(&reg64));

  // set TXSTRT (do not set SFCST to allow for CRC)
  reg32 = (0x1 << 1);
  dwm_spi_write(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, (uint8_t *)(&reg32));

  // wait until TXFRS in SYS_STATUS is set
  while (1) {
    dwm_spi_read(SYS_STATUS, SYS_STATUS_OFFSET, LEN_SYS_STATUS, sysStatus);
    if (sysStatus[0] & (0x1 << 7)) {
      break;
    }
  }

  dwm_idle_mode();

  return DWM_SUCCESS;
}

uint64_t dwm_get_raw_curr_timestamp(void)
{
  uint8_t currTime[LEN_TX_TIME];
  uint64_t raw_ts;
  dwm_spi_read(SYS_TIME, SYS_TIME_OFFSET, LEN_SYS_TIME, currTime);
  raw_ts = bytes_to_40bit(currTime);
  return raw_ts;
}

uint64_t dwm_get_raw_tx_timestamp(void)
{
  uint8_t txTime[LEN_TX_TIME];
  uint64_t raw_ts;
  dwm_spi_read(TX_TIME, TX_TIME_OFFSET, LEN_TX_TIME, txTime);
  raw_ts = bytes_to_40bit(txTime);
  return raw_ts;
}


uint64_t dwm_get_raw_rx_timestamp(void)
{
  uint8_t rxTime[LEN_RX_TIME];
  uint64_t raw_ts;
  dwm_spi_read(RX_TIME, RX_TIME_OFFSET, LEN_TX_TIME, rxTime);
  raw_ts = bytes_to_40bit(rxTime);
  return raw_ts;
}

uint64_t dwm_get_sys_status(void)
{
  dwm1000_status status;
  uint8_t sysStatus[LEN_SYS_STATUS];
  uint64_t reg64;

  dwm_spi_read(SYS_STATUS, SYS_STATUS_OFFSET, LEN_SYS_STATUS, sysStatus);
  reg64 = bytes_to_40bit(sysStatus);

  return reg64;
}

dwm1000_status dwm_enable_autorx(void)
{
  uint8_t sysCfg[LEN_SYS_CFG];
  uint32_t reg32;

  // set the RXAUTOR bit in SYS_CFG
  dwm_spi_read(SYS_CFG, SYS_CFG_OFFSET, LEN_SYS_CFG, sysCfg);
  reg32 = bytes_to_32bit(sysCfg);
  reg32 |= ((uint32_t)0x1 << 29);
  dwm_spi_write(SYS_CFG, SYS_CFG_OFFSET, LEN_SYS_CFG, (uint8_t *)(&reg32));

  return DWM_SUCCESS;
}

dwm1000_status dwm_get_tx_timestamp(double *timestamp) {
  uint8_t txTime[LEN_TX_TIME];
  // return TX_STAMP field
  dwm_spi_read(TX_TIME, TX_TIME_OFFSET, LEN_TX_TIME, txTime);
  *timestamp = fmod((double)bytes_to_40bit(txTime), TIME_OVERFLOW) * TIME_RES_US;
  return DWM_SUCCESS;
}

dwm1000_status dwm_rx_pkt_nonblocking(uint8_t *buf, uint8_t buf_len, uint32_t *rx_len) {
  dwm1000_status status;
  uint8_t sysStatus[LEN_SYS_STATUS];
  uint64_t reg64;
  uint8_t rxFinfo[LEN_RX_FINFO];

  if (buf == NULL || rx_len == NULL)
  {
      return DWM_ERROR;
  }

  dwm_spi_read(SYS_STATUS, SYS_STATUS_OFFSET, LEN_SYS_STATUS, sysStatus);
  reg64 = bytes_to_40bit(sysStatus);

  status = DWM_SUCCESS;

  // check for errors
  if (reg64 & DWM_ERR_RXPHE) {
      status = DWM_ERR_RXPHE;
  }
  else if (reg64 & DWM_ERR_RXFCE) {
      status = DWM_ERR_RXFCE;
  }
  else if (reg64 & DWM_ERR_RXRFSL) {
      status = DWM_ERR_RXRFSL;
  }
  else if (reg64 & DWM_ERR_LDEERR) {
      status = DWM_ERR_LDEERR;
  }
  else if (reg64 & DWM_ERR_RXPTO) {
      status = DWM_ERR_RXPTO;
  }
  else if (reg64 & DWM_ERR_RFPLL_LL) {
      status = DWM_ERR_RFPLL_LL;
  }
  else if (reg64 & DWM_ERR_CLKPLL_LL) {
      status = DWM_ERR_CLKPLL_LL;
  }
  else if (reg64 & DWM_ERR_RXSFDTO) {
      status = DWM_ERR_RXSFDTO;
  }
  else if (reg64 & DWM_ERR_AFFREJ) {
      status = DWM_ERR_AFFREJ;
  }

  // handle errors
  if (status != DWM_SUCCESS) {
    dwm_reset_rx();
    return status;
  }

  // get RXFLEN in RX_FINFO
  dwm_spi_read(RX_FINFO, RX_FINFO_OFFSET, LEN_RX_FINFO, rxFinfo);
  *rx_len = bytes_to_32bit(rxFinfo) & 0x7F;

  // read data from RX_BUFFER at offset 0x0
  dwm_spi_read(RX_BUFFER, 0x0, MIN(buf_len, (*rx_len)), buf);

  return DWM_SUCCESS;
}



dwm1000_status dwm_rx_pkt_blocking(uint8_t *data, uint32_t len, uint32_t *rx_len) {
  uint64_t reg64;
  dwm1000_status status;
  uint32_t reg32, msgLen;
  uint8_t rxFinfo[LEN_RX_FINFO];
  uint8_t sysStatus[LEN_SYS_STATUS];

  // DWM1000 must be in IDLE mode before any RX/TX
  dwm_idle_mode();

  // set RXENAB in SYS_CTRL
  reg32 = ((uint32_t)0x1 << 8);
  dwm_spi_write(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, (uint8_t *)(&reg32));

  while (1) {
    dwm_spi_read(SYS_STATUS, SYS_STATUS_OFFSET, LEN_SYS_STATUS, sysStatus);
    reg64 = bytes_to_40bit(sysStatus);

    // wait until RXDFR in SYS_STATUS is set
    if (reg64 & ((uint64_t)0x1 << 13)) {
      status = DWM_SUCCESS;
      break;
    }

    // check for errors
    if (reg64 & DWM_ERR_RXPHE) {
      status = DWM_ERR_RXPHE;
      break;
    }
    if (reg64 & DWM_ERR_RXFCE) {
      status = DWM_ERR_RXFCE;
      break;
    }
    if (reg64 & DWM_ERR_RXRFSL) {
      status = DWM_ERR_RXRFSL;
      break;
    }
    if (reg64 & DWM_ERR_LDEERR) {
      status = DWM_ERR_LDEERR;
      break;
    }
    if (reg64 & DWM_ERR_RXPTO) {
      status = DWM_ERR_RXPTO;
      break;
    }
    if (reg64 & DWM_ERR_RFPLL_LL) {
      status = DWM_ERR_RFPLL_LL;
      break;
    }
    if (reg64 & DWM_ERR_CLKPLL_LL) {
      status = DWM_ERR_CLKPLL_LL;
      break;
    }
    if (reg64 & DWM_ERR_RXSFDTO) {
      status = DWM_ERR_RXSFDTO;
      break;
    }
    if (reg64 & DWM_ERR_AFFREJ) {
      status = DWM_ERR_AFFREJ;
      break;
    }
  }

  // handle errors
  if (status != DWM_SUCCESS) {
    dwm_reset_rx();
    return status;
  }

  // get RXFLEN in RX_FINFO
  dwm_spi_read(RX_FINFO, RX_FINFO_OFFSET, LEN_RX_FINFO, rxFinfo);
  msgLen = bytes_to_32bit(rxFinfo) & 0x7F;

  *rx_len = msgLen - DWM_CRC_LEN;

  // read data from RX_BUFFER at offset 0x0
  dwm_spi_read(RX_BUFFER, 0x0, MIN(len, msgLen), data);

  // go back to idle mode for power saving
  dwm_idle_mode();

  return status;
}


dwm1000_status dwm_enable_rx_timestamp(void) {
  uint16_t reg16;
  uint32_t reg32;
  dwm1000_prf prf;
  dwm1000_preamble_code code;
  uint8_t otpCtrl[LEN_OTP_CTRL];
  uint8_t ldeCfg1[LEN_LDE_CFG1];
  uint8_t ldeCfg2[LEN_LDE_CFG2];
  uint8_t ldeRepc[LEN_LDE_REPC];
  uint8_t pmscCtrl1[LEN_PMSC_CTRL1];

  DEBUG_PRINT("setting RX timestamps...\r\n");

  dwm_set_system_clock(XTI_9MHZ);

  // set LDERUNE bit in PMSC_CTRL1 register
  dwm_spi_read(PMSC, PMSC_CTRL1_OFFSET, LEN_PMSC_CTRL1, pmscCtrl1);
  reg32 = bytes_to_32bit(pmscCtrl1);
  reg32 |= ((uint32_t)0x1 << 17);
  dwm_spi_write(PMSC, PMSC_CTRL1_OFFSET, LEN_PMSC_CTRL1, (uint8_t *)(&reg32));
  dwm_spi_read(PMSC, PMSC_CTRL1_OFFSET, LEN_PMSC_CTRL1, pmscCtrl1);
  reg32 = bytes_to_32bit(pmscCtrl1);
  DEBUG_PRINTF("PMSC_CTRL1: 0x%08x\n\r", reg32);

  // set LDELOAD bit in OTP_CTRL register (forces LDE algo load)
  reg16 = ((uint16_t)0x1 << 15);
  dwm_spi_write(OTP_IF, OTP_CTRL_OFFSET, LEN_OTP_CTRL, (uint8_t *)(&reg16));
  dwm_spi_read(OTP_IF, OTP_CTRL_OFFSET, LEN_OTP_CTRL, otpCtrl);
  reg16 = bytes_to_16bit(otpCtrl);
  DEBUG_PRINTF("OTP_CTRL: 0x%04x\n\r", reg16);

  // loop until the bit is cleared (means load is done)
  while (1) {
    dwm_spi_read(OTP_IF, OTP_CTRL_OFFSET, LEN_OTP_CTRL, otpCtrl);
    reg16 = bytes_to_16bit(otpCtrl);
    reg16 &= ((uint16_t)0x1 << 15);
    if (reg16 == 0x0) {
      break;
    }
  }

  // set LDE_CFG1 in LDE_IF
  ldeCfg1[0] = 0x6D;
  dwm_spi_write(LDE_CTRL, LDE_CFG1_OFFSET, LEN_LDE_CFG1, ldeCfg1);
  dwm_spi_read(LDE_CTRL, LDE_CFG1_OFFSET, LEN_LDE_CFG1, ldeCfg1);
  DEBUG_PRINTF("LDE_CFG1: 0x%02x\n\r", ldeCfg1[0]);

  // set LDE_CFG2 in LDE_IF
  dwm_get_rx_prf(&prf);
  switch (prf) {
    case PRF_16MHZ:
      reg16 = 0x1607U;
      break;
    case PRF_64MHZ:
      reg16 = 0x0607U;
      break;
    default:
      return DWM_INVALID_CONFIG;
  }
  dwm_spi_write(LDE_CTRL, LDE_CFG2_OFFSET, LEN_LDE_CFG2, (uint8_t *)(&reg16));
  dwm_spi_read(LDE_CTRL, LDE_CFG2_OFFSET, LEN_LDE_CFG2, ldeCfg2);
  reg16 = bytes_to_16bit(ldeCfg2);
  DEBUG_PRINTF("LDE_CFG2: 0x%04x\n\r", reg16);

  // set LDE_REPC in LDE_IF
  dwm_get_rx_preamble_code(&code);
  switch (code) {
    case PREAMBLE_CODE_1:
      reg16 = 0x5998;
      break;
    case PREAMBLE_CODE_2:
      reg16 = 0x5998;
      break;
    case PREAMBLE_CODE_3:
      reg16 = 0x51EA;
      break;
    case PREAMBLE_CODE_4:
      reg16 = 0x428E;
      break;
    case PREAMBLE_CODE_5:
      reg16 = 0x451E;
      break;
    case PREAMBLE_CODE_6:
      reg16 = 0x2E14;
      break;
    case PREAMBLE_CODE_7:
      reg16 = 0x8000;
      break;
    case PREAMBLE_CODE_8:
      reg16 = 0x51EA;
      break;
    case PREAMBLE_CODE_9:
      reg16 = 0x28F4;
      break;
    case PREAMBLE_CODE_10:
      reg16 = 0x3332;
      break;
    case PREAMBLE_CODE_11:
      reg16 = 0x3AE0;
      break;
    case PREAMBLE_CODE_12:
      reg16 = 0x3D70;
      break;
    case PREAMBLE_CODE_17:
      reg16 = 0x3332;
      break;
    case PREAMBLE_CODE_18:
      reg16 = 0x35C2;
      break;
    case PREAMBLE_CODE_19:
      reg16 = 0x35C2;
      break;
    case PREAMBLE_CODE_20:
      reg16 = 0x47AE;
      break;
  }
  dwm_spi_write(LDE_CTRL, LDE_REPC_OFFSET, LEN_LDE_REPC, (uint8_t *)(&reg16));
  dwm_spi_read(LDE_CTRL, LDE_REPC_OFFSET, LEN_LDE_REPC, ldeRepc);
  reg16 = bytes_to_16bit(ldeRepc);
  DEBUG_PRINTF("LDE_REPC: 0x%04x\n\r", reg16);

  dwm_set_system_clock(AUTO_CLK);

  return DWM_SUCCESS;
}


dwm1000_status dwm_disable_rx_timestamp(void) {
  uint32_t reg32;
  uint8_t pmscCtrl1[LEN_PMSC_CTRL1];
  // clear LDERUNE bit in PMSC_CTRL1 register
  dwm_spi_read(PMSC, PMSC_CTRL1_OFFSET, LEN_PMSC_CTRL1, pmscCtrl1);
  reg32 = bytes_to_32bit(pmscCtrl1);
  reg32 &= ~((uint32_t)0x1 << 17);
  dwm_spi_write(PMSC, PMSC_CTRL1_OFFSET, LEN_PMSC_CTRL1, (uint8_t *)(&reg32));
  dwm_spi_read(PMSC, PMSC_CTRL1_OFFSET, LEN_PMSC_CTRL1, pmscCtrl1);
  reg32 = bytes_to_32bit(pmscCtrl1);
  DEBUG_PRINTF("PMSC_CTRL1: 0x%08x\n\r", reg32);
  return DWM_SUCCESS;
}


dwm1000_status dwm_get_rx_timestamp(double *timestamp) {
  uint64_t reg64;
  uint8_t rxTime[LEN_RX_TIME];
  uint8_t sysStatus[LEN_SYS_STATUS];

  while (1) {
    // read RX_TIME register once LDEDONE is set in SYS_STATUS
    dwm_spi_read(SYS_STATUS, SYS_STATUS_OFFSET, LEN_SYS_STATUS, sysStatus);
    reg64 = bytes_to_40bit(sysStatus);

    if (reg64 & ((uint64_t)0x1U << 10)) {
      dwm_spi_read(RX_TIME, RX_TIME_OFFSET, LEN_RX_TIME, rxTime);
      break;
    }
  }

  // return RX_STAMP field
  *timestamp = fmod((double)bytes_to_40bit(rxTime), TIME_OVERFLOW) * TIME_RES_US;

  return DWM_SUCCESS;
}

dwm1000_status dwm_configure_IRQ(void *rx_callback, void *tx_callback) {
  uint32_t sysMask = 0;

  // make sure the ISR is registered
  // before we enable any interrupts
  if (dwm_hw_conf_int(rx_callback, tx_callback) != DWM_SUCCESS)
  {
      DEBUG_PRINT("dwm interrupt config failed!\r\n");
      return DWM_ERROR;
  }

  if (rx_callback) {
    sysMask |= ((uint32_t)0x1 << 13);
  }

  if (tx_callback) {
    sysMask |= ((uint32_t)0x1 << 7);
  }

  // configure the dwm to generate interrupts
  dwm_spi_write(SYS_MASK, SYS_MASK_OFFSET, LEN_SYS_MASK, (uint8_t *)(&sysMask));
  sysMask = 0;
  dwm_spi_read(SYS_MASK, SYS_MASK_OFFSET, LEN_SYS_MASK, (uint8_t *)(&sysMask));
  DEBUG_PRINTF("SYS_MASK: 0x%08x\n\r", sysMask);

  return DWM_SUCCESS;
}



dwm1000_status dwm_enable_rx_IRQ(void) {
  return DWM_SUCCESS;
}


dwm1000_status dwm_disable_rx_IRQ(void) {
  return DWM_SUCCESS;
}


dwm1000_status dwm_tx_pkt_nonblocking(uint8_t *data, uint32_t len) {
  uint64_t reg64;
  uint32_t reg32, maxLen;
  uint8_t sysCtrl[LEN_SYS_CTRL];
  uint8_t txFctrl[LEN_TX_FCTRL];
  uint8_t sysStatus[LEN_SYS_STATUS];

  // check PHR_MODE in SYS_CFG for max frame length
  dwm_spi_read(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, sysCtrl);
  reg32 = (bytes_to_32bit(sysCtrl) & (0x3UL << 16)) >> 16;
  if (reg32 == 0x0) {
    maxLen = 0x7F; // 127 bytes max if PHR_MODE = 00
  } else if (reg32 == 0x3) {
    maxLen = 0x3FF; // 1023 bytes max if PHR_MODE = 11 (long frames mode)
  } else {
    return DWM_ERROR;
  }

  // add CRC length (2 bytes) and check length
  if ((len + 2) > maxLen) {
    return DWM_ERROR;
  }

  // DWM1000 must be in IDLE mode before any RX/TX
  dwm_idle_mode();

  // write data into the TX_BUFFER at offset 0x0
  dwm_spi_write(TX_BUFFER, 0x0, len, data);

  // set TFLEN = (len + 2) and TXBOFFS = 0 in TX_FCTRL register
  dwm_spi_read(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, txFctrl);
  reg64 = bytes_to_40bit(txFctrl);
  reg64 &= ~((uint64_t)0x7F << 0);
  reg64 |= (len + 2); // TFLEN must include CRC length in bytes
  reg64 &= ~((uint64_t)0x3FF << 22);
  dwm_spi_write(TX_FCTRL, TX_FCTRL_OFFSET, LEN_TX_FCTRL, (uint8_t *)(&reg64));

  // set TXSTRT (do not set SFCST to allow for CRC)
  reg32 = (0x1 << 1);
  dwm_spi_write(SYS_CTRL, SYS_CTRL_OFFSET, LEN_SYS_CTRL, (uint8_t *)(&reg32));

  // go back to idle mode for power saving
  dwm_idle_mode();

  return DWM_SUCCESS;
}


dwm1000_status dwm_trigger_IRQ(void) {
  return DWM_SUCCESS;
}


/** @brief rushed enable leds for demo
 */
dwm1000_status dwm_GPIO_enable_leds(void) {

  //uint8_t gpio_en_mask[LEN_GPIO_MODE];
  uint32_t reg32;

  reg32 = 0;
  reg32 |= (0x01 << 6);
  reg32 |= (0x01 << 8);
  reg32 |= (0x01 << 10);
  reg32 |= (0x01 << 12);

  dwm_spi_write(GPIO_CTRL, GPIO_MODE_OFFSET, LEN_GPIO_MODE, (uint8_t*) &(reg32));

  reg32 = 0;
  // enable clock for gpio
  dwm_spi_read(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, (uint8_t*) &(reg32));
  reg32 |= (1UL<<18) | (1UL<<23);
  dwm_spi_write(PMSC, PMSC_CTRL0_OFFSET, LEN_PMSC_CTRL0, (uint8_t*) &(reg32));

  reg32 = 0;
  dwm_spi_read(PMSC, PMSC_LEDC_OFFSET, LEN_PMSC_LEDC, (uint8_t*) &(reg32));
  reg32 |= (1UL<<8) | (0xfUL<<16);

  printf("%lx\r\n", reg32);
  dwm_spi_write(PMSC, PMSC_LEDC_OFFSET, LEN_PMSC_LEDC, (uint8_t*) &(reg32));

  return DWM_SUCCESS;
}


dwm1000_status dwm_init(void) {
  uint16_t reg16;
  uint32_t reg32;
  uint8_t ecCtrl[LEN_EC_CTRL];
  uint8_t agcTune2[LEN_AGC_TUNE2];
  uint8_t agcTune3[LEN_AGC_TUNE3];

  // setup SPI
  dwm_spi_init();

  // perform soft reset
  //dwm_soft_reset();

  // enable AUTO clocking
  dwm_set_system_clock(AUTO_CLK);

  uint32_t devid = dwm_get_dev_id();
  DEBUG_PRINTF("dev id: 0x%08x\n\r", devid);
  // check devid for correct startup
  if (devid != DWM_DEV_ID) {
    return DWM_ERROR;
  }
  uint64_t ip = dwm_get_ip_address();
  DEBUG_PRINTF("ip address: 0x%016llx\n\r", ip);

  // set AGC_TUNE2 in AGC_CTRL
  reg32 = 0x2502A907U;
  dwm_spi_write(AGC_CTRL, AGC_TUNE2_OFFSET, LEN_AGC_TUNE2, (uint8_t *)(&reg32));
  dwm_spi_read(AGC_CTRL, AGC_TUNE2_OFFSET, LEN_AGC_TUNE2, agcTune2);
  reg32 = bytes_to_32bit(agcTune2);
  DEBUG_PRINTF("AGC_TUNE2: 0x%08x\n\r", reg32);

  // set AGC_TUNE3 in AGC_CTRL
  reg16 = 0x0035U;
  dwm_spi_write(AGC_CTRL, AGC_TUNE3_OFFSET, LEN_AGC_TUNE3, (uint8_t *)(&reg16));
  dwm_spi_read(AGC_CTRL, AGC_TUNE3_OFFSET, LEN_AGC_TUNE3, agcTune3);
  reg16 = bytes_to_16bit(agcTune3);
  DEBUG_PRINTF("AGC_TUNE3: 0x%04x\n\r", reg16);

  // set PLLLDT in EC_CTRL
  reg32 = (0x1 << 2);
  dwm_spi_write(EXT_SYNC, EC_CTRL_OFFSET, LEN_EC_CTRL, (uint8_t *)(&reg32));
  dwm_spi_read(EXT_SYNC, EC_CTRL_OFFSET, LEN_EC_CTRL, ecCtrl);
  reg32 = bytes_to_32bit(ecCtrl);
  DEBUG_PRINTF("EC_CTRL: 0x%08x\n\r", reg32);

  return DWM_SUCCESS;
}


// ****************************************************************************
// Debugging Routines
// ****************************************************************************


void print_settings(void) {
  uint8_t size;
  dwm1000_prf prf;
  dwm1000_psr psr;
  dwm1000_bitrate rate;
  dwm1000_channel channel;
  dwm1000_preamble_code code;

  DEBUG_PRINT("\n\r");

  if (dwm_get_channel(&channel) != DWM_SUCCESS) {
    DEBUG_PRINT("get channel error\n\r");
  } else {
    DEBUG_PRINTF("channel: %d\n\r", channel);
  }

  if (dwm_get_rx_prf(&prf) != DWM_SUCCESS) {
    DEBUG_PRINT("get RX PRF error\n\r");
  } else {
    switch (prf) {
      case PRF_16MHZ:
        DEBUG_PRINT("RX PRF: 16MHz\n\r");
        break;
      case PRF_64MHZ:
        DEBUG_PRINT("RX PRF: 64MHz\n\r");
        break;
    }
  }
  if (dwm_get_tx_prf(&prf) != DWM_SUCCESS) {
    DEBUG_PRINT("get TX PRF error\n\r");
  } else {
    switch (prf) {
      case PRF_16MHZ:
        DEBUG_PRINT("TX PRF: 16MHz\n\r");
        break;
      case PRF_64MHZ:
        DEBUG_PRINT("TX PRF: 64MHz\n\r");
        break;
    }
  }

  if (dwm_get_tx_psr(&psr) != DWM_SUCCESS) {
    DEBUG_PRINT("get TX PSR error\n\r");
  } else {
    switch (psr) {
      case PSR_64:
        DEBUG_PRINT("TX PSR: 64\n\r");
        break;
      case PSR_128:
        DEBUG_PRINT("TX PSR: 128\n\r");
        break;
      case PSR_256:
        DEBUG_PRINT("TX PSR: 256\n\r");
        break;
      case PSR_512:
        DEBUG_PRINT("TX PSR: 512\n\r");
        break;
      case PSR_1024:
        DEBUG_PRINT("TX PSR: 1024\n\r");
        break;
      case PSR_1536:
        DEBUG_PRINT("TX PSR: 1536\n\r");
        break;
      case PSR_2048:
        DEBUG_PRINT("TX PSR: 2048\n\r");
        break;
      case PSR_4096:
        DEBUG_PRINT("TX PSR: 4096\n\r");
        break;
    }
  }

  if (dwm_get_rx_pac_size(&size) != DWM_SUCCESS) {
    DEBUG_PRINT("get RX PAC size error\n\r");
  } else {
    DEBUG_PRINTF("RF PAC size: %d\n\r", size);
  }

  if (dwm_get_rx_preamble_code(&code) != DWM_SUCCESS) {
    DEBUG_PRINT("get RX preamble code error\n\r");
  } else {
    DEBUG_PRINTF("RX preamble code: %d\n\r", code);
  }

  if (dwm_get_tx_preamble_code(&code) != DWM_SUCCESS) {
    DEBUG_PRINT("get TX preamble code error\n\r");
  } else {
    DEBUG_PRINTF("TX preamble code: %d\n\r", code);
  }


  if (dwm_get_tx_bit_rate(&rate) != DWM_SUCCESS) {
    DEBUG_PRINT("get TX bit rate error\n\r");
  } else {
      switch (rate) {
      case RF_110KBps:
        DEBUG_PRINT("TX bit rate: 110KBps\n\r");
        break;
      case RF_850KBps:
        DEBUG_PRINT("TX bit rate: 850KBps\n\r");
        break;
      case RF_6800KBps:
        DEBUG_PRINT("TX bit rate: 6800KBps\n\r");
        break;
    }
  }
  if (dwm_get_rx_bit_rate(&rate) != DWM_SUCCESS) {
    DEBUG_PRINT("get RX bit rate error\n\r");
  } else {
    switch (rate) {
      case RF_110KBps:
        DEBUG_PRINT("RX bit rate: 110KBps\n\r");
        break;
      case RF_850KBps:
        DEBUG_PRINT("RX bit rate: 850KBps\n\r");
        break;
      case RF_6800KBps:
        DEBUG_PRINT("RX bit rate: 6800KBps\n\r");
        break;
    }
  }

  DEBUG_PRINT("\n\r");
}


void print_status(dwm1000_status status) {
  switch (status) {
    case DWM_SUCCESS:
      DEBUG_PRINT("Generic Success\r\n");
      break;
    case DWM_ERROR:
      DEBUG_PRINT("ERROR: Generic Error\r\n");
      break;
    case DWM_INVALID_CONFIG:
      DEBUG_PRINT("ERROR: Invalid Configuration Settings\r\n");
      break;
    case DWM_ERR_RXPHE:
      DEBUG_PRINT("ERROR: Receiver PHY Header Error\r\n");
      break;
    case DWM_ERR_RXFCE:
      DEBUG_PRINT("ERROR: Receiver CRC Error\r\n");
      break;
    case DWM_ERR_RXRFSL:
      DEBUG_PRINT("ERROR: Receiver Reed Solomon Frame Sync Loss\r\n");
      break;
    case DWM_ERR_LDEERR:
      DEBUG_PRINT("ERROR: Leading edge detection processing error\r\n");
      break;
    case DWM_ERR_RXPTO:
      DEBUG_PRINT("ERROR: Preamble detection timeout\r\n");
      break;
    case DWM_ERR_RFPLL_LL:
      DEBUG_PRINT("ERROR: RF PLL Losing Lock\r\n");
      break;
    case DWM_ERR_CLKPLL_LL:
      DEBUG_PRINT("ERROR: Clock PLL Losing Lock\r\n");
      break;
    case DWM_ERR_RXSFDTO:
      DEBUG_PRINT("ERROR: Receive SFD timeout\r\n");
      break;
    case DWM_ERR_AFFREJ:
      DEBUG_PRINT("ERROR: Automatic Frame Filtering rejection\r\n");
      break;
  }
}
