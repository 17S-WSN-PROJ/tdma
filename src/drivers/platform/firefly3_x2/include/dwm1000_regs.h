/*
 * dwm1000_regs.h - register map for dwm1000
 */

#ifndef _DWM100_REGS_H_
#define _DWM100_REGS_H_

/*
 * SPI register IDs
 */
typedef enum _dwm1000_reg_ids {
  DEV_ID     = 0x00U, // Device Identifier – includes device type and revision info
  EUI        = 0x01U, // Extended Unique Identifier
  PANADR     = 0x03U, // PAN Identifier and Short Address
  SYS_CFG    = 0x04U, // System Configuration bitmap
  SYS_TIME   = 0x06U, // System Time Counter (40-bit)
  TX_FCTRL   = 0x08U, // Transmit Frame Control
  TX_BUFFER  = 0x09U, // Transmit Data Buffer
  DX_TIME    = 0x0AU, // Delayed Send or Receive Time (40-bit)
  RX_FWTO    = 0x0CU, // Receive Frame Wait Timeout Period
  SYS_CTRL   = 0x0DU, // System Control Register
  SYS_MASK   = 0x0EU, // System Event Mask Register
  SYS_STATUS = 0x0FU, // System Event Status Register
  RX_FINFO   = 0x10U, // RX Frame Information
  RX_BUFFER  = 0x11U, // Receive Data Buffer
  RX_FQUAL   = 0x12U, // Rx Frame Quality information
  RX_TTCKI   = 0x13U, // Receiver Time Tracking Interval
  RX_TTCKO   = 0x14U, // Receiver Time Tracking Offset
  RX_TIME    = 0x15U, // Receive Message Time of Arrival
  TX_TIME    = 0x17U, // Transmit Message Time of Sending
  TX_ANTD    = 0x18U, // 16-bit Delay from Transmit to Antenna
  SYS_STATE  = 0x19U, // System State information
  ACK_RESP_T = 0x1AU, // Acknowledgment Time and Response Time
  RX_SNIFF   = 0x1DU, // Pulsed Preamble Reception Configuration
  TX_POWER   = 0x1EU, // TX Power Control
  CHAN_CTRL  = 0x1FU, // Channel Control
  USR_SFD    = 0x21U, // User-specified short/long TX/RX SFD sequences
  AGC_CTRL   = 0x23U, // Automatic Gain Control configuration
  EXT_SYNC   = 0x24U, // External synchronization control
  ACC_MEM    = 0x25U, // Read access to accumulator data
  GPIO_CTRL  = 0x26U, // Peripheral register bus 1 access - GPIO control
  DRX_CONF   = 0x27U, // Digital Receiver configuration
  RF_CONF    = 0x28U, // Analog RF Configuration
  TX_CAL     = 0x2AU, // Transmitter calibration block
  FS_CTRL    = 0x2BU, // Frequency synthesizer control block
  AON        = 0x2CU, // Always-On register set
  OTP_IF     = 0x2DU, // One Time Programmable Memory Interface
  LDE_CTRL   = 0x2EU, // Leading edge detection control block
  DIG_DIAG   = 0x2FU, // Digital Diagnostics Interface
  PMSC       = 0x36U, // Power Management System Control Block
} dwm1000_reg_id;


/*
 * Length in bytes of data sent to or received from DWM1000 for an SPI
 * transaction to a given SPI register set. Specific offsets to registers
 * inside these register maps is given in #define's below.
 */
typedef enum _dwm1000_reg_lengths {
  LEN_DEV_ID     = 4U,
  LEN_EUI        = 8U,
  LEN_PANADR     = 4U,
  LEN_SYS_CFG    = 4U,
  LEN_SYS_TIME   = 5U,
  LEN_TX_FCTRL   = 5U,
  LEN_TX_BUFFER  = 1024U,
  LEN_DX_TIME    = 5U,
  LEN_RX_FWTO    = 2U,
  LEN_SYS_CTRL   = 4U,
  LEN_SYS_MASK   = 4U,
  LEN_SYS_STATUS = 5U,
  LEN_RX_FINFO   = 4U,
  LEN_RX_BUFFER  = 1024U,
  LEN_RX_FQUAL   = 8U,
  LEN_RX_TTCKI   = 4U,
  LEN_RX_TTCKO   = 5U,
  LEN_RX_TIME    = 14U,
  LEN_TX_TIME    = 10U,
  LEN_TX_ANTD    = 2U,
  LEN_SYS_STATE  = 5U,
  LEN_ACK_RESP_T = 4U,
  LEN_RX_SNIFF   = 4U,
  LEN_TX_POWER   = 4U,
  LEN_CHAN_CTRL  = 4U,
  LEN_USR_SFD    = 41U,
  LEN_AGC_CTRL   = 32U,
  LEN_EXT_SYNC   = 12U,
  LEN_ACC_MEM    = 4064U,
  LEN_GPIO_CTRL  = 44U,
  LEN_DRX_CONF   = 44U,
  LEN_RF_CONF    = 58U,
  LEN_TX_CAL     = 52U,
  LEN_FS_CTRL    = 21U,
  LEN_AON        = 12U,
  LEN_OTP_IF     = 18U,
  LEN_LDE_CTRL   = 0U,
  LEN_DIG_DIAG   = 41U,
  LEN_PMSC       = 48U,
} dwm1000_reg_length;


/*
 * Specific register offsets and lengths inside of register map above.
 */

#define PMSC_CTRL0_OFFSET  0x0000U
#define PMSC_CTRL1_OFFSET  0x0004U
#define PMSC_SNOZT_OFFSET  0x000CU
#define PMSC_TXFSEQ_OFFSET 0x0026U
#define PMSC_LEDC_OFFSET   0x0028U
#define CHAN_CTRL_OFFSET   0x0000U
#define TX_FCTRL_OFFSET    0x0000U
#define FS_PLLCFG_OFFSET   0x0007U
#define FS_PLLTUNE_OFFSET  0x000BU
#define RF_RXCTRLH_OFFSET  0x000BU
#define RF_TXCTRL_OFFSET   0x000CU
#define DRX_TUNE0b_OFFSET  0x0002U
#define DRX_TUNE1a_OFFSET  0x0004U
#define DRX_TUNE1b_OFFSET  0x0006U
#define DRX_TUNE2_OFFSET   0x0008U
#define DRX_TUNE4h_OFFSET  0x0026U
#define OTP_CTRL_OFFSET    0x0006U
#define AGC_TUNE1_OFFSET   0x0004U
#define AGC_TUNE2_OFFSET   0x000CU
#define AGC_TUNE3_OFFSET   0x0012U
#define LDE_CFG1_OFFSET    0x0806U
#define LDE_CFG2_OFFSET    0x1806U
#define LDE_REPC_OFFSET    0x2804U
#define TC_PGDELAY_OFFSET  0x000BU
#define SYS_CFG_OFFSET     0x0000U
#define TX_POWER_OFFSET    0x0000U
#define SYS_CTRL_OFFSET    0x0000U
#define SYS_STATUS_OFFSET  0x0000U
#define RX_FINFO_OFFSET    0x0000U
#define EC_CTRL_OFFSET     0x0000U
#define RX_TIME_OFFSET     0x0000U
#define TX_TIME_OFFSET     0x0000U
#define SYS_TIME_OFFSET    0x0000U
#define SYS_MASK_OFFSET    0x0000U
#define GPIO_MODE_OFFSET   0x0000U
#define AON_CTRL_OFFSET    0x0002U
#define AON_CFG0_OFFSET    0x0006U
#define AON_CFG1_OFFSET    0x000AU

#define LEN_PMSC_CTRL0     0x4U
#define LEN_PMSC_CTRL1     0x4U
#define LEN_PMSC_SNOZT     0x1U
#define LEN_PMSC_LEDC      0x1U
#define LEN_TXFSEQ_SNOZT   0x2U
#define LEN_LEDC_SNOZT     0x4U
#define LEN_FS_PLLCFG      0x4U
#define LEN_FS_PLLTUNE     0x1U
#define LEN_RF_RXCTRLH     0x1U
#define LEN_RF_TXCTRL      0x4U
#define LEN_DRX_TUNE0b     0x2U
#define LEN_DRX_TUNE1a     0x2U
#define LEN_DRX_TUNE1b     0x2U
#define LEN_DRX_TUNE2      0x4U
#define LEN_DRX_TUNE4h     0x2U
#define LEN_OTP_CTRL       0x2U
#define LEN_AGC_TUNE1      0x2U
#define LEN_AGC_TUNE2      0x4U
#define LEN_AGC_TUNE3      0x2U
#define LEN_LDE_CFG1       0x1U
#define LEN_LDE_CFG2       0x2U
#define LEN_LDE_REPC       0x2U
#define LEN_TC_PGDELAY     0x1U
#define LEN_EC_CTRL        0x4U
#define LEN_GPIO_MODE      0x4U
#define LEN_AON_CTRL       0x4U
#define LEN_AON_CFG0       0x4U
#define LEN_AON_CFG1       0x4U

#endif
