/*
 * dwm1000_spi.h - SPI library interface for dwm1000
 */

#ifndef _DWM100_SPI_H_
#define _DWM100_SPI_H_

#include <stdint.h>

#include "dwm1000.h"
#include "dwm1000_regs.h"

/** @brief initializes fast spi
 *
 *  Used post initialization of the dwm
 *  to have the fastest communication with
 *  the decawave as possible.
 */
void dwm_fast_spi(void);

/**
 * @brief initializes a master to slave connection with the DWM1000 chip.
 *        Data is always sent MSB first for the DWM1000.
 */
void dwm_spi_init(void);

/**
 * @brief sets up platform specific interrupt config
 *
 */
dwm1000_status dwm_hw_conf_int(void *rx_callback, void *tx_callback);

/**
 * @brief SPI read transaction.
 *
 * @param regId the register ID to read from
 * @param subIndex the subindex into the register to read from.
 * @param bufLen the length of data
 * @param data the array of bytes to hold data received
 */
void dwm_spi_read(dwm1000_reg_id regId, uint16_t subIndex, uint32_t bufLen, uint8_t *data);

/**
 * @brief SPI write transaction.
 *
 * @param regId the register ID to write to
 * @param subIndex the subindex into the register to write to
 * @param bufLen the length of data
 * @param data the array of bytes to send
 */
void dwm_spi_write(dwm1000_reg_id regId, uint16_t subIndex, uint32_t bufLen, uint8_t *data);

void dwm_host_delay_us(uint16_t us);

#endif
