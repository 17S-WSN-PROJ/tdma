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
 *  Kaan Dogrusoz
 *******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <include.h>
#include <nrk.h>
#include <nrk_timer.h>
#include <nrk_error.h>
#include <nrk_driver.h>
#include <nrk_driver_list.h>
#include <twi_base_calls.h>
#include <nrk_error.h>
#include <nrk_ext_int.h>
#include <dwm1000.h>
#include <dwm1000_spi.h>
#include <ulib.h>
#include <hal_firefly3.h>

// DWM SPI constants
#define DWM1000_RESET_PIN (NRK_PORTG_0)

#define RW_SHIFT        (7)
#define SUBIND0_BIT     (1<<6)
#define SUBIND1_BIT     (1<<7)
#define REGID_MASK      (0x3f)

#define SUBIND0_SHIFT    (0)
#define SUBIND0_MASK     (0x7f)
#define SUBIND1_SHIFT    (7)
#define SUBIND1_MASK     (0xff)

#define MAX_TX_LEN       125
#define MAX_RX_LEN       125

static void (*_rx_callback) (void) = NULL;
static void (*_tx_callback) (void) = NULL;


/** @brief sets up a spi transaction
 *
 *  Will set CS to low but won't set
 *  it back to high, CS has to be set
 *  back to high after this function
 *  is called.
 */
void dwm_send_spihdr(uint8_t reg_id, uint16_t subind, uint8_t is_write)
{
    uint8_t pkt;
    pkt = 0;
    pkt |= is_write << RW_SHIFT;
    pkt |= reg_id & REGID_MASK;

    if (subind)
    {
        pkt |= SUBIND0_BIT;

        FASTSPI_TX(pkt);
        pkt = 0;

        // we may need an extended address
        if (subind > 0x7FFF)
        {
            pkt |=  SUBIND1_BIT;
        }

        pkt |= subind & SUBIND0_MASK;
        FASTSPI_TX(pkt);

        if (subind > 0x7FFF)
        {
            pkt = 0;
            pkt |= (subind >> SUBIND1_SHIFT) & SUBIND1_MASK;
            FASTSPI_TX(pkt);
        }
    }
    else
    {
        FASTSPI_TX(pkt);
    }
}

/* @brief configures fast spi
 */
void dwm_fast_spi()
{
    SPCR = BM(SPE) | BM(MSTR);
    SPSR = BM(SPI2X);
}

/** @brief inits the dwm platform specific stuff
 */
void dwm_spi_init()
{
    // configure the pin used for the reset line
    SPCR = BM(SPE) | BM(MSTR) | BM(SPR0);

    // reset the dwm1000
    nrk_gpio_direction(DWM1000_RESET_PIN, NRK_PIN_OUTPUT);
    nrk_gpio_clr(DWM1000_RESET_PIN);

    nrk_spin_wait_us(100);// wait 10ns
    // tristate the reset line
    nrk_gpio_direction(DWM1000_RESET_PIN, NRK_PIN_OUTPUT);
    nrk_gpio_set(DWM1000_RESET_PIN);

    // wait for the pll to lock
    nrk_spin_wait_us(100);
}

void dwm_ISR(void)
{
    uint32_t status;
    uint16_t flag_clear;
    dwm_spi_read(SYS_STATUS, SYS_STATUS_OFFSET, 4, (uint8_t*) &status);
    if ((status & ((uint32_t)0x1 << 7)) && _tx_callback)
    {
        flag_clear = 0xf8;
        dwm_spi_write(SYS_STATUS, SYS_STATUS_OFFSET, 2, &flag_clear);
        _tx_callback();
    }

    if ((status & ((uint32_t)0x1 << 13)) && _rx_callback)
    {
        flag_clear = 0x6f00;
        dwm_spi_write(SYS_STATUS, SYS_STATUS_OFFSET, 2, &flag_clear);
        _rx_callback();
    }
    return;
}

dwm1000_status dwm_hw_conf_int(void *rx_callback, void *tx_callback)
{
    // isr sanity check
    if (rx_callback == NULL && tx_callback == NULL)
    {
        return DWM_ERROR;
    }

    // init the external interrupt pins
    if (nrk_ext_int_configure(NRK_EXT_INT_2,
                              NRK_RISING_EDGE, dwm_ISR) < 0)
    {
       printf("External interrupt config failed\r\n");
       return DWM_ERROR;
    }

    _rx_callback = rx_callback;
    _tx_callback = tx_callback;

    nrk_ext_int_enable(NRK_EXT_INT_2);

    return DWM_SUCCESS;
}

/** @brief reads len bytes over spi
 *
 */
void dwm_spi_read(dwm1000_reg_id reg_id, uint16_t subind, uint32_t len, uint8_t *data)
{
    int i;
    if (len > MAX_TX_LEN)
    {
        printf("Trying to send to large of a packet");
        return;
    }

    SPI_ENABLE();
    // have to wait 500us to get out of deepsleep
    dwm_send_spihdr(reg_id, subind, FALSE);
    for (i = 0; i < len; i++)
    {
        FASTSPI_RX(data[i]);
    }
    SPI_DISABLE();
}

/** @brief sends len bytes over spi
 *
 */
void dwm_spi_write(dwm1000_reg_id reg_id, uint16_t subind, uint32_t len, uint8_t *data)
{
    int i;
    if (len > MAX_TX_LEN)
    {
        printf("Trying to send to large of a packet");
        return;
    }

    SPI_ENABLE();
    // have to wait 500 to get out of deepsleep
    dwm_send_spihdr(reg_id, subind, TRUE);
    for (i = 0; i < len; i++)
    {
        FASTSPI_TX(data[i]);
    }
    SPI_DISABLE();
}

void dwm_host_delay_us(uint16_t us)
{
    nrk_spin_wait_us(us);
}

