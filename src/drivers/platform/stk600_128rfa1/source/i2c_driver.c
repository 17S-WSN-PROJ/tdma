
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
 *  Andrew Jameson
 *******************************************************************************/

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>
#include <include.h>
#include <stdio.h>
#include <ulib.h>
#include <nrk_error.h>
#include <nrk.h>
#include <stdint.h>
#include <basic_rf.h>
#include <nrk_timer.h>
#include <twi_base_calls.h>
#include <math.h>

#define DEBUG 0

// Both of these variables are set in the set_status() method
uint8_t i2c_address; /* The I2C address to read/write to */
uint8_t base_memory_location; /* The address in the device that the read/write should start from */

uint8_t is_open;
uint8_t channel;
uint8_t write(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size);

uint8_t dev_manager_ff_sensors(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    switch (action) {

        /**
         * Note that currently the functions below initialize both the ADC as
         * well as the I2C methods of getting inputs, even though only one of
         * them will be used at any given time.
         */

        case INIT:
            return init(action, opt, buffer, size);

        case OPEN:
            return open(action, opt, buffer, size);

        case READ:
            return read(action, opt, buffer, size);

        case WRITE:
            return write(action, opt, buffer, size);

        case CLOSE:
            return close(action, opt, buffer, size);

        case GET_STATUS:
            return get_status(action, opt, buffer, size);

        case SET_STATUS:
            return set_status(action, opt, buffer, size);

        default:
            nrk_kernel_error_add(NRK_DEVICE_DRIVER, 0);
            return 0;
    }
}

uint8_t init(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

        init_i2c();
        is_open = 0;
        return 1;
}

uint8_t open(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    if (is_open == 1) return NRK_ERROR;
    is_open = 1;
    if (opt & READ_FLAG) {
        channel = 0;
        return NRK_OK;
    }
    if (opt & WRITE_FLAG) {
        return NRK_ERROR;
    }
    if (opt & APPEND_FLAG) {
        return NRK_ERROR;
    }
    if (((opt)&(READ_FLAG | WRITE_FLAG | APPEND_FLAG)) == 0)
        return NRK_ERROR;
    else return NRK_OK;

}

/**
 * Read from the specified I2C device into the 
 *
 * @return the number of bytes read
 *
 **/
uint8_t read(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

     return ee24xx_read_bytes(base_memory_location, size, buffer);
}

/**
 *
 *
 */
uint8_t write(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    return ee24xx_write_bytes(base_memory_location, size, buffer);

}

uint8_t close(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    is_open = 0;
    close_i2c(); // Shut down the I2C
    return NRK_OK;
}

// Returns the current I2C address that is being communicated with
uint8_t get_status(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {
 
    return i2c_address;
}

/**
 * This method is used to set up the device and location where the I2C communication
 * should begin.
 *
 * @param action - SET_STATUS by default
 * @param opt - this is the I2C address of the device that will be communicated with
 * @param buffer - not used in this method
 * @param size - this is the starting memory location which will be used for the
 *                 read/write
 *
 *  @returns NRK_OK if everything worked, NRK_ERROR if there was a problem.
 *
 */
uint8_t set_status(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    i2c_address = opt;
    base_memory_location = size;

    set_i2c_device(i2c_address);

    return NRK_OK;
}













