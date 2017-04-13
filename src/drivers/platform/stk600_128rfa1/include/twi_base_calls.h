/******************************************************************************
*
*  Filename: twi_base_calls.h
*  Author: Andrew Jameson
*  Last Updated: 2011-02-23
*
*  This is a header file for the basic read/write functionality of the TWI. The
*  source file was originally developed by Joerg Wunsch
*
*******************************************************************************/

#ifndef _TWI_BASE_CALLS_H_
#define	_TWI_BASE_CALLS_H_

// Function definitions

/**
 * Initializes the SCL and the SDA pins and sets the SCL frequency.
 *
 * @returns void
 *
 */ 
void init_i2c(void);

/**
 * Shuts off the SCL and the SDA pins
 *
 * @returns void
 *
 */
void close_i2c(void);

/**
 * Sets the address of the i2c device to be used.
 *
 * @param address - the address of the i2c device to be used.
 *
 * @returns void;
 */
void set_i2c_device(uint8_t address);

/*
 * Note [7]
 *
 * Read "len" bytes from EEPROM starting at "eeaddr" into "buf".
 *
 * This requires two bus cycles: during the first cycle, the device
 * will be selected (master transmitter mode), and the address
 * transfered.  Address bits exceeding 256 are transfered in the
 * E2/E1/E0 bits (subaddress bits) of the device selector.
 *
 * The second bus cycle will reselect the device (repeated start
 * condition, going into master receiver mode), and transfer the data
 * from the device to the TWI master.  Multiple bytes can be
 * transfered by ACKing the client's transfer.  The last transfer will
 * be NACKed, which the client will take as an indication to not
 * initiate further transfers.
 *
 * @param eeaddr - the starting address in the memory to read fromt
 * @param len - the number of bytes to read
 * @param *buf - a buffer to place the data that is read into
 *
 * @returns -1 if an error has occured, otherwise the number of bytes that were read
 *
 */
int ee24xx_read_bytes(uint16_t eeaddr, int len, uint8_t *buf);


/*
 * Write "len" bytes into EEPROM starting at "eeaddr" from "buf".
 *
 * This is a bit simpler than the previous function since both, the
 * address and the data bytes will be transfered in master transmitter
 * mode, thus no reselection of the device is necessary.  However, the
 * EEPROMs are only capable of writing one "page" simultaneously, so
 * care must be taken to not cross a page boundary within one write
 * cycle.  The amount of data one page consists of varies from
 * manufacturer to manufacturer: some vendors only use 8-byte pages
 * for the smaller devices, and 16-byte pages for the larger devices,
 * while other vendors generally use 16-byte pages.  We thus use the
 * smallest common denominator of 8 bytes per page, declared by the
 * macro PAGE_SIZE above.
 *
 * The function simply returns after writing one page, returning the
 * actual number of data byte written.  It is up to the caller to
 * re-invoke it in order to write further data.
 *
 * @param eeaddr - the starting address in the memory to write to
 * @param len - the number of bytes to write
 * @param *buf - a buffer containing the data to be written
 *
 * @returns -1 if an error has occured, otherwise the number of bytes that were written
 */
int ee24xx_write_page(uint16_t eeaddr, int len, uint8_t *buf);

/*
 * Wrapper around ee24xx_write_page() that repeats calling this
 * function until either an error has been returned, or all bytes
 * have been written.
 *
 * @param eeaddr - the starting address in the memory to write to
 * @param len - the number of bytes to write
 * @param *buf - a buffer containing the data to be written
 *
 * @returns -1 if an error has occured, otherwise the number of bytes that were written
 */
int ee24xx_write_bytes(uint16_t eeaddr, int len, uint8_t *buf);

/**
 *
 * Prints the error message from the TWI status register if an error has occurred.
 *
 * @returns void
 */
void error(void);


#endif	/* _TWI_BASE_CALLS_H_ */

