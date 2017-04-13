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
 *  Anthony Rowe
 *  Zane Starr
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
#define BOSCH_EEPROM_ADDRESS 0xEE


int16_t eeprom_values[11]; /* Used to store the calibration values read from the Bosch's EEPROM */

// These eeprom register values are from the Bosch datasheet
#define AC1 (int16_t)eeprom_values[0]
#define AC2 (int16_t)eeprom_values[1]
#define AC3 (int16_t)eeprom_values[2]
#define AC4 (uint16_t)eeprom_values[3]
#define AC5 (uint16_t)eeprom_values[4]
#define AC6 (uint16_t)eeprom_values[5]
#define B1 (int16_t)eeprom_values[6]
#define B2 (int16_t)eeprom_values[7]
#define MB (int16_t)eeprom_values[8]
#define MC (int16_t)eeprom_values[9]
#define MD (int16_t)eeprom_values[10]

#define TEMP_BASE_REGISTER 0xF6 /* The address of the initial place to read from for temperature */
#define PRESS_BASE_REGISTER 0xF6 /* The address of the initial place to read from for pressure */

#define ADC_STARTUP_DELAY  1000
#define ADC_SETUP_DELAY  200

int32_t UT, UP; /* The uncompensated temperature and pressure values */

// Temp variables used in converting from the raw values to the finished product
int32_t X1, X2, X3, B3, B5, B6, P, T;
uint32_t B4, B7;

//uint16_t h_cnt;
uint8_t h_cnt;
uint8_t channel;
uint8_t is_open;

// VREF is set to VCC by default
#define ADC_INIT() \
    do { \
	ADCSRA = BM(ADPS0) | BM(ADPS1); \
} while (0)
	//ADMUX = BM(REFS0);  \

#define ADC_VREF_VCC() \
   do { \
} while(0)

	//ADMUX &= ~(BM(REFS1));  \
	//ADMUX |= BM(REFS0);  \

#define ADC_VREF_1_1() \
   do { \
	ADMUX &= ~(BM(REFS0));  \
	ADMUX |= BM(REFS1);  \
} while(0)


#define ADC_VREF_2_56() \
   do { \
	ADMUX |= BM(REFS1) | BM(REFS0);  \
} while(0)

#define ADC_SET_CHANNEL(channel) do { ADMUX &= ~0x1F; ADMUX |= (ADMUX & ~0x1F) | (channel); } while (0)

// Enables/disables the ADC
#define ADC_ENABLE() do { ADCSRA |= BM(ADEN); } while (0)
#define ADC_DISABLE() do { ADCSRA &= ~BM(ADEN); } while (0)

#define ADC_SAMPLE_SINGLE() \
    do { \
ADCSRA |= BM(ADSC); \
while (!(ADCSRA & 0x10)); \
} while(0)

// Macros for obtaining the latest sample value
#define ADC_GET_SAMPLE_10(x) \
do { \
x =  ADCL; \
x |= ADCH << 8; \
} while (0)

#define ADC_GET_SAMPLE_8(x) \
do { \
x = ((uint8_t) ADCL) >> 2; \
x |= ((int8_t) ADCH) << 6; \
} while (0)



int8_t dev_manager_ff3_sensors(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

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

int8_t init(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {
        // For handling the ADC
        // Set the pwr ctrl pin as output
	//DDRE = 0;
        //DDRE |= PWR_CTRL_MASK;
        //PORTE |= PWR_CTRL_MASK;
        nrk_gpio_direction(PWR_CTRL_PIN, NRK_PIN_OUTPUT);
        //nrk_gpio_clr(PWR_CTRL_PIN);
        nrk_gpio_set(PWR_CTRL_PIN);
	// Input for motion detector
        nrk_gpio_direction(NRK_PORTF_6, NRK_PIN_INPUT );
        nrk_gpio_clr(NRK_PORTF_6);
        nrk_gpio_direction(NRK_PORTE_2, NRK_PIN_INPUT);
        nrk_gpio_clr(NRK_PORTE_2);
	nrk_gpio_pullups(0);

        init_adc();
        
        // For handling the I2C
         init_i2c();

	set_i2c_device(BOSCH_EEPROM_ADDRESS);
        get_eeprom_values();
        is_open = 0;
        return 1;
    
}

int8_t open(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {
    // Note that here nothing needs to be done for the I2C. This is the original
    // code for the firefly 2.2 board driver
	if (is_open == 1) return NRK_ERROR;
    is_open = 1;
    if (opt & READ_FLAG) {
        // Turn on Sensor Node Power
        //PORTE &= ~(PWR_CTRL_MASK);
        //PORTE |= PWR_CTRL_MASK;
        nrk_gpio_set(PWR_CTRL_PIN);
        channel = 0;
        ADC_SET_CHANNEL(0);
        nrk_spin_wait_us(ADC_STARTUP_DELAY);
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

int8_t read(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    int32_t value_from_sensor;



    ADC_ENABLE();
    uint8_t count = 0;
    // key and value get passed as opt and size
    uint8_t key = opt;
    uint8_t value = size;

    if (size != 1 && size != 2 && size != 4) return 0;
    if ((channel != TEMP2) && (channel != PRESS) && channel <= 7) {
        /* Conversion to 8-bit value*/
        uint16_t val = get_adc_val();

        if (size == 2) {
            buffer[count] = val & 0xFF;
            count++;
            buffer[count] = (val >> 8) & 0xFF;
        }

        if (size == 1) {
            buffer[count] = (val >> 2) & 0xFF;
        }


    } else if (channel == MOTION ){

	// Pull the pin low to dissipate charge otherwise it always floats high
	// In the future, we either need a buffer or a tiny pull down 
        nrk_gpio_direction(NRK_PORTE_2, NRK_PIN_OUTPUT);
        nrk_gpio_clr(NRK_PORTE_2);
        nrk_gpio_direction(NRK_PORTE_2, NRK_PIN_INPUT);
	if(size==2) {buffer[count]=nrk_gpio_get(NRK_PORTE_2); count++; buffer[count]=0;  }
	if(size==1) {buffer[count]=nrk_gpio_get(NRK_PORTE_2); }
     } 
	else if (channel == AUDIO_P2P) {
        /* Conversion to 8-bit value*/
        //uint16_t val=get_adc_val();
        uint16_t val, min, max;
        uint16_t i;
        max = 0;
        min = 1025;
        for (i = 0; i < 256; i++) {
            // 10ms snapshot at around 11KHz measured on scope
	    ADC_SAMPLE_SINGLE();
            nrk_spin_wait_us(32);
    	    ADC_GET_SAMPLE_10(val);
            if (val < min)min = val;
            if (val > max)max = val;
        }
        val = max - min;
        if (size == 2) {
            buffer[count] = val & 0xFF;
            count++;
            buffer[count] = (val >> 8) & 0xFF;
        }

        if (size == 1) {
            buffer[count] = (val >> 2) & 0xFF;
        }
    }
    else if(channel == HUMIDITY)
    {
	h_cnt=0;
	// Port F.2
	// Wait while pin is high
	do{
	}while((PINF & 0x4) == 0x4 );
	// Wait while pin is low
	do{
	}while((PINF & 0x4) == 0 );
	// Time how long pin stays high
	//_nrk_high_speed_timer_stop();
	//_nrk_high_speed_timer_reset();
	//_nrk_high_speed_timer_start();
	do{
	// Make h_cnt only 8 bit so it accumulates faster
	h_cnt++;
	}while((PINF & 0x4) == 0x4 );
//	h_cnt=_nrk_high_speed_timer_get();
	buffer[0]=h_cnt&0xff;
	buffer[1]=h_cnt >> 8;
	buffer[2]=h_cnt >> 16;
	buffer[3]=h_cnt >> 24;

    }
    // Here the special cases of the I2C sensors (pressure and temperature)
    // are dealt with
    else if(channel == TEMP2 || channel == PRESS){

        // Size must be 2 for these measurements. If the user has specified 1,
        // give them a zero to indicate something is wrong
        if(size != 4)
            return 0;

        if(channel == TEMP2)
	{
            value_from_sensor = calc_temp();
	}
	    else{
            calc_temp(); /* Note this is neccessary because the pressure calculations depend upon the
             raw temperature values*/
            value_from_sensor = calc_press(0); // OSS hard coded to 0 for now, doesn't really matter
        }
        // Now break up the value to fit in the buffer
        buffer[count] = value_from_sensor & 0xFF;
        count++;
        buffer[count] = (value_from_sensor >> 8 ) & 0xFF;
        count++;
        buffer[count] = (value_from_sensor >> 16 ) & 0xFF;
        count++;
        buffer[count] = (value_from_sensor >> 24 ) & 0xFF;

    }

    ADC_DISABLE();
    count++;
    return count;


}


int8_t close(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    // Turn off sensor power
    //PORTE |= PWR_CTRL_MASK;
    nrk_gpio_clr(PWR_CTRL_PIN);
    //PORTE &= ~(PWR_CTRL_MASK);
    	is_open = 0;
    close_i2c(); // Shut down the I2C
    ADC_DISABLE();
    return NRK_OK;
}

int8_t get_status(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    // key and value get passed as opt and size
    uint8_t key = opt;

    // use "key" here
    if (key == SENSOR_SELECT) return channel;
    return NRK_ERROR;
}

int8_t set_status(uint8_t action, uint8_t opt, uint8_t *buffer, uint8_t size) {

    // Note that here nothing needs to be done for the I2C. This is the original
    // code for the firefly 2.2 board driver

    // key and value get passed as opt and size
    uint8_t key = opt;
    uint8_t value = size;

    // use "key" and "value" here
    if (key == SENSOR_SELECT) {
        // Set to audio channel if it is an average value
        if (value == AUDIO_P2P) {
            channel = value;
            ADC_VREF_VCC();
            ADC_SET_CHANNEL(AUDIO);
            nrk_spin_wait_us(ADC_SETUP_DELAY);
            return NRK_OK;

        } else {
            if (value > 13) { // Update for all of the sensors
                _nrk_errno_set(1);
                return NRK_ERROR;
            }
            channel = value;
            ADC_VREF_VCC();
            ADC_SET_CHANNEL(channel);
            nrk_spin_wait_us(ADC_SETUP_DELAY);
            return NRK_OK;
        }
    }
    return NRK_ERROR;
}

void get_eeprom_values(){

   uint8_t raw_eeprom_data[22]; // Buffer for storing the raw values being returned
   int j = 0;
   int rv;

   // Get the values
  rv = ee24xx_read_bytes(0xAA, 22, raw_eeprom_data);

  if(DEBUG){

   //   printf("The raw values from the EEPROM are: \r\n");

    //  for(j = 0; j<22; j++)
      //  printf("%02x ", raw_eeprom_data[j]);
     // printf("\r\n");
  }

  // Place the data in a formatted buffer
  for(j = 0; j < 11; j++){
      eeprom_values[j] = ((raw_eeprom_data[2*j] | 0x0000) << 8) |
(raw_eeprom_data[2*j + 1]);
  }

 // Print out the values of the EEPROM
/*  if(DEBUG){
      printf("AC1: %i\r\n", AC1);
      printf("AC2: %i\r\n", AC2);
      printf("AC3: %i\r\n", AC3);
      printf("AC4: %u\r\n", AC4);
      printf("AC5: %u\r\n", AC5);
      printf("AC6: %u\r\n", AC6);
      printf("B1: %i\r\n", B1);
      printf("B2: %i\r\n", B2);
      printf("MB: %i\r\n", MB);
      printf("MC: %i\r\n", MC);
      printf("MD: %i\r\n", MD);
      printf("-------------------------------------------------\r\n");
  }
  */
}


// read_voltage_status()
//
// This function sets different voltage threshold levels on
// the cc2420 chip to search for the voltage.
// If the voltage is above 3.3 volts, then the ADC reads
// the external voltage value going through a voltage divider.
// This function will return VOLTS*100
/*
uint16_t read_voltage_status() {
    volatile uint16_t val;
    uint8_t check, level;
    nrk_sem_t *radio_sem;

    radio_sem = rf_get_sem();

    // if semaphore not created, then assume you own the radio
    if (radio_sem != NULL)
        nrk_sem_pend(radio_sem);

    // activate cc2420 vreg
     //SET_VREG_ACTIVE();
    // FIXME: Check at end if VREG needs to be disabled again...

    level = 0;
    while (level < 0x1F) {
        val = 0x0020 | level;
        //FASTSPI_SETREG(CC2420_BATTMON, val);
        nrk_spin_wait_us(2);
        //FASTSPI_GETREG(CC2420_BATTMON, val);
        if (val & 0x0040) break;
        level++;
    }
    if (radio_sem != NULL)
        nrk_sem_post(radio_sem);
    if (level == 0) {
        val = get_adc_val();
        // FIXME:  This probably isn't correct...
        if (val > 174) val -= 174;
        if (val < 330) val = 330;
    } else val = (9000 - (level * 125)) / 27;

    return val;
}
*/
void init_adc() {
    // Initialize values here
    ADC_INIT();
    ADC_ENABLE();
    channel = 0;
    ADC_SET_CHANNEL(0);
    ADC_DISABLE();
}

uint16_t get_adc_val() {
    uint16_t adc_val;
    ADC_SAMPLE_SINGLE();
    delay();
    ADC_GET_SAMPLE_10(adc_val);
    return adc_val;
}

void delay() {
    nrk_spin_wait_us(ADC_SETUP_DELAY);
}

int32_t calc_true_press(uint8_t oss){

    B6 = B5 - 4000;                                     if(DEBUG) printf("B6 %li\r\n", B6);
//    X1 = (B2 * (B6 * B6/ pow(2,12)))/ pow(2,11);        if(DEBUG) printf("X1 %li\r\n", X1);
    X1 = (B2 * (B6 * B6/ 4096))/ 2048;        if(DEBUG) printf("X1 %li\r\n", X1);
    X2 = (int32_t)AC2 * B6 / 2048;                          if(DEBUG) printf("X2 %li\r\n", X2);
//    X2 = AC2 * B6 / pow(2,11);                          if(DEBUG) printf("X2 %li\r\n", X2);
    X3 = X1 + X2;                                       if(DEBUG) printf("X3 %li\r\n", X3);
    B3 = ((((int32_t)AC1 * 4 + X3) << oss) + 2) / 4;               if(DEBUG) printf("B3 %li\r\n", B3);
//    X1 = AC3 * B6 / pow(2,13);                          if(DEBUG) printf("X1 %li\r\n", X1);
    X1 = AC3 * B6 / 8192;                          if(DEBUG) printf("X1 %li\r\n", X1);
//    X2 = (B1 * (B6 * B6 / pow(2,12))) / pow(2,16);      if(DEBUG) printf("X2 %li\r\n", X2);
    X2 = (B1 * (B6 * B6 / 4096)) / 65536;      if(DEBUG) printf("X2 %li\r\n", X2);
//    X3 = ((X1 + X2) + 2) / pow(2,2);                    if(DEBUG) printf("X3 %li\r\n", X3);
    X3 = ((X1 + X2) + 2) / 4;                    if(DEBUG) printf("X3 %li\r\n", X3);
//    B4 = AC4 * (X3 + 32768) / pow(2,15);                if(DEBUG) printf("B4 %lu\r\n", B4);
    B4 = AC4 * (X3 + 32768) / 32768;                if(DEBUG) printf("B4 %lu\r\n", B4);
    B7 = (UP - B3) * (50000 >> oss);                    if(DEBUG) printf("B7 %lu\r\n", B7);

    if(B7 < 0x80000000)
       P = (B7 * 2)/ B4;
    else
        P = (B7 / B4) * 2;
                                                        if(DEBUG) printf("P %li\r\n", P);
    X1 = (P / 256) * (P / 256);               if(DEBUG) printf("X1 %li\r\n", X1);
//    X1 = (P / pow(2,8)) * (P / pow(2,8));               if(DEBUG) printf("X1 %li\r\n", X1);
    X1 = (X1 * 3038) / 65536;                       if(DEBUG) printf("X1 %li\r\n", X1);
//    X1 = (X1 * 3038) / pow(2,16);                       if(DEBUG) printf("X1 %li\r\n", X1);
    X2 = (-7357 * P ) / 65536;                      if(DEBUG) printf("X2 %li\r\n", X2);
//	X2 = (-7357 * P ) / pow(2,16);                      if(DEBUG) printf("X2 %li\r\n", X2);
//    P = P + (X1 + X2 + 3791) / pow(2,4);                if(DEBUG) printf("P %li\r\n", P);
    P = P + (X1 + X2 + 3791) / 16;                if(DEBUG) printf("P %li\r\n", P);

    if(DEBUG) printf("Pressure is %li Pascals\r\n", P);

    return P;

}


inline int32_t calc_true_temp(){

    X1 = ((int32_t)UT - (int32_t)AC6) * (int32_t)AC5 / (int32_t)32768;
//    X1 = ((int32_t)UT - (int32_t)AC6) * (int32_t)AC5 / (int32_t)pow(2,15);
if(DEBUG) printf("X1 %li\r\n", X1);
//    X2 = (int32_t) MC * pow(2,11) / (X1 + MD);
    X2 = (int32_t) MC * 2048 / (X1 + MD);
if(DEBUG) printf("X2 %li\r\n", X2);
    B5 = X1 + X2;
if(DEBUG) printf("B5 %li\r\n", B5);
//    T = (B5 + 8) / pow(2,4);
    T = (B5 + 8) / 16;
if(DEBUG) printf("XT %li\r\n", T);

    if(DEBUG) printf("Temperature is %li /10 degrees Celcius.\r\n", T);

    return T;
}

void read_uncomp_temp(){

    uint8_t write_buf[1];
    uint8_t rx_buf[2];
    uint8_t i;

    write_buf[0] = 0x2E;

    // Step 1: Write 0x2E into register 0xF4
    ee24xx_write_bytes(0xF4, 1, write_buf);

    // Step 2: Wait 4.5 ms
 //   nrk_wait_ticks(1000); // 1 tick is 1 ms I believe
    nrk_spin_wait_us(4500);

    // Step 3: Read registers 0xF6, 0xF7
    ee24xx_read_bytes(0xF6, 2, rx_buf);

//    if(DEBUG){
  //      for(i = 0; i < 2; i++)
  //          printf("uncomp_temp[%i] is %u\r\n", i, rx_buf[i]);
  //  }

    UT = (((uint16_t)rx_buf[0] | 0x0000) << 8) | ((uint16_t)rx_buf[1]);

//    if(DEBUG)
  //   printf("UT is %i\r\n", UT);

}


int32_t calc_temp(){
    read_uncomp_temp();
    return calc_true_temp();
}


int32_t calc_press(uint8_t oss){
    read_uncomp_press(oss);
    return calc_true_press(oss);
}

void read_uncomp_press(uint8_t oss){

    uint8_t write_buf[1]; // Used to send a value to the Bosch sensor
    uint8_t rx_buf[3]; // Used to store the received values from the sensor
    uint8_t i;

    write_buf[0] = 0x34 + (oss << 6);

    // Step 1: Write 0x2E into register 0xF4
    ee24xx_write_bytes(0xF4, sizeof(write_buf), write_buf);

    // Step 2: Wait 4.5 ms
//    nrk_wait_ticks(1000); // 1 tick is 1 ms I believe
    nrk_spin_wait_us(4500);

    // Step 3: Read registers 0xF6, 0xF7, 0xF8
    ee24xx_read_bytes(PRESS_BASE_REGISTER, sizeof(rx_buf), rx_buf);

   /* if(DEBUG){
        for(i = 0; i < 3; i++)
            printf("uncomp_press[%i] is %u\r\n", i, rx_buf[i]);
    }
*/
     // Assemble the raw pressure value

     UP = ((uint32_t) rx_buf[0]) << 16;
     UP = UP  | ((uint32_t)(rx_buf[1]) << 8);
     UP = UP  + rx_buf[2];
     UP = UP >> (8-oss);

 //    if(DEBUG)
 //    printf("UP is %li\r\n", UP);

}
