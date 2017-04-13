# Platform name  cc2420DK, firefly, micaZ, firefly2_1, firefly2_2, firefly2_3, stk600_128rfa1
PLATFORM = firefly3_x2


# Target file name (without extension).
TARGET = main

# Set the Port that you programmer is connected to
# PROGRAMMING_PORT = /dev/ttyUSB0
# PROGRAMMING_PORT = /dev/tty.usbserial-AE00BUKC
# PROGRAMMING_PORT = /dev/tty.usbserial-A101XVZK

PROGRAMMING_PORT = `ls /dev/tty.* | grep usb | head -n 1` # programmer connected to serial device

# Set this such that the nano-RK directory is the base path
ROOT_DIR = ../..

# set alternate compiler version
CC = avr-gcc-4.8.3 -g

# Set platform specific defines
# The following will be defined based on the PLATFORM variable:
# PROG_TYPE  (e.g. avrdude, or uisp)
# MCU (e.g. atmega32, atmega128, atmega1281)
# RADIO (e.g. cc2420)
include $(ROOT_DIR)/include/platform.mk

ifndef CLIENT_MAC
  CLIENT_MAC = 1
  NODE_ADDR = 1
endif
ifdef CLIENT_MAC
	NODE_ADDR = CLIENT_MAC
endif


SRC = $(TARGET).c

# Add extra source files.
# For example:
SRC += $(ROOT_DIR)/src/net/pcf_tdma/$(RADIO)/pcf_tdma.c
SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM)/source/dwm1000.c
SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM)/source/dwm1000_spi.c
SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM)/source/dwm1000_sstwr.c
SRC += TWI_Master.c

CFLAGS += -D CLIENT_MAC=$(CLIENT_MAC)


# Add extra includes files.
# For example:
EXTRAINCDIRS =
EXTRAINCDIRS += $(ROOT_DIR)/src/net/pcf_tdma
EXTRAINCDIRS += $(ROOT_DIR)/safe_cyclist/include



#  This is where the final compile and download happens
include $(ROOT_DIR)/include/platform/$(PLATFORM)/common.mk
