
# We use the stripped version to avoid typos 
PLATFORM_TYPE = $(strip $(PLATFORM))

ifeq ($(PLATFORM_TYPE),firefly)
# FireFly Architecture specific includes should go here...
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega32
RADIO = cc2420
PLATFORM_FOUND= true 
endif

ifeq ($(PLATFORM_TYPE),firefly2_1)
# micaZ Architecture specific includes should go here...
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega128
RADIO = cc2420
PLATFORM_FOUND = true 
endif

ifeq ($(PLATFORM_TYPE),firefly2_2)
# micaZ Architecture specific includes should go here...
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega1281
RADIO = cc2420
PLATFORM_FOUND = true 
endif

ifeq ($(PLATFORM_TYPE),firefly2_3)
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega1281
RADIO = cc2420
PLATFORM_FOUND = true 
endif

ifeq ($(PLATFORM_TYPE),stk600_128rfa1)
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega128rfa1
RADIO = rf231_soc
PLATFORM_FOUND = true
endif


ifeq ($(PLATFORM_TYPE),firefly3)
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega128rfa1
RADIO = rf231_soc
PLATFORM_FOUND = true
endif
ifeq ($(PLATFORM_TYPE),firefly3_usb)
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega128rfa1
RADIO = rf231_soc
PLATFORM_FOUND = true
endif

ifeq ($(PLATFORM_TYPE),firefly3_alps)
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega256rfr2
RADIO = rfr2_soc
PLATFORM_FOUND = true
endif

ifeq ($(PLATFORM_TYPE),firefly3_x2)
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega128rfa1
RADIO = rf231_soc
PLATFORM_FOUND = true
endif


ifeq ($(PLATFORM_TYPE),MARS)
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega128rfa1
RADIO = rf231_soc
PLATFORM_FOUND = true
endif



ifeq ($(PLATFORM_TYPE),zigduino)
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = avrdude
MCU = atmega128rfa1
RADIO = rf231_soc
PLATFORM_FOUND = true
endif



ifeq ($(PLATFORM_TYPE),micaZ)
# micaZ Architecture specific includes should go here...
# Notice that this sets architecture path, the mcu type and the radio
#PROG_TYPE = uisp
PROG_TYPE = avrdude
MCU = atmega128
RADIO = cc2420
PLATFORM_FOUND = true 
endif

ifeq ($(PLATFORM_TYPE),micaZsim)
PLATFORM_FOUND = true
RADIO = cc2420sim
#
# THIS IS THE SIMULAVR TARGET
# 
endif

ifeq ($(PLATFORM_TYPE),cc2420DK)
# micaZ Architecture specific includes should go here...
# Notice that this sets architecture path, the mcu type and the radio
MCU = atmega128
RADIO = cc2420
PLATFORM_FOUND = true 
endif

ifeq ($(PLATFORM_TYPE),tmote)
# tmote Architecture specific includes should go here...
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = msp430-bsl
MCU = msp430x1611
RADIO = cc2420
PLATFORM_FOUND = true 
endif

ifeq ($(PLATFORM_TYPE),imec)
# tmote Architecture specific includes should go here...
# Notice that this sets architecture path, the mcu type and the radio
PROG_TYPE = msp430-bsl
MCU = msp430x149
RADIO = cc2420
PLATFORM_FOUND = true 
endif

