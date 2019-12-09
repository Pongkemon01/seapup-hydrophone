## Copyright Cypress Semiconductor Corporation, 2010-2018,
## All Rights Reserved
## UNPUBLISHED, LICENSED SOFTWARE.
##
## CONFIDENTIAL AND PROPRIETARY INFORMATION
## WHICH IS THE PROPERTY OF CYPRESS.
##
## Use of this file is governed
## by the license agreement included in the file
##
##      <install>/license/license.txt
##
## where <install> is the Cypress software
## installation root directory path.
##

MODULE = zeabus_hp_fx3s
OBJS = $(MODULE).o zeabus_flash.o zeabus_eeprom.o zeabus_gpio.o zeabus_usb.o zeabus_error.o zeabus_app_init.o
USER_CFLAGS = -Os
ARMGCC_VERSION=9.2.1

# Tools
CC  = arm-none-eabi-gcc
AS  = arm-none-eabi-gcc
LD  = arm-none-eabi-ld
AR  = arm-none-eabi-ar
ELF2IMG = elf2img

# Add any user defined ASMFLAGS
ASMFLAGS = $(USER_ASMFLAGS)

# The common include path
Include =-I. -I./include

# The common compiler options
CCFLAGS = -DTX_ENABLE_EVENT_TRACE -DDEBUG -DCYU3P_FX3=1 -D__CYU3P_TX__=1 $(Include)

# Add any user defined CFLAGS
CCFLAGS += $(USER_CFLAGS)

# The optimization level depends on the build configuration.

# The common libraries
# NOTE: This order is important for GNU linker. Do not change
LDLIBS = -L ./lib -lcyu3sport -lcyu3lpp -lcyfxapi -lcyu3threadx
LDLIBS  += \
	/usr/local/arm-none-eabi/lib/libc.a	\
	/usr/local/lib/gcc/arm-none-eabi/$(ARMGCC_VERSION)/libgcc.a

# System module
OBJS += cyfxtx.o cyfx_gcc_startup.o

# The common linker options
LDFLAGS = --entry CyU3PFirmwareEntry $(LDLIBS)

# Add any user provided LDFLAGS
LDFLAGS += $(USER_LDFLAGS)

# Arguments
ASMFLAGS += -Wall -c -mcpu=arm926ej-s -mthumb-interwork
CCFLAGS  += -Wall -mcpu=arm926ej-s -mthumb-interwork -ffunction-sections -fdata-sections

# Select the linker script based on the chosen FX3 device.
LDFLAGS += -T ./lib/zeabus.ld -d --gc-sections --no-wchar-size-warning -Map $(MODULE).map

%.o: %.c
	$(CC) $(CCFLAGS) -c -o $@ $< 

%.o: %.S
	$(AS) $(ASMFLAGS) -o $@ $<

%.elf: $(OBJS)
	$(LD) $+ $(LDFLAGS) -o $@

$(MODULE).img: $(MODULE).elf
	$(ELF2IMG) -o $@ -i $<

clean:
	rm -f *~ *.bak *.old
	rm -f *.rel *.rst *.lnk *.lst *.map *.asm *.sym *.mem *.tmp.c *.lk
	rm -f *.o *.elf *.map *.tmp.c *.img

all: $(MODULE).img
