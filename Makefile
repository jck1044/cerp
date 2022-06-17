## Use all the fancy Makefile setup in libopencm3-examples

BINARY = cerpa

OBJS += samp.o uart.o HK.o

LDSCRIPT = libopencm3-examples/examples/stm32/f0/stm32f0-discovery/stm32f0-discovery.ld

LIBNAME		= opencm3_stm32f0
DEFS		+= -DSTM32F0

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m0 $(FP_FLAGS)


################################################################################
# OpenOCD specific variables

OOCD		?= openocd
OOCD_INTERFACE	?= stlink-v2-1
OOCD_TARGET	?= stm32f0x

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242

OPENCM3_DIR = libopencm3

include libopencm3-examples/examples/rules.mk

