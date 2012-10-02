include /Users/nraynaud/dev/STM32F4.platform/Developer/Share/Makefile
SRCS += *.c
SRCS += Source/*.c
SRCS += Source/portable/GCC/ARM_CM4F/port.c
#SRCS += Source/portable/MemMang/heap_1.c
OBJS = $(SRCS:.c=.o)

CFLAGS += -std=c99 -nostdlib
CFLAGS += -include stm32f4xx_conf.h
CFLAGS += -ISource/include -ISource/portable/GCC/ARM_CM4F -I.

main.elf:
	$(CC) $(CFLAGS) $(SRCS) -o $@ -L$(SDK)/usr/lib -lstm32f4 -lCMSIS_DSP
	
all: deploy