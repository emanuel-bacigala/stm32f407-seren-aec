CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

CFLAGS = -O2 -Wall -std=gnu99 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fdata-sections -ffunction-sections
CFLAGS += -DHAVE_CONFIG_H -Ilibspeexdsp -I.

DIR=libspeexdsp
SRCS =	$(DIR)/buffer.c \
		$(DIR)/fftwrap.c \
		$(DIR)/filterbank.c \
		$(DIR)/jitter.c \
		$(DIR)/mdf.c \
		$(DIR)/resample.c \
		$(DIR)/smallft.c \
		$(DIR)/scal.c \
		$(DIR)/kiss_fft.c \
		$(DIR)/kiss_fftr.c \
		$(DIR)/preprocess.c

OBJS = $(SRCS:.c=.o)

.PHONY: libspeexdsp.a

all: libspeexdsp.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libspeexdsp.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS)
