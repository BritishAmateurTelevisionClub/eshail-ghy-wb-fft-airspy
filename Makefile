# ========================================================================================
# Makefile for airspy_fft_ws
# ========================================================================================

# ========================================================================================
# Compile flags

CC = gcc
COPT = -O3 -march=native
CFLAGS = -Wall -Wextra -std=gnu11 -D_GNU_SOURCE
CFLAGS += -D BUILD_VERSION="\"$(shell git describe --dirty --always)\""	\
		-D BUILD_DATE="\"$(shell date '+%Y-%m-%d_%H:%M:%S')\""

BIN = airspy_fft_ws

# ========================================================================================
# Source files

SRCDIR = .

SRC = 	$(SRCDIR)/libairspy/libairspy/src/*.c \
		$(SRCDIR)/main.c

# ========================================================================================
# External Libraries

LIBSDIR = 

LIBS = -lm -pthread -lsamplerate `pkg-config --libs libairspy` -lusb-1.0 -lfftw3 -lwebsockets

CFLAGS += `pkg-config --cflags libairspy`

# ========================================================================================
# Makerules

all:
	$(CC) $(COPT) $(CFLAGS) $(SRC) -o $(BIN) $(LIBSDIR) $(LIBS)

debug: COPT = -Og -ggdb -fno-omit-frame-pointer -D__DEBUG
debug: all

clean:
	rm -fv $(BIN)