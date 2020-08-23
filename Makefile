# ========================================================================================
# Makefile for airspy_fft_ws
# ========================================================================================

# ========================================================================================
# Compile flags

ifneq (, $(shell which gcc-8))
  CC = gcc-8
else
  CC = gcc
endif

COPT = -O3 -march=native -mtune=native
CFLAGS = -Wall -Wextra -Wpedantic -std=gnu11 -D_GNU_SOURCE
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

LIBSDIR = libwebsockets/build/include
OBSDIR = libwebsockets/build/lib

LIBS = -lm -pthread `pkg-config --libs libairspy` -lusb-1.0 -lfftw3 -Wl,-Bstatic -lwebsockets -Wl,-Bdynamic

CFLAGS += `pkg-config --cflags libairspy`

# ========================================================================================
# Makerules

all:
	@pkg-config --modversion "libairspy = 1.0"
	$(CC) $(COPT) $(CFLAGS) $(SRC) -o $(BIN) -I $(LIBSDIR) -L $(OBSDIR) $(LIBS)

debug: COPT = -Og -ggdb -fno-omit-frame-pointer -D__DEBUG
debug: all

clean:
	rm -fv $(BIN)
