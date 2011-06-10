OS:=$(shell uname -s)

CC=gcc
CFLAGS=-c
LFLAGS  = -L"../../../src.lib/iniparser3.0b"
common_libs=-lm -liniparser
linux_libs=
qnx_libs=-lsocket
INCLUDES=-I"../../include/" -I"include/" -I"../../../src.utils/tsg/include" -I"../../../src.lib/iniparser3.0b/src/"  
SOURCES=main.c  _dma.c _open_PLX9080.c _create_DMA_buff.c _decodestate.c _open_ini_file.c ../../../src.utils/_tcpsocket.c ../../../src.utils/utils.c 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=fake_timing_tcp_driver

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(OBJECTS) $(LFLAGS) $(common_libs) $(qnx_libs) 
endif

.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

clean:
	rm -rf $(OBJECTS) $(EXECUTABLE) 

