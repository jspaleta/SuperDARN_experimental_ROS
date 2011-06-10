OS:=$(shell uname -s)

CC=gcc
CFLAGS=-Wall -c
common_libs=-lm
linux_libs=
qnx_libs=-lsocket
INCLUDES=-I"../../include/" -I"include/" -I"../../../src.utils/tsg/include" -I"../../server/include" 
SOURCES=main.c _select_beam.c _select_card.c _get_status.c _rxfe_commands.c ../../../src.utils/_tcpsocket.c ../../../src.utils/utils.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=fake_dio_tcp_driver

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(OBJECTS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(OBJECTS) $(common_libs) $(qnx_libs) 
endif
	cp $(EXECUTABLE) ../../../bin
.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(INCLUDES)

clean:
	rm -rf $(OBJECTS) $(EXECUTABLE) 



