OS:=$(shell uname -s)

CC=gcc
CFLAGS=-c -Wall
LFLAGS  = -L"../../../src.lib/iniparser3.0b"
common_libs=-lrt -lm -liniparser
linux_libs=
qnx_libs=-lsocket
INCLUDES=-I"../../include/" -I"include/" -I"../../../src.lib/iniparser3.0b/src/"  
SOURCES=main.c _configure_driver.c pretrigger_driver.c unpack_sequence.c _process_aux_commands.c _decodestate.c _ini_file_functions.c collect_data.c \
../../../src.utils/_tcpsocket.c \
../../../src.utils/utils.c \
../../../src.utils/aux_command.c 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=fake_general_tcp_driver

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(OBJECTS) $(LFLAGS) $(common_libs) $(qnx_libs) 
endif
#	cp $(EXECUTABLE) ../../../../bin/
.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

clean:
	rm -rf $(OBJECTS) $(EXECUTABLE) 

