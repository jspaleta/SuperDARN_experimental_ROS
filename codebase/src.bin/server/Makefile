OS:=$(shell uname -s)

CC=gcc
CFLAGS=-c 
LFLAGS  = -L"../../src.lib/iniparser3.0b"
common_libs=-lm -liniparser
linux_libs=-lm -lpthread -lrt
qnx_libs=-lsocket
INCLUDES=-I"../include/" -I"include/" -I"../../src.utils/tsg/include" -I"../../src.lib/iniparser3.0b/src/"
SOURCES=main.c _process_aux_commands.c client_handler.c timeout_handler.c status_handler.c \
settings_handler.c dio_handler.c timing_handler.c coordination_handler.c \
dummy_handler.c reciever_handler.c dds_handler.c \
../../src.utils/utils.c ../../src.utils/tsg/maketsg.c \
../../src.utils/_tcpsocket.c ../../src.utils/aux_command.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=arby_server
INI_TEST=ini_test
INI_SOURCES=ini_test.c 
INI_OBJECTS=$(INI_SOURCES:.c=.o)

all: $(SOURCES) $(EXECUTABLE)

ini: $(INI_SOURCES) $(INI_TEST)

$(EXECUTABLE): $(OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(OBJECTS) $(LFLAGS) $(common_libs) $(qnx_libs) 
endif
	cp $(EXECUTABLE) ../../../bin/
$(INI_TEST): $(INI_OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(INI_OBJECTS) $(LFAGS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(INI_OBJECTS) $(LFLAGS) $(common_libs) $(qnx_libs) 
endif

.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(LFAGS) $(INCLUDES)

clean:
	rm -rf $(OBJECTS) $(EXECUTABLE) $(INI_OBJECTS) $(INI_TEST) 



