OS:=$(shell uname -s)
CC=gcc
CFLAGS=-c
common_libs=-lm
linux_libs=-lrt
qnx_libs=-lsocket
INCLUDES=-I"../../include/" -I"./include/" -I"../../../src.utils/tsg/include"
SOURCES=arby_client.c  ../../../src.utils/utils.c  ../../../src.utils/tsg/maketsg.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=arby_client

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(OBJECTS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(OBJECTS) $(common_libs) $(qnx_libs) 
endif

.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(INCLUDES)

clean:
	rm -rf $(OBJECTS) arby_client 



