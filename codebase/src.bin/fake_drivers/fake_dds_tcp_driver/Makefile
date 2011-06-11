OS:=$(shell uname -s)

CC=gcc
CFLAGS=-c -DPOSIX
common_libs=-lm 
linux_libs=
qnx_libs=-lsocket
INCLUDES=-I"../../include/" -I"include/" -I"../../../src.utils/tsg/include" -I"../../../src.utils/include"  
SOURCES=main.c  ics660_set_parameter.c ics660_init.c pdebug.c dc60m_init.c load_frequency.c \
        load_filter_taps.c load_phase.c  _decodestate.c \
        one_shot_b.c ../../../src.utils/_tcpsocket.c ../../../src.utils/beam_phase.c ../../../src.utils/utils.c timing_sequence.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=fake_dds_tcp_driver

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
ifeq ($(OS),Linux)
	$(CC) -o $@ $(OBJECTS) $(common_libs) $(linux_libs) 
endif
ifeq ($(OS),QNX)
	$(CC) -o $@ $(OBJECTS) $(common_libs) $(qnx_libs) 
endif
	cp $(EXECUTABLE) ../../../../bin/
.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(INCLUDES)

clean:
	rm -rf $(OBJECTS) $(EXECUTABLE)

