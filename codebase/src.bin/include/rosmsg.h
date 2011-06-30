#ifndef _ROSMSG_H

#define _ROSMSG_H
#include "rtypes.h"
#define MAX_MSG_STRING 80

struct msgvar {
        char name[MAX_MSG_STRING];
        char type[MAX_MSG_STRING];
        uint32 offset;
        uint32 bytes;
};

struct ROSMsg {
     char command_type;
     char command_name[MAX_MSG_STRING];
     char driver[MAX_MSG_STRING];
     int32 num_vars;
     int32 bytes;
     int32 status; // 0 = inactive  negative = error  postive = good
     int32 is_reply;
     uint64 buffer;
     uint64 vars;
};


#define REGISTER_SEQ '+'
#define CtrlProg_READY '1'
#define CtrlProg_END '@'
#define WAIT 'W'
#define PRETRIGGER '3'
#define EXTERNAL_TRIGGER '6'
#define GET_TRTIMES '7'
#define TRIGGER '4'
#define POSTTRIGGER '5'
#define GET_DATA 'd'
#define GET_DATA_STATUS 'D'
#define GET_TRIGGER_OFFSET 't'
#define SET_TRIGGER_OFFSET 'T'
#define PRE_CLRSEARCH 'c'
#define POST_CLRSEARCH 'C'
#define CLRSEARCH_READY 'a'
#define CLRSEARCH 'b'
//#define FULL_CLRFREQ '-'
#define GET_EVENT_TIME 'E' 
#define SITE_SETTINGS 'R'
#define GPS_SET_TRIGGER_RATE 'b'


#endif
