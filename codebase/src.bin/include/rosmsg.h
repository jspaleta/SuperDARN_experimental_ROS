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
#define REMOVE_SEQ '-'
#define REGISTER_RADAR_CHAN 'a'
#define GET_PARAMETERS 'b'
#define SET_PARAMETERS 'c'
#define CtrlProg_READY 'd'
#define CtrlProg_END 'e'
#define SET_INACTIVE 'f'
#define SET_ACTIVE 'g'
#define GET_DATA 'h'
#define AUX_COMMAND 'i'
#define WAIT 'j'
#define PRETRIGGER 'k'
#define EXTERNAL_TRIGGER 'l'
#define GET_TRTIMES 'm'
#define TRIGGER 'n'
#define POSTTRIGGER 'o'
#define GET_DATA_STATUS 'q'
#define GET_TRIGGER_OFFSET 'r'
#define SET_TRIGGER_OFFSET 's'
#define PRE_CLRSEARCH 't'
#define POST_CLRSEARCH 'u'
#define CLRSEARCH_READY 'v'
#define CLRSEARCH 'w'
#define GET_EVENT_TIME 'x' 
#define SITE_SETTINGS 'y'
#define REQUEST_ASSIGNED_FREQ '>'
#define REQUEST_CLEAR_FREQ_SEARCH '<'
#define QUERY_INI_SETTING 'z'
#define PING '='
#define OKAY '^'
#define NOOP '~'
#define QUIT '.'
//#define GET_SITE_SETTINGS 's'
//#define GPS_SET_TRIGGER_RATE 'b'
//#define UPDATE_SITE_SETTINGS 'S'
//#define SET_SITE_IFMODE 'I'







#endif
