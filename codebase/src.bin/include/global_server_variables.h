#include <pthread.h>
#include <sys/time.h>
#include "rtypes.h"
#include "rosmsg.h"
#include "control_program.h"
#include "site.h" 
#include "iniparser.h"

#ifndef _GLOBAL_SERVER_H
#define _GLOBAL_SERVER_H

#define IF_ENABLED         1 
#define IF_DISABLED         0 

typedef struct _fft_index{ 
// Struct to store and order the values of the fft preserving the index in the original array
	double pwr;
	double apwr;
	double freq;
	double detrend;
	int32 index;
} t_fft_index;

struct Thread_List_Item {
     struct Thread_List_Item *prev;
     struct Thread_List_Item *next;     
     pthread_t id;
     struct timeval timeout;
     struct timeval last_seen;
     char name[80];
     int32 type;
     void *data;
};



struct ControlState {
/* State Variables for internal ROS Usage */
     char ready;
     char processing;
     int32 cancelled;
     int32 socket;
     int32 active;
     int32 rx_trigger_offset_usec;
     int32 dds_trigger_offset_usec;
     int32 current_assigned_freq; 
     int32 gpssecond;
     int32 gpsnsecond;
     int32 tx_sideband; //in kHz 
     int32 rx_sideband; //in kHz
     int32 N; 
     int32 num_registered_seqs;
     int32 max_allowed_seqs;
     float current_assigned_noise; 
     double current_assigned_pwr; 
     struct timeval trigger_timeout;
     struct Thread_List_Item *thread;
     t_fft_index *fft_array;
};

struct ControlProgram {
     struct ControlState *state;
     struct DataPRM *data;
     struct CLRFreqPRM clrfreqsearch; 
     struct ControlPRM *parameters;
     struct RadarPRM *radarinfo;
     uint32 *main_shm;  // pointer to shared memory space holding the data samples
     uint32 *back_shm;  // pointer to shared memory space holding the data samples
     uint32 *main_addr;  // pointer to malloc'd memory space holding the data samples
     uint32 *back_addr;  // pointer to malloc'd memory space holding the data samples
};

struct ClrPwr {
     double freq;
     double pwr;
};

struct BlackList {
     int32 start;
     int32 end;
     uint64 program;
};

struct SeqBuf {
  struct SeqPRM prm; 
  unsigned char *code;
  unsigned char *rep;
  int *ptab;
};
#endif
