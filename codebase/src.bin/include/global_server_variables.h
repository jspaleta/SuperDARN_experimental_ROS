#include <pthread.h>
#include <sys/time.h>
#include "tsg.h"
#include "rtypes.h"
#include "control_program.h"
#include "site.h" 
#include "iniparser.h"

#ifndef _GLOBAL_SERVER_H
#define _GLOBAL_SERVER_H


#define SITE_NAME "mcm"

#define SITE_DIR "/tmp/site_data/"

#define IF_ENABLED         1 
#define IF_DISABLED         0 
#define IF_FREQ            71000 // in KHz

#define FULL_CLR_FREQ_START	8000  // in KHz	
#define FULL_CLR_FREQ_END	20000 // in KHz
#define MAX_CLR_WAIT	0     // in secs	

#define RECV_COMPLEX_SAMPLE_SIZE 32  //IQ together 
#define RECV_REAL_OFFSET 0
#define RECV_IMAG_OFFSET 1

#define Max_Control_THREADS     30
#define Max_Viewer_THREADS     5
#define TIMING_HOST_PORT 45001
#define GPS_HOST_PORT 45004
#define DDS_HOST_PORT 45002
#define RECV_HOST_PORT 45006
#define DIO_HOST_PORT 45005
#ifdef __QNX__
  #define DIO_HOST_IP "127.0.0.1"
  #define TIMING_HOST_IP "127.0.0.1"
  #define DDS_HOST_IP "127.0.0.1"
  #define RECV_HOST_IP "127.0.0.1"
  #define GPS_HOST_IP "127.0.0.1"  
#else
  #define DIO_HOST_IP "127.0.0.1"
  #define TIMING_HOST_IP "127.0.0.1"
  #define DDS_HOST_IP "127.0.0.1"
  #define RECV_HOST_IP "127.0.0.1"
  #define GPS_HOST_IP "127.0.0.1"
#endif

#define MAX_SEQS 4
#define CLIENT 0
#define VIEWER 1
#define WORKER 2

#define RECV_SAMPLE_HEADER 2 
#define RECV_CLRFREQ_SAMPLES  2000

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
     int32 cancelled;
     int32 socket;
     char ready;
     char processing;
     int32 linked; //required: DO NOT SET MANUALLY
     int32 best_assigned_freq; 
     float best_assigned_noise; 
     int32 current_assigned_freq; 
     float current_assigned_noise; 
     int32 gpssecond;
     int32 gpsnsecond;
     double best_assigned_pwr; 
     double current_assigned_pwr; 
     int32 freq_change_needed; 
     int32 tx_sideband; //in kHz 
     int32 rx_sideband; //in kHz
     int32 N; 
     struct TSGbuf *pulseseqs[MAX_SEQS]; //array of pulseseq pointers
     struct ControlProgram *linked_program;
     struct timeval trigger_timeout;
//     struct timeval last_trigger_event;
     struct Thread_List_Item *thread;
     t_fft_index *fft_array;
};

struct ControlProgram {
// ros state variables
     struct ControlState *state;
     struct DataPRM *data;
     struct CLRFreqPRM clrfreqsearch; 
     struct ControlPRM *parameters;
     struct RadarPRM *radarinfo;
     uint32 *main;
     uint64 main_address;
     uint32 *back;
     uint64 back_address;
     int32 active;
};

struct ClrPwr {
// ros state variables
     double freq;
     double pwr;
};

struct BlackList {
// ros state variables
     int32 start;
     int32 end;
     uint64 program;
};
#define REGISTER_SEQ '+'
#define TIMING_REGISTER_SEQ '+'
#define DDS_REGISTER_SEQ '+'

#define CtrlProg_READY '1'
#define DIO_CtrlProg_READY '1'
#define DDS_CtrlProg_READY '1'
#define RECV_CtrlProg_READY '1'
#define TIMING_CtrlProg_READY '1'

#define CtrlProg_END '@'
#define DIO_CtrlProg_END '@'
#define DDS_CtrlProg_END '@'
#define RECV_CtrlProg_END '@'
#define TIMING_CtrlProg_END '@'

#define WAIT 'W'
#define TIMING_WAIT 'W'

#define PRETRIGGER '3'
#define DIO_PRETRIGGER '3'
#define DDS_PRETRIGGER '3'
#define RECV_PRETRIGGER '3'
#define TIMING_PRETRIGGER '3'

#define EXTERNAL_TRIGGER '6'
#define GET_TRTIMES '7'

#define TRIGGER '4'
#define DIO_TRIGGER '4'
#define TIMING_TRIGGER '4'
#define TIMING_GPS_TRIGGER 'G'

#define POSTTRIGGER '5'
#define RECV_POSTTRIGGER '5'
#define TIMING_POSTTRIGGER '5'

#define GET_DATA 'd'
#define GET_DATA_STATUS 'D'
#define RECV_GET_DATA 'd'

#define PRE_CLRFREQ 'c'
#define POST_CLRFREQ 'C'

#define RECV_CLRFREQ 'F'
#define FULL_CLRFREQ '-'


#define GET_EVENT_TIME 'E' 
#define SITE_SETTINGS 'R'

//#define GPS_GET_HDW_STATUS 240 
//#define GPS_GET_SOFT_TIME 241
//#define GPS_SCHEDULE_SINGLE_SCAN 244
//#define GPS_SCHEDULE_REPEAT_SCAN 245
//#define GPS_TRIGGER_NOW 246
#define GPS_SET_TRIGGER_RATE 'b'
//#define GPS_MSG_ERROR 248

#define REFRESHRATE	1
#define MAX_ERROR 0.002
#define TIME_INTERVAL	100000000



struct DriverMsg {
     char type;
     int32 status;

};

#define DEFAULT_FREQ 13000
#define SIDEBAND 100
struct FreqTable {
  int32 num;
  int32 dfrq;
  int32 *start;
  int32 *end;
};

/*
struct tx_status {
  int32 LOWPWR[MAX_TRANSMITTERS];
  int32 AGC[MAX_TRANSMITTERS];
  int32 status[MAX_TRANSMITTERS];
};
*/

struct GPSStatus {
	
	int32	hardware;
	int32	antenna;
	int32	lock;
	int32	gps_lock;
	int32	phase_lock;
	int32	reference_lock;
	int32	sv[6];
	float	signal[6];
	float	lat;
	float	lon;
	float	alt;
	float	mlat;	
	float	mlon;
	float	malt;
	int32	poscnt;
	int32	gpssecond;
	int32	gpsnsecond;
	int32	syssecond;
	int32	sysnsecond;
	int32	lastsetsec;
	int32	lastsetnsec;
	int32	nextcomparesec;
	int32	nextcomparensec;
	float	drift;
	float	mdrift;
	int32	tcpupdate;
	int32	tcpconnected;
	int32	timecompareupdate;
	int32	timecompareupdateerror;
	int32	lasttriggersecond;
	int32	lasttriggernsecond;
	int32	lasttcpmsg;
	int32	intervalmode;
	int32	scheduledintervalmode;
	int32	oneshot;
	float	settimecomparetime;
	int32	triggermode;
	int32	ratesynthrate;


};
#endif
