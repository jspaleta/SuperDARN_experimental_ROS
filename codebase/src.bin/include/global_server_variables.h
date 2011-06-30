#include <pthread.h>
#include <sys/time.h>
#include "tsg.h"
#include "rtypes.h"
#include "rosmsg.h"
#include "control_program.h"
#include "site.h" 
#include "iniparser.h"

#ifndef _GLOBAL_SERVER_H
#define _GLOBAL_SERVER_H


#define SITE_NAME "tst"

#define SITE_DIR "/tmp/site_data/"

#define IF_ENABLED         1 
#define IF_DISABLED         0 

//#define IF_FREQ            71000 // in KHz
#define FULL_CLR_FREQ_START	8000  // in KHz	
#define FULL_CLR_FREQ_END	20000 // in KHz
//#define MAX_CLR_WAIT	0     // in secs	

#define RECV_COMPLEX_SAMPLE_SIZE 32  //IQ together 
#define RECV_REAL_OFFSET 0
#define RECV_IMAG_OFFSET 1

#define Max_Control_THREADS     30

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
     struct TSGbuf *pulseseqs[MAX_SEQS]; //array of pulseseq pointers
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


//#define TIME_INTERVAL	100000000
#define DEFAULT_FREQ 13000

//#define SIDEBAND 100
//struct FreqTable {
//  int32 num;
//  int32 dfrq;
//  int32 *start;
//  int32 *end;
//};

/*
struct tx_status {
  int32 LOWPWR[MAX_TRANSMITTERS];
  int32 AGC[MAX_TRANSMITTERS];
  int32 status[MAX_TRANSMITTERS];
};
*/
/*
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
*/
#endif
