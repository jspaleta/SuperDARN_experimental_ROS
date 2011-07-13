#include <pthread.h>
#include <sys/time.h>
#include "rtypes.h"
#include "rosmsg.h"
#include "control_program.h"
#include "site.h" 
#include "iniparser.h"

#ifndef _SITE_DEFAULTS_H
#define _SITE_DEFAULTS_H


#define SITE_NAME "tst"

#define SITE_DIR "/tmp/site_data/"

#define FULL_CLR_FREQ_START	8000  // in KHz	
#define FULL_CLR_FREQ_END	20000 // in KHz

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
#define MAX_SEQ_LENGTH 1048576
#define MAX_PULSES 100

#define CLIENT 0
#define VIEWER 1
#define WORKER 2
#define RECV_SAMPLE_HEADER 2 
#define RECV_CLRFREQ_SAMPLES  2000
#define DEFAULT_FREQ 13000

#endif
