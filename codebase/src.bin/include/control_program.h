#include <pthread.h>
#include <sys/time.h>
#include "rtypes.h"
#include "rosmsg.h"
#include "site.h"

#ifndef _CONTROL_PROGRAM_H
#define _CONTROL_PROGRAM_H

#define ROS_IP "127.0.0.1"
#define ROS_PORT 45000


struct TRTimes {
  int32 length;
  uint32 *start_usec;  /* unsigned int32 pointer */
  uint32 *duration_usec; /* unsigned int32 pointer */
};

struct SeqPRM {
  uint32 len;
  uint32 step_usec;  //packed timesequence stepsize in microseconds
  uint32 mpinc_usec;
  uint32 mppul;
};

struct tx_status {
  int32 LOWPWR[MAX_TRANSMITTERS];
  int32 AGC[MAX_TRANSMITTERS];
  int32 status[MAX_TRANSMITTERS];
};


struct DataPRM {
  uint32 event_secs;
  uint32 event_nsecs;
  int32 bufnum;
  int32 samples;
  int32 use_shared_memory;
  int32 shared_memory_offset;
  int32 status;
};

struct RadarPRM {
     int32 site;
     int32 radar; //required: DO NOT SET MANUALLY
     int32 channel; //required: DO NOT SET MANUALLY
};

struct RXFESettings {
     uint32 ifmode;  // IF Enabled
     uint32 amp1;    // Stage 1 Amp 20 db before IF mixer
     uint32 amp2;    // Stage 2 Amp 10 db after IF mixer
     uint32 amp3;    // Stage 3 Amp 10 db after IF mixer
     uint32 att1;    // 1/2 db Attenuator
     uint32 att2;    //  1  db Attenuator
     uint32 att3;    //  2  db Attenuator 
     uint32 att4;    //  4  db Attenuator
};
/*
struct ChannelStatus {
     int32 active;
};
*/

struct SiteSettings {
     char name[80];
     uint32 num_radars;
     uint32 ifmode;
     struct RXFESettings rf_settings;  /* reciever front end settings for this site */
     struct RXFESettings if_settings;  /* reciever front end settings for this site */
};


struct ControlPRM {
     int32 radar; //required: DO NOT SET MANUALLY
     int32 channel; //required: DO NOT SET MANUALLY
     int32 local;  //if local use shared memory for data handling else use tcp
     int32 priority; //optional: valid 0-99: lower value higher priority when running multiple operational programs
     int32 pulseseq_index[3]; //required: registered pulse sequence to use 
// transmit beam definition one of these needs to be non-zero
     int32 tbeam;  //required: valid 0-31: defines standard look directions 
     uint32 tbeamcode; //optional: used for special beam directions, used only if beam is invalid value.
// Imaging transmit beam options
     float tbeamazm; //optional: used for imaging radar
     float tbeamwidth; //optional: used for imaging radar
//transmit setup
     int32 tfreq;  //required: transmit freq in kHz 
     int32 trise;  // required: rise time in microseconds
//reciever setup
     int32 number_of_samples;  //required: number of recv samples to collect 
     int32 buffer_index; //required: valid 0-1: DMA buffer to use for recv
     float baseband_samplerate; //required: normally equals (nbaud/txpl) but can be changed for oversampling needs
     int32 filter_bandwidth; //required: normally equals basebad_samplerate but can be changed for oversampling needs
     int32 match_filter;  // required: valid 0-1: whether to use match filter, normally equal 1 
     int32 rfreq;  //optional: if invalid value tfreq is used
// reciever beam definitions: only used if tbeam is invalid
     int32 rbeam;  //optional: valid 0-31: defines standard look directions: if invalid tbeam is used 
     uint32 rbeamcode; //optional: used for special beam directions, used only if rbeam and tbeam is invalid value.
// Imaging receiver beam options
     float rbeamazm; //optional: used for imaging radar
     float rbeamwidth; //optional: used for imaging radar
// ROS Feedback
     int32 status; // coded value: non-zero values will code to an error msg
     char name[80]; //optional: but a very good idea to set
     char description[120]; //optional: but a very good idea to set
};


struct CLRFreqPRM {
     int32 freq_start_khz; //In kHz
     int32 freq_end_khz;  //in kHz
     int32 rbeam;  //beam number
     int32 nave;  // Number of passes to average.
     float rx_bandwidth_khz;  //in kHz  typically c/(rsep*2)
     float tx_bandwidth_khz;  //in kHz  typically c/(rsep*2)
};

#endif
