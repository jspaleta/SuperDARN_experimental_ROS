#ifndef _DDS_H 
#define _DDS_H
#define MAX_TSG 16
#define MAX_TIME_SEQ_LEN 1048576
#define X_bit 0x04
#define P_bit 0x10
#define IMAGING 0 

#define PULSE_LENGTH	   1000 //micro-seconds
#define IPP		   10000 //micro-seconds
#define SAMPLE_FREQ	   100000 //samples-per-second
#define FIFOLVL		   1024
#define STATE_DELAY        130.00 // in Microseconds
#define CH0_SAMPLES_TO_COLLECT 	10000
#define CH1_SAMPLES_TO_COLLECT  10000
#define DDS_MAX_CHANNELS 4
#define DDS_MAX_CARDS 1 
#define DDS_MASTER_INDEX 0 
#define DDS_TERM_INDEX 0 

// Below set the maximum DDS output for each radar to ensure correct maximum driving signal into the transmitters with
//  a single active radar channel

// Initial Fort Hays no adjustment { East 2.5 Vp-p, West 2.0 Vp-p}
//#define DDS_MAX_RADAR_OUTPUT { 32767, 32767 }

// 2010-11-03 Fort Hays { East, West } adjusted  for 1.5 Vp-p at Transmitters
//#define DDS_MAX_RADAR_OUTPUT { 19660,24575  }
// 2010-11-05 Fort Hays { East, West 1.46 Vpp } adjusted  at Transmitters inputs
//#define DDS_MAX_RADAR_OUTPUT { 18903,22477  }
// 2011-03-08 Adjusted to { 1.8 Vpp East, 1.6 Vpp West } TX output 400-600 W_peak at 15 MHz --KTS
#define DDS_MAX_RADAR_OUTPUT { 28355,30355 }

#endif
