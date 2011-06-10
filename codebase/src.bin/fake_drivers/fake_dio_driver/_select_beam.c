#include <sys/types.h>
#ifdef __QNX__
  #include <hw/inout.h>
  #include <sys/socket.h>
  #include <sys/neutrino.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
#endif
#include <math.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "include/plx_functions.h"
#include "include/plx_defines.h"
#include "utils.h"

extern int verbose;
extern double *freqs[MAX_RADARS];
extern double *angles[MAX_RADARS];
extern int num_std_angles[MAX_RADARS], num_angles[MAX_RADARS], num_freqs[MAX_RADARS],std_angle_index_offset[MAX_RADARS],angle_index_offset[MAX_RADARS];


int lookup_beamnm(int r, double freq,double beamnm){
  double df;
  int best_freq_index,best_beamnm_index, beam_address=0;
  if(freqs[r]!=NULL) {
    df=freqs[r][1]-freqs[r][0];
    best_freq_index=(int)(freq-freqs[r][0])/df;
    if (best_freq_index >= num_freqs[r])  best_freq_index=num_freqs[r]-1;
    if (best_freq_index < 0)  best_freq_index=0;
    best_beamnm_index=beamnm;
    if (best_beamnm_index >= num_std_angles[r])  best_beamnm_index=num_std_angles[r]-1;
    if (best_beamnm_index < 0)  best_beamnm_index=0;
    beam_address=best_beamnm_index+best_freq_index*num_freqs[r]+std_angle_index_offset[r];
  }
  return beam_address;
}

int lookup_angle(int r, double freq,double angle){
  double df,dangle;
  int best_freq_index,best_angle_index, beam_address;
  df=freqs[r][1]-freqs[r][0];
  best_freq_index=(int)(freq-freqs[r][0])/df;
  if (best_freq_index >= num_freqs[r])  best_freq_index=num_freqs[r]-1;
  if (best_freq_index < 0)  best_freq_index=0;

  dangle=angles[1]-angles[0];
  best_angle_index=(int)(angle-angles[r][0])/dangle;
  if (best_angle_index >= num_angles[r])  best_angle_index=num_angles[r]-1;
  if (best_angle_index < 0)  best_angle_index=0;
  beam_address=best_angle_index+best_freq_index*num_freqs[r]+angle_index_offset[r];
  return beam_address;
}

/*-BEAM_CODE---------------------------------------------------------*/
int reverse_bits(int data){
        
        int temp=0;
        
        temp=temp + ((data & 1)  << 12);
        temp=temp + ((data & 2)  << 10);
        temp=temp + ((data & 4)  << 8);
        temp=temp + ((data & 8)  << 6);
        temp=temp + ((data & 16)  << 4);
        temp=temp + ((data & 32)  << 2);
        temp=temp + ((data & 64)  << 0);
        temp=temp + ((data & 128)  >> 2);
        temp=temp + ((data & 256)  >> 4);
        temp=temp + ((data & 512)  >> 6);
        temp=temp + ((data & 1024)  >> 8);
        temp=temp + ((data & 2048)  >> 10);
        temp=temp + ((data & 4096)  >> 12);

        return temp;
}


int _select_beam(unsigned int base,struct ControlPRM *client){

        /* This code selects the beam code to use.  
        */
        int beamnm; 
        if (verbose > 1) { 
          printf("DIO: Select beam\n");	
          printf("  Selected Freq [kHz]: %d\n",client->tfreq);	
          printf("  Selected Radar: %d\n",client->radar);	
          printf("  Selected Beamnm: %d\n",client->tbeam);	
          printf("  Selected Beam angle: %f\n",client->tbeamazm);	
          printf("  Selected Beam width: %f\n",client->tbeamwidth);	
        }  
        /* the beam number is 4 bits.  This number
           uses (lo) bits 5-6 of CH0, PortB , and (hi) bits 6-7 of CH0, PortC
           to output the beam number. Note: The beam number is used in addressing
           the old style phasing matrix.
        */
        beamnm=client->tbeam;
        if ( (beamnm>MAX_BEAM) || (beamnm<0) ){
                fprintf(stderr,"INVALID BEAMNM - must be between 0 and %d\n",MAX_BEAM);
        } else {
        }

        return 0;
       
}



