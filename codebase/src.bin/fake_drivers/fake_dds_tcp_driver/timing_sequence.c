/* Program ics660_timingseq */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#ifdef __QNX__
#include <devctl.h>
#include <hw/inout.h>
#include <sys/resource.h>
#include  <hw/pci.h>
#endif
#include "ics660b.h"
#include "dds_defs.h"
#include "global_server_variables.h"

extern int verbose;
//extern struct RXFESettings rxfe_settings;
extern uint32 ifmode;

int msi_timing_sequence(int numclients,  int max_seq_length, struct  ControlPRM  *clients,int seqlength[MAX_RADARS][MAX_CHANNELS] ,int **client_seqs[MAX_RADARS][MAX_CHANNELS],int active[MAX_RADARS][DDS_MAX_CHANNELS],FILE *ics660[4])
{
  long SC,SO;
  int dds_nchan=16;
  int pci_ind=0,pci_min;
  int temp,i,j,ioff,r,rr,a,c,cc,dds_chan;
  int status;
  int bank_width;
  int bank_length;
  int transmit_length;
  int reset_value;
  int channel,chip,b_ind,seqlen;
  int enable = ENABLE;
  int external=(int)EXTERNAL;
  int pulse_len, ind, sig_lev;
  uint32_t tlen,blen;
  double freq,phi_inc,phi_dif;
  double phase,pi;
  float synth_state  = STATE_TIME;
  uint32_t *data_ar;
  union dac_samples {
    uint32_t two_samp;
    struct {
      short low;
      short high;
    } samples;
  } dac_value;
  short max_val[MAX_RADARS][DDS_MAX_CHANNELS+1]={0,32767,16383,8192,4096};
  short rf_val[MAX_RADARS]=DDS_MAX_RADAR_OUTPUT;
  short if_val[MAX_RADARS];  
//  short max_val[5]={0,15500,23000,32767,32767};
  int num_active_chans[MAX_RADARS]; 
  float baudfact,codelen;
  char trans_seq[200000];
  float t_seq_state;
  int flag;
// Samples per channel on each trigger
//JDS add an offset for zeros at the end...because of IF
  SO=max_seq_length+100.0E-6/STATE_TIME;
// Total number of samples
  SC=dds_nchan*SO;
  pci_min=0;
  for (r=0; r<MAX_RADARS;r++) {
    if_val[r]=32767;
    max_val[r][0]=0;
    max_val[r][1]=rf_val[r];
    for( c=2; c<=DDS_MAX_CHANNELS; c++){
      max_val[r][c]=max_val[r][1]/c;
    }
  }
  if (verbose > -1) printf("Inside MSI timing sequence clients:%d SC: %d Max: %d\n",numclients,SC,max_seq_length);	
#ifdef __QNX__
  for( pci_ind=pci_min; pci_ind < DDS_MAX_CARDS; pci_ind ++) {
     status=ics660_init(ics660[pci_ind],pci_ind);
  }
#endif

  for( r=0; r<MAX_RADARS; r++){
    for( c=0; c<DDS_MAX_CHANNELS; c++){
      active[r][c]=-1; 
    }
    num_active_chans[r]=0;
  } 
  data_ar = (uint32_t *)calloc((size_t)SC, (size_t)sizeof(uint32_t));
 // printf("ICS660_XMT SC = %d\n",SC);
  for( j=0; j<numclients; j++){
        r=clients[j].radar-1; 
        c=clients[j].channel-1; 
        active[r][c]=c; 
        num_active_chans[r]++;
  } 
//  for (j=0;j<numclients; j++) {
//    r=clients[j].radar-1;
//    c=clients[j].channel-1;
//    cc=0;
//    for(i=DDS_MAX_CHANNELS/num_active_chans[r];i>0;i--) {
//      if((active[r][cc]==-1) && (cc < DDS_MAX_CHANNELS))   active[r][cc]=c;
//      cc++;
//    }
//    num_active_chans[r]=3;
//  }
  if (verbose > -1) {
   printf("  timing sequence active channels on each output:\n");
   for( r=0; r<MAX_RADARS; r++){
     printf("    radar %d   active: %d \n",r,num_active_chans[r]);
     for (c=0;c<DDS_MAX_CHANNELS;c++) {
       printf("     chan : %d  val : %d\n",c,active[r][c]);
     }
   }
     
  }
  for( j=0; j<numclients; j++){
        if (verbose > 1) printf("Client Number :%d radar:%d channel:\n",j,clients[j].radar,clients[j].channel);
        r=clients[j].radar-1; 
        rr=r+2;
        c=clients[j].channel-1; 
        printf( "Radar: %d Active Number of channels: %d  Max Value: %hd %hx\n",r,num_active_chans[r],max_val[r][num_active_chans[r]],max_val[r][num_active_chans[r]]);
        printf ("IF Signal radar: %d: Output: %d %d %d %d %d\n",r,rr,seqlength[r][c],max_seq_length,dds_nchan,DDS_MAX_CHANNELS);
        if (verbose > 1) printf("r: %d c:%d count: %d\n",r,c,seqlength[r][c]);	
        flag=0;
        for( i=0; i<seqlength[r][c]; i++){
          if (verbose > 1) {
            if (client_seqs[r][c][i]!=0 && flag==0) {
              if (verbose > 1) printf("Beginning of TX: %d\n",i);
              flag=1;
            }
            if (client_seqs[r][c][i]==0 && flag==1) {
              if (verbose > 1) printf("End of TX: %d\n",i-1);
              flag=0;
            }
          }
          ioff = i*dds_nchan;
          cc=0;
          while( cc < DDS_MAX_CHANNELS ) {
            if(active[r][cc]==c) {
              if(IMAGING==0) {
                dds_chan=r*DDS_MAX_CHANNELS+cc;
                ind = (ioff + dds_chan);
                dac_value.samples.low = (short)client_seqs[r][c][i]*max_val[r][num_active_chans[r]];
                dac_value.samples.high =(short)client_seqs[r][c][i]*max_val[r][num_active_chans[r]];
                data_ar[ind] = (uint32_t) dac_value.two_samp;
              } else {
                //printf(" IMAGING: Active r: %d Index: %d\n",r,i);
                //IMAGING
                for ( a=0;a<4;a++ ) {
                  dds_chan=a*DDS_MAX_CHANNELS+cc;
                  ind = (ioff + dds_chan);
                  dac_value.samples.low = (short)client_seqs[r][c][i]*max_val[r][num_active_chans[r]];
                  dac_value.samples.high =(short)client_seqs[r][c][i]*max_val[r][num_active_chans[r]];
                  data_ar[ind] = (uint32_t) dac_value.two_samp;
                }
              }
            }
            cc++;
          }
        } //end sequence loop 
        if (verbose > 1) printf("End of client\n");	
        if (ifmode==1) {                                                 
          if(IMAGING==0) {
            printf ("IF Signal radar: %d Output: %d %d %d %d %d\n",r,rr,seqlength[r][c],max_seq_length,dds_nchan,DDS_MAX_CHANNELS);
            for( i=0; i<max_seq_length-1; i++){                               
              cc=0;  
              ioff = i*dds_nchan;
              dds_chan=rr*DDS_MAX_CHANNELS+cc;                               
              ind=(ioff+dds_chan);                                      
              dac_value.samples.low = (short)1*if_val[r];                 
              dac_value.samples.high =(short)1*if_val[r];                 
              if (ind < SC)
                data_ar[ind] = (uint32_t) dac_value.two_samp;             
              else printf("Index too big: %d %d\n",ind,SC);
            }                                                             
          } else {
            fprintf(stderr,"Error: IF cannot be enabled in IMAGING Mode\n");
          }
        }            
      } // end of client loop

      if (verbose > 1) printf("Inside MSI timing sequence 3\n");	
    
    
  printf("timing seq is ready to be sent\n");
#ifdef __QNX__
  for( pci_ind=pci_min; pci_ind < DDS_MAX_CARDS; pci_ind ++) {
      printf("  pci_ind: %d\n",pci_ind);
      bank_length = SC/2-1;
      bank_width = 0;
      transmit_length = SC/2-1;
      ics660_set_parameter((int)ics660[pci_ind],ICS660_BANK_LENGTH,&bank_length, sizeof(bank_length));
      ics660_set_parameter((int)ics660[pci_ind],ICS660_BANK_WIDTH,&bank_width, sizeof(bank_width));
      ics660_set_parameter((int)ics660[pci_ind],ICS660_TRANS_LENGTH,&transmit_length, sizeof(transmit_length));

      /* Write DAC Reset register */
      reset_value = RESET_VALUE;
      ics660_set_parameter((int)ics660[pci_ind],ICS660_DAC_RESET,&reset_value, sizeof(RESET_VALUE));

      write((int)ics660[pci_ind],data_ar,(size_t)4*(SC));
      write((int)ics660[pci_ind],data_ar,(size_t)4*(SC));
      
    }  
     /* Set DAC enable bit in control register */
      for( pci_ind=pci_min; pci_ind < DDS_MAX_CARDS; pci_ind ++) 
        ics660_set_parameter((int)ics660[pci_ind],ICS660_DAC_ENABLE,&enable, sizeof(enable));
      for( pci_ind=pci_min; pci_ind < DDS_MAX_CARDS; pci_ind ++) 
        ics660_set_parameter((int)ics660[pci_ind],ICS660_SET_DC_READY, &enable, sizeof(enable));
  
      for( pci_ind=pci_min; pci_ind < DDS_MAX_CARDS; pci_ind ++) 
        ics660_set_parameter((int)ics660[pci_ind],ICS660_RELEASE_RESETS, &enable, sizeof(enable));
#endif
  free(data_ar);      
  if (verbose > 1) printf("Leaving MSI timing sequence\n");	

  return 0;

}
