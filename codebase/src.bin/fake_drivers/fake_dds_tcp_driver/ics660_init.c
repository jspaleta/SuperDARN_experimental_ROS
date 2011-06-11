/* Program ics660_init */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <sys/mman.h>
#ifdef __QNX__
  #include  <hw/pci.h>
  #include <hw/inout.h>
  #include <devctl.h>
#endif
#include "ics660b.h"
#include "dds_defs.h"
extern int verbose;

long ics660_init(FILE *ics660, int pci_ind)
{
  long signal_length = SIGLEN;
  long status;
  long interp_val;
  double state_time = STATE_TIME;
  struct ICS660_sequencer seq_data;
  uint32_t sequencer_base = 0x00100000;
  long wr_addr,bl_adv,seq_word;
  int pci_term=DDS_TERM_INDEX;
  int pci_master=DDS_MASTER_INDEX;
  int zero=0;
  int disable=(int)DISABLE;
  int enable=(int)ENABLE;
  int external=(int)EXTERNAL;
  int internal=(int)INTERNAL;
  int pulse=(int)PULSE;
  int pci=(int)PCI;
  int i;
  struct timespec sl_time,hold_time;

  sl_time.tv_sec = (time_t)0;
  sl_time.tv_nsec = (long)500000;


  pdebug("ICS660_INIT - pci_ind = %d \n",pci_ind);

#ifdef __QNX__
  /*** Select clock source (internal/external) */
  pdebug("ICS660_INIT - CLOCK_SOURCE pci_ind = %d \n",pci_ind);
  ics660_set_parameter(ics660,(int)ICS660_CLOCK_SELECT,&external,sizeof(external));

  //pdebug("ICS660_INIT - BOARD_RESET pci_ind = %d %d\n",pci_ind,BOARD_RESET);
  ics660_set_parameter(ics660,(int)ICS660_BOARD_RESET,&zero,sizeof(zero));
  //pdebug("ICS660_INIT - CLEAR_IMASK pci_ind = %d \n",pci_ind);
  ics660_set_parameter(ics660,(int)ICS660_CLEAR_IMASK,&zero,sizeof(zero));
  //nanosleep(&sl_time,&hold_time);

  /* Set control register */
  
  /*** Set DAC_ENABLE bit to disable */
  pdebug("ICS660_INIT - DAC_ENABLE pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_DAC_ENABLE,&disable,sizeof(disable));

  /*** Select trigger source (internal/external) */
  pdebug("ICS660_INIT - TRIGGER_SOURCE pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_TRIGGER_SOURCE,&external,sizeof(external));

  /*** Select clock source (internal/external) */
  pdebug("ICS660_INIT - CLOCK_SOURCE pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_CLOCK_SELECT,&external,sizeof(external));

  /*** Select operating mode (continuous/one-shot/loop/pulse) */
  pdebug("ICS660_INIT - MODE pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_MODE,&pulse,sizeof(pulse));

  /*** Select upconverter */
  pdebug("ICS660_INIT - UPCONVERTER_SELECT pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_UPCONVERTER_SELECT,&enable,sizeof(enable));
  
  /*** Select input data path (FPDP ot PCI) */
  pdebug("ICS660_INIT - DATA_SOURCE pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_DATA_SOURCE,&pci,sizeof(pci));
  
  /*** Set sync master/term bits as required */
  pdebug("ICS660_INIT - SYNC_MASTER pci_ind = %d \n",pci_ind);
  fflush(stdout);
  if(pci_ind == pci_master){ 
    if (verbose>0) printf("Setting Sync Master %d\n",pci_ind);
    ics660_set_parameter(ics660,(int)ICS660_SYNC_MASTER,&enable,sizeof(enable));
  } else {
    ics660_set_parameter(ics660,(int)ICS660_SYNC_MASTER,&disable,sizeof(enable));
  }
  
  pdebug("ICS660_INIT - SYNC_TERM pci_ind = %d \n",pci_ind);
  fflush(stdout);
  /*** Set sync term on last board in chain ***/
  if(pci_ind==pci_term){
    if (verbose>0) printf("Setting Sync Terminator %d\n",pci_ind);
    ics660_set_parameter(ics660,(int)ICS660_SYNC_TERM,&enable,sizeof(enable));
  } else {
    ics660_set_parameter(ics660,(int)ICS660_SYNC_TERM,&disable,sizeof(disable));
  }
    
  /* Set PERSISTENCE register */
  pdebug("ICS660_INIT - PERSISTENCE pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_PERSISTENCE_REG,&zero,sizeof(zero));

  /* Set TRANSMIT_LENGTH register */
  pdebug("ICS660_INIT - TRANSMIT_LENGTH pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_TRANS_LENGTH,&signal_length,sizeof(signal_length));  

  /* Set BANK_LENGTH register */
  pdebug("ICS660_INIT - BANK_LENGTH pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_BANK_LENGTH,&signal_length,sizeof(signal_length));

  /* Set BANK_WIDTH register */
  pdebug("ICS660_INIT - BANK_WIDTH pci_ind = %d \n",pci_ind);
  fflush(stdout);
  ics660_set_parameter(ics660,(int)ICS660_BANK_WIDTH,&zero,sizeof(zero));


  /* Program write sequence generator */
 
  for( i=0; i<32; i++ ){
    wr_addr = (31 - i) % 16;
    bl_adv = ( i == 15 | i == 31 ) ? 1 : 0;
    seq_word = (bl_adv << 6 | wr_addr << 2) & 0x0000007c; 
    seq_data.offset = (uint32_t)(4*i);
    seq_data.value = (uint32_t)seq_word;
    pdebug("ICS660_INIT - WRITE_SEQUENCER pci_ind = %d  %x\n",pci_ind,seq_word);
    ics660_set_parameter(ics660,(int)ICS660_WRITE_SEQUENCER,&seq_data,sizeof(seq_data));
  }
 
  /* Configure upconverter */
  
 interp_val = (uint32_t)((state_time*CLOCK_FREQ)/4+0.499999);
// interp_val = (uint32_t)(50);
  
  status = dc60m_init(ics660,interp_val);
  
  return status;
#else
  return 0;

#endif
}

