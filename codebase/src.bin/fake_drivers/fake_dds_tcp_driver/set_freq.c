/* function set_freq */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <sys/mman.h>
#ifdef __QNX__
  #include  <hw/pci.h>
  #include <hw/inout.h>
#endif
#include "ics660b.h"
#include "dds_defs.h"


int set_freq( int freq_in, FILE *ics660[]){
  int pci_ind;
  int pci_min=0;
  int pci_max=0;
  int channel,chip;
  double freq;
  double state_time=STATE_TIME; 

  int status=0;

  freq = (double)freq_in * 1000.;

// fprintf(stderr,"ICS660 SET_FREQ - freq  %f\n",freq);

  


  for( pci_ind=pci_max; pci_ind >= pci_min; pci_ind --)
    { 
      
      for( chip=1; chip<=4; chip++){
	for(channel=1; channel<=4; channel++){

	  load_frequency(ics660[pci_ind], chip, channel, freq);
	  
	}
      }
      
    }
    pci_ind=0;
     one_shot_b(ics660[pci_ind]);
   
 // return(status);
  
}
