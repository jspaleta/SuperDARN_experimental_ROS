/*set_filter function*/
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#ifdef __QNX__
  #include  <hw/pci.h>
  #include <devctl.h>
  #include <sys/stat.h>
  #include <hw/inout.h>
  #include <sys/resource.h>
#endif
#include "ics660b.h"
#include "dds_defs.h"

extern int verbose;

int set_filter(int T_rise,FILE *ics660[]){
  int chip,channel,pci_ind;

  int pci_max = 0;
  int pci_min = 0;
  double state_time=STATE_TIME;
 // double BW;
 // BW=double((2.5*(state_time/T_rise))^2);


  for( pci_ind=pci_max; pci_ind>=pci_min; pci_ind-- ){
    for( chip=1; chip<=4; chip++ ){
      for( channel=1; channel<=4; channel++ ){
        if (verbose > 0) printf("\n  In set_filter pci_ind:%d chip:%d channel:%d trise:%d state:%lf\n"
                          ,pci_ind,chip,channel,T_rise,state_time);	
	load_filter_taps(ics660[pci_ind],chip,channel,T_rise,state_time);
	//load_filter_taps(ics660[pci_ind],chip,channel,BW);
}
}
}
 pci_ind=0;
 one_shot_b(ics660[pci_ind]);
}
