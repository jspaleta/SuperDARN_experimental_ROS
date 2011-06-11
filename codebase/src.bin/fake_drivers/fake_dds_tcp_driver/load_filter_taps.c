/* Function to load the PFIR filter taps 

   arguments are the 

     channel - 1,2,3,or 4
     fc - the corner frequency of the filter (in base band)
     t_in - the state time increment (i.e. data input rate)

*/
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <sys/mman.h>
#ifdef __QNX__
  #include  <hw/pci.h>
  #include <devctl.h>
  #include <hw/inout.h>
#endif
#include "ics660b.h"

extern int verbose;

void load_filter_taps(FILE *ics660, uint32_t chip, uint32_t channel, int fc, double t_in)
{
  struct ICS660_FILTER filter_str;
  if (verbose > 1) printf("  In Load filter taps file:%d chip:%d channel:%d trise:%d state:%lf\n"
                          ,ics660,chip,channel,fc,t_in);	

  filter_str.chip = chip;
  filter_str.channel = channel;
  filter_str.f_corner = fc;
  filter_str.state_time = t_in;
#ifdef __QNX__
  ics660_set_parameter((int)ics660,(int)ICS660_LOAD_FILTER,&filter_str,sizeof(filter_str));
#endif
}
