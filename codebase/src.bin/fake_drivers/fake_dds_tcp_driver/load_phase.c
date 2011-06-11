/* Function to load the phase of a channel 

   arguments are the 

   dc60m_regs - the register locations
   channel - the channel number (1-4)
   fr - the input frequency in Hz
   ph - the input phase in radians
*/
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#ifdef __QNX__
  #include <devctl.h>
  #include  <hw/pci.h>
  #include <hw/inout.h>
#endif
#include <sys/mman.h>
#include "ics660b.h"

extern int verbose;
void load_phase(FILE *ics660,uint32_t chip, uint32_t channel, double ph)
{
  struct ICS660_PHASE phase_str;
if (verbose > 1) printf("  In Load phase file:%d chip:%d channel:%d phase:%lf\n",ics660,chip,channel,ph);  
  phase_str.chip = chip;
  phase_str.channel = channel;
  phase_str.phase = ph;
#ifdef __QNX__
  ics660_set_parameter(ics660,(int)ICS660_LOAD_PHASE,&phase_str,sizeof(phase_str));
#endif
}
