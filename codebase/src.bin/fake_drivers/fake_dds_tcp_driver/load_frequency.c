/* Function to load the frequency and phase of a channel 

   arguments are the 

   dc60m_regs - the register locations
   channel - the channel number (1-4)
   fr - the input frequency in Hz
   ph - the input phase in radians
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

void load_frequency(FILE *ics660, uint32_t chip, int channel, double fr)
{
  struct ICS660_FREQ freq_str;
  if (verbose > 1) printf("  In Load frequency file:%d chip:%d channel:%d freq:%lf\n",ics660,chip,channel,fr);	
  freq_str.chip = chip;
  freq_str.channel = channel;
  freq_str.freq = fr;
#ifdef __QNX__  
  ics660_set_parameter(ics660,(int)ICS660_LOAD_FREQUENCY,&freq_str,sizeof(freq_str));
#endif

}
