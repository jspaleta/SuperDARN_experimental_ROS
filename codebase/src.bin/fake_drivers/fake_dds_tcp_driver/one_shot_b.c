/* function to generate a one-shot pulse from chip 1 */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#ifdef __QNX__
  #include  <hw/pci.h>
  #include <devctl.h>
  #include <sys/stat.h>
  #include <hw/inout.h>
  #include <sys/resource.h>
#endif
#include "ics660b.h"
#include "dds_defs.h"


void one_shot_b(FILE *ics660){
  struct timespec sl_time,hold_time;
  struct dc60m_p_val dc60m_p;

  sl_time.tv_sec = (time_t)0;
  sl_time.tv_nsec = (long)500000;

  dc60m_p.chip = (uint32_t)0x02;
  dc60m_p.value = (uint32_t)0xe5;
#ifdef __QNX__
  ics660_set_parameter(ics660, (int)ICS660_SYNC_MODE,&dc60m_p,sizeof(dc60m_p));

  nanosleep(&sl_time,&hold_time);

  dc60m_p.value = (uint32_t)0x65;
  ics660_set_parameter(ics660, (int)ICS660_SYNC_MODE,&dc60m_p,sizeof(dc60m_p));
#endif
}
