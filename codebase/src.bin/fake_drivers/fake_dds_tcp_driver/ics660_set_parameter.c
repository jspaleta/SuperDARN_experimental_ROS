/* Program ics660_xmt */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#ifdef __QNX__
  #include <devctl.h>
  #include <hw/pci.h>
  #include <hw/inout.h>
  #include <sys/resource.h>
#endif
#include "ics660b.h"

int ics660_set_parameter(FILE *ics660, int parameter, void* value, size_t nbytes)
{
  int status, error;
  int ICS660_PARAMETER;
#ifdef __QNX__
  pdebug("ICS660_SET_PARAMETER - parameter = %x  %x\n",parameter,(int)value);
  if( error = devctl((int) ics660,(int)parameter, value, nbytes,(int *)&status))
    { 
      fprintf(stderr,"- ICS660_SET_PARAMETER ERROR - setting parameter %d  %s\n",parameter,
	      strerror(error));
    }
  return(status);
#else
  return 0;
#endif
}
