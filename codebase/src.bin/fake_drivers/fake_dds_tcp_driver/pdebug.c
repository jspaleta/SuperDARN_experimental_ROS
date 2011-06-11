#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdint.h>
#include <sys/mman.h>
#ifdef __QNX__
  #include <hw/pci.h>
  #include <hw/inout.h>
  #include <devctl.h>
#endif
#include "ics660b.h"

int pdebug(char *fmt,...)
{
  va_list ap;
  char *p, *sval;
  int ival;
  double dval;

  if(DEBUG){ 
    va_start(ap, fmt); // make ap point to 1st unnamed arg
    for(p=fmt; *p; p++){
      if(*p != '%'){
	putchar(*p);
	continue;
      }
      switch( *++p){
      case 'd':
	ival = va_arg(ap,int);
	printf("%d",ival);
	break;
	 
      case 'f':
	dval = va_arg(ap, double);
	printf("%f",dval);
	break;

      case 's':
	for( sval = va_arg(ap,char *); *sval; sval++ )
	  putchar(*sval);
	break;

      default:
	putchar(*p);
	break;
      }
    }
    va_end(ap);
  }
  fflush(stdout);
}
