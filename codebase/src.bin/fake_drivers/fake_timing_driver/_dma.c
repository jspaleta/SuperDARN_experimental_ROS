#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#ifdef __QNX__
  #include <hw/inout.h>
#endif
#include "_regs_PLX9080.h"


void DMAxfer(long addr,int dmapadr,int dmaladr,long dmasiz){
  long temp_out,temp_in;
  
#ifdef __QNX__
//  out32(addr+PLX9080_INTCSR,0xf0900);
  out8(addr+PLX9080_DMACSR1,0x8);
  out8(addr+PLX9080_DMACSR1,0x0);
  out32(addr+PLX9080_DMAMODE1,0x00801);
  out32(addr+PLX9080_DMAPADR1,dmapadr);
  out32(addr+PLX9080_DMALADR1,dmaladr);
  out32(addr+PLX9080_DMASIZ1,dmasiz);
  out32(addr+PLX9080_DMADPR1,0x0);
  out8(addr+PLX9080_DMACSR1,0x01);
  out8(addr+PLX9080_DMACSR1,0x03);
#endif
}

void DMA_readout(long addr,int dmapadr,int dmaladr,long dmasiz){
  long temp_out,temp_in;
  
#ifdef __QNX__
  out8(addr+PLX9080_DMACSR1,0x8);
  out8(addr+PLX9080_DMACSR1,0x0);
  out32(addr+PLX9080_DMAMODE1,0x00803);
  out32(addr+PLX9080_DMAPADR1,dmapadr);
  out32(addr+PLX9080_DMALADR1,dmaladr);
  out32(addr+PLX9080_DMASIZ1,dmasiz);
  out32(addr+PLX9080_DMADPR1,0x0);
  out8(addr+PLX9080_DMACSR1,0x01);
  out8(addr+PLX9080_DMACSR1,0x3);
#endif
}


void DMAabort(long addr){
  long temp_out,temp_in;

#ifdef __QNX__
  out8(addr+PLX9080_DMACSR1,0x0);
  out8(addr+PLX9080_DMACSR1,0x4);
#endif
  printf("DMA abort!\n");

}


int DMApoll(long addr) {
  /* This function holds until the DMA transfer on
     DMA channel 1 is done, then returns to the calling
     process. If more than 1 second passes and the DMA
     is not complete, then the function assumes a DMA
     error, and returns an error condition.
  */
  struct timespec interval;
  struct timespec remaining;
  struct timespec start;
  struct timespec stop;
  float  loops=1e12,count;
  int temp_in; 
#ifdef __QNX__
  interval.tv_sec=0;
  interval.tv_nsec=10000;
  clock_gettime(CLOCK_REALTIME,&start);
  count=0;
  while ((in32(addr+PLX9080_DMACSR1) & 0x00000010) !=0x10) {
    clock_gettime(CLOCK_REALTIME,&stop);
    if ((stop.tv_sec-start.tv_sec)>1) {
      DMAabort(addr);
      return -1;
    }
    if ( count > loops ){
      DMAabort(addr);
      return -1;
    }
    count++;
    /* I commented this line out because the nanosleep takes
       MUCH longer than the DMA X-fer, and was chewing up
       0.5ms (the min sleep period in QNX4) unnessisarily.
       However, without this sleep, the processor is 100%
       used during a DMA.  This is not a problem for most
       machines, unless there is a condition that is making
       the DMA take longer than it should.  Adding this line
       back in only consumes and extra 2 ms every pulse
       sequence, and may allow the CPU to be used more...
       if necessary
    */
    
//      nanosleep(&interval,&remaining); 
  }
#endif
  return 0;
}
