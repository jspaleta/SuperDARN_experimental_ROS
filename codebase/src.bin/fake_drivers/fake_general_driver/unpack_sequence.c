#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"
#include "utils.h"

int unpack_sequence(int r, int c,int state_time_usec,struct TSGbuf *pulseseq,unsigned int *seq_buf) {
  int i,j,rep,rep_usec,code,steps,count;
  count=0;
  for(i=0;i<pulseseq->len;i++) {
    rep=pulseseq->rep[i];
    rep_usec=pulseseq->step;
    steps=((double)rep*(double)rep_usec)/(double)state_time_usec;
    code=pulseseq->code[i];
    for(j=0;j<steps;j++) {
      seq_buf[count]=code;
      count++;
    }
  }
  return count;
}
