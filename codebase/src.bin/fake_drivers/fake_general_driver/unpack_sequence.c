#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"
#include "utils.h"
#include "driver_structures.h"

extern int verbose;
extern dictionary *diagnostic_INI;

int unpack_sequence(int r, int c,t_driver dinfo,struct TSGbuf *pulseseq,unsigned int *seq_buf) {
  int i,j,rep,rep_usec,code,steps,count;
  int state_time_usec=0;
  iniparser_set(diagnostic_INI,"unpack_sequence", NULL,NULL);

  if(strcmp(dinfo.type,"TIMING")==0) {
    state_time_usec=dinfo.info.timing.state_time_usec;
  } else if (strcmp(dinfo.type,"DDS")==0){
    state_time_usec=dinfo.info.dds.state_time_usec;
  } else {
    iniparser_set(diagnostic_INI,"unpack_sequence:err_string", "Bad driver type",NULL);
    return 0; 
  }
  count=0;
  for(i=0;i<pulseseq->len;i++) {
    rep=pulseseq->rep[i];
    rep_usec=pulseseq->step;
    steps=((double)rep*(double)rep_usec)/(double)state_time_usec;
    if(steps<dinfo.max_seq_length) {
      code=pulseseq->code[i];
      for(j=0;j<steps;j++) {
        seq_buf[count]=code;
        count++;
      }
    } else {
      iniparser_set(diagnostic_INI,"unpack_sequence:err_string", "sequence too long to process",NULL);
      return 0;
    }
  }
  return count;
}
