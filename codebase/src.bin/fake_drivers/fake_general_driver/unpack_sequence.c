#include <string.h>
#include <time.h>
#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"
#include "utils.h"
#include "driver_structures.h"

extern int verbose;
extern dictionary *diagnostic_INI;

int unpack_sequence(int r, int c,t_driver dinfo,struct SeqBuf *pulseseq,unsigned int *seq_buf) {
  int i,j,rep,rep_usec,code,steps,count;
  int state_time_usec=0;
  struct timeval t0,t1;
  char *time_string,command_string[80],value[80],entry_string[80];
  int strlength=0;
  gettimeofday(&t0,NULL);
  if(strcmp(dinfo.type,"TIMING")==0) {
    state_time_usec=dinfo.info.timing.state_time_usec;
  } else if (strcmp(dinfo.type,"DDS")==0){
    state_time_usec=dinfo.info.dds.state_time_usec;
  } else {
    iniparser_set(diagnostic_INI,"unpack_sequence", NULL,NULL);
    iniparser_set(diagnostic_INI,"unpack_sequence:err_string", "Bad driver type",NULL);
    return 0; 
  }
  rep_usec=pulseseq->prm.step_usec;
  if(verbose > 1 ) printf("Unpack: %d %d\n",pulseseq->prm.len,dinfo.max_seq_length);
  if(verbose > 1 ) printf("Unpack: driver_state_time %d seq_state_time %d\n",state_time_usec,rep_usec);
  count=0;

  for(i=0;i<pulseseq->prm.len;i++) {
    rep=pulseseq->rep[i];
    steps=((double)rep*(double)rep_usec)/(double)state_time_usec;
    code=pulseseq->code[i];
    if(verbose > 2 ) printf("%d :: Steps: %d Code %d\n",i,steps,code);
    if(count<dinfo.max_seq_length) {
      for(j=0;j<steps;j++) {
        seq_buf[count]=code;
        count++;
      }
    } else {
      iniparser_set(diagnostic_INI,"unpack_sequence", NULL,NULL);
      iniparser_set(diagnostic_INI,"unpack_sequence:err_string", "sequence too long to process",NULL);
      return 0;
    }
  }
  gettimeofday(&t1,NULL);

  iniparser_set(diagnostic_INI,"unpack_sequence", NULL,NULL);
  sprintf(entry_string,"%s:%s",command_string,"elapsed_secs");
  iniparser_set(diagnostic_INI,entry_string, "0.0",NULL);
  sprintf(entry_string,"%s:%s",command_string,"time_stamp");
  time_string=ctime(&t0.tv_sec);
  strlength=strlen(time_string)-1;
  strncpy(value,time_string,strlength);
  value[strlength]='\0';
  iniparser_set(diagnostic_INI,entry_string, value,NULL);


  return count;
}
