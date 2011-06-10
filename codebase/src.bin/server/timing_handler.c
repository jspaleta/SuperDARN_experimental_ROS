#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include "control_program.h"
#include "global_server_variables.h"

extern int timingsock;
extern pthread_mutex_t timing_comm_lock;

extern int verbose;
extern struct TRTimes bad_transmit_times;
void *timing_ready_controlprogram(struct ControlProgram *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);
   if (arg!=NULL) {
     if (arg->state->pulseseqs[arg->parameters->current_pulseseq_index]!=NULL) {
       msg.type=TIMING_CtrlProg_READY;
       msg.status=1;
       send_data(timingsock, &msg, sizeof(struct DriverMsg));
       send_data(timingsock, arg->parameters, sizeof(struct ControlPRM));
       recv_data(timingsock, &msg, sizeof(struct DriverMsg));
     } 
   }
  pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};

void *timing_end_controlprogram(void *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);
  msg.type=TIMING_CtrlProg_END;
  msg.status=1;
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_register_seq(struct ControlProgram *control_program)
{
  struct DriverMsg msg;
  int index;
  pthread_mutex_lock(&timing_comm_lock);
  msg.type=TIMING_REGISTER_SEQ;
  msg.status=1;
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  send_data(timingsock, control_program->parameters, sizeof(struct ControlPRM));
  index=control_program->parameters->current_pulseseq_index;
  send_data(timingsock, &index, sizeof(index)); //requested index
  send_data(timingsock,control_program->state->pulseseqs[index], sizeof(struct TSGbuf)); // requested pulseseq
  send_data(timingsock,control_program->state->pulseseqs[index]->rep, 
    sizeof(unsigned char)*control_program->state->pulseseqs[index]->len); // requested pulseseq
  send_data(timingsock,control_program->state->pulseseqs[index]->code, 
    sizeof(unsigned char)*control_program->state->pulseseqs[index]->len); // requested pulseseq
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
}
void *timing_pretrigger(void *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);
  msg.type=TIMING_PRETRIGGER;
  msg.status=1;
//  printf("TIMING: PRETRIGGER: Send msg\n");
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
//  printf("TIMING: PRETRIGGER: free %p %p\n",bad_transmit_times.start_usec,bad_transmit_times.duration_usec);
  if(bad_transmit_times.start_usec!=NULL) free(bad_transmit_times.start_usec);
  if(bad_transmit_times.duration_usec!=NULL) free(bad_transmit_times.duration_usec);
  bad_transmit_times.start_usec=NULL;
  bad_transmit_times.duration_usec=NULL;
//  printf("TIMING: PRETRIGGER: free end\n");
//  printf("TIMING: PRETRIGGER: recv bad_transmit times object\n");
  recv_data(timingsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
//  printf("TIMING: PRETRIGGER: length %d\n",bad_transmit_times.length);
  if (bad_transmit_times.length>0) {
//    printf("TIMING: PRETRIGGER: Mallocs start\n");
    bad_transmit_times.start_usec=malloc(sizeof(unsigned int)*bad_transmit_times.length);
    bad_transmit_times.duration_usec=malloc(sizeof(unsigned int)*bad_transmit_times.length);
//    printf("TIMING: PRETRIGGER: Mallocs end\n");
  } else {
    bad_transmit_times.start_usec=NULL;
    bad_transmit_times.duration_usec=NULL;
  }
//  printf("TIMING: PRETRIGGER: recv start usec object\n");
  recv_data(timingsock, bad_transmit_times.start_usec, sizeof(unsigned int)*bad_transmit_times.length);
//  printf("TIMING: PRETRIGGER: recv duration usec object\n");
  recv_data(timingsock, bad_transmit_times.duration_usec, sizeof(unsigned int)*bad_transmit_times.length);
//  printf("TIMING: PRETRIGGER: recv msg\n");
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
//  printf("TIMING: PRETRIGGER: exit\n");
  pthread_exit(NULL);
};

void *timing_trigger(int trigger_type)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);
  switch(trigger_type) {
    case 0:
//  printf("TIMING: Normal Trigger Type %d\n",trigger_type);
      msg.type=TIMING_TRIGGER;
      break;
    case 1:
//  printf("TIMING: Normal Trigger Type %d\n",trigger_type);
      msg.type=TIMING_TRIGGER;
      break;
    case 2:
//  printf("TIMING: GPS Trigger Type %d\n",trigger_type);
      msg.type=TIMING_GPS_TRIGGER;
      break;
  }
  msg.status=1;
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_wait(void *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);
  msg.type=TIMING_WAIT;
  msg.status=1;
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};
void *timing_posttrigger(void *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);

   msg.type=TIMING_POSTTRIGGER;
   msg.status=1;
   send_data(timingsock, &msg, sizeof(struct DriverMsg));
   recv_data(timingsock, &msg, sizeof(struct DriverMsg));
   pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};

/*
void *timing_handler(void *arg)
{

  pthread_mutex_lock(&timing_comm_lock);
   if (verbose>1) fprintf(stderr,"Inside the timing handler\n");
   if (verbose>1) fprintf(stderr,"Timing: Do some work\n");
   if (verbose>1) fprintf(stderr,"Leaving timing handler\n");
  pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};
*/

