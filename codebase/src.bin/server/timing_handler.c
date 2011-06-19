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
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&timing_comm_lock);
   if (arg!=NULL) {
     if (arg->state->pulseseqs[arg->parameters->current_pulseseq_index]!=NULL) {
       s_msg.type=TIMING_CtrlProg_READY;
       s_msg.status=1;
       send_data(timingsock, &s_msg, sizeof(struct DriverMsg));
       recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
       if(r_msg.status==1) {	
         send_data(timingsock, arg->parameters, sizeof(struct ControlPRM));
         recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
       }
     } 
   }
  pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};

void *timing_end_controlprogram(struct ControlProgram *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&timing_comm_lock);
  if (arg!=NULL) {
    s_msg.type=TIMING_CtrlProg_END;
    s_msg.status=1;
    send_data(timingsock, &s_msg, sizeof(struct DriverMsg));
    recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
    if(r_msg.status==1) {	
         send_data(timingsock, arg->parameters, sizeof(struct ControlPRM));
         recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
    }
  }
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_register_seq(struct ControlProgram *control_program)
{
  struct DriverMsg s_msg , r_msg;
  int32 index;
  pthread_mutex_lock(&timing_comm_lock);
  s_msg.type=TIMING_REGISTER_SEQ;
  s_msg.status=1;
  send_data(timingsock, &s_msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) {
    send_data(timingsock, control_program->parameters, sizeof(struct ControlPRM));
    index=control_program->parameters->current_pulseseq_index;
    send_data(timingsock, &index, sizeof(index)); //requested index
    send_data(timingsock,control_program->state->pulseseqs[index], sizeof(struct TSGbuf)); // requested pulseseq
    send_data(timingsock,control_program->state->pulseseqs[index]->rep, 
      sizeof(unsigned char)*control_program->state->pulseseqs[index]->len); // requested pulseseq
    send_data(timingsock,control_program->state->pulseseqs[index]->code, 
      sizeof(unsigned char)*control_program->state->pulseseqs[index]->len); // requested pulseseq
    recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
  }
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
}
void *timing_pretrigger(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&timing_comm_lock);
  s_msg.type=TIMING_PRETRIGGER;
  s_msg.status=1;
  send_data(timingsock, &s_msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));

  s_msg.type=GET_TRTIMES;
  s_msg.status=1;
  send_data(timingsock, &s_msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) {
    if(bad_transmit_times.start_usec!=NULL) free(bad_transmit_times.start_usec);
    if(bad_transmit_times.duration_usec!=NULL) free(bad_transmit_times.duration_usec);
    bad_transmit_times.start_usec=NULL;
    bad_transmit_times.duration_usec=NULL;
    recv_data(timingsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
    if (bad_transmit_times.length>0) {
      bad_transmit_times.start_usec=malloc(sizeof(unsigned int)*bad_transmit_times.length);
      bad_transmit_times.duration_usec=malloc(sizeof(unsigned int)*bad_transmit_times.length);
      recv_data(timingsock, bad_transmit_times.start_usec, sizeof(unsigned int)*bad_transmit_times.length);
      recv_data(timingsock, bad_transmit_times.duration_usec, sizeof(unsigned int)*bad_transmit_times.length);
    } else {
      bad_transmit_times.start_usec=NULL;
      bad_transmit_times.duration_usec=NULL;
    }
    recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
  }
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_trigger(int *trigger_type_p)
{
  struct DriverMsg s_msg,r_msg;
  int trigger_type;
  trigger_type=*trigger_type_p;
  pthread_mutex_lock(&timing_comm_lock);
 
  switch(trigger_type) {
    case 0:
      s_msg.type=TIMING_TRIGGER;
      break;
    case 1:
      s_msg.type=TIMING_TRIGGER;
      break;
    case 2:
      s_msg.type=EXTERNAL_TRIGGER;
      break;
  }
  s_msg.status=1;
  send_data(timingsock, &s_msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_wait(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&timing_comm_lock);
  s_msg.type=WAIT;
  s_msg.status=1;
  send_data(timingsock, &s_msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};
void *timing_posttrigger(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&timing_comm_lock);

   s_msg.type=TIMING_POSTTRIGGER;
   s_msg.status=1;
   send_data(timingsock, &s_msg, sizeof(struct DriverMsg));
   recv_data(timingsock, &r_msg, sizeof(struct DriverMsg));
   pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};


