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
void *timing_ready_controlprogram(struct ControlProgram *control_program)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,CtrlProg_READY,"ctrlprog_ready","NONE");
  pthread_mutex_lock(&timing_comm_lock);
  if (control_program!=NULL) {
     if (control_program->state->pulseseqs[control_program->parameters->current_pulseseq_index]!=NULL) {
       driver_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
       driver_msg_send(timingsock, &s_msg);
       driver_msg_recv(timingsock, &r_msg);
     }
  }
  pthread_mutex_unlock(&timing_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);

};

void *timing_end_controlprogram(struct ControlProgram *control_program)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  pthread_mutex_lock(&timing_comm_lock);
  driver_msg_set_command(&s_msg,CtrlProg_END,"ctrlprog_end","NONE");
  driver_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
  driver_msg_send(timingsock, &s_msg);
  driver_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);

};

void *timing_register_seq(struct ControlProgram *control_program)
{
 struct DriverMsg s_msg,r_msg;
  int32 index;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,REGISTER_SEQ,"register_seq","DDS");
  if (control_program!=NULL) {
    if (control_program->parameters!=NULL) {
      if (control_program->state!=NULL) {
        driver_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
        driver_msg_add_var(&s_msg,&index,sizeof(index),"index","int32");
        driver_msg_add_var(&s_msg,control_program->state->pulseseqs[index],sizeof(struct TSGbuf),"pulseseq","struct TDGBuf");
        driver_msg_add_var(&s_msg,control_program->state->pulseseqs[index]->rep,sizeof(unsigned char)*control_program->state->pulseseqs[index]->len,"rep","array");
        driver_msg_add_var(&s_msg,control_program->state->pulseseqs[index]->code,sizeof(unsigned char)*control_program->state->pulseseqs[index]->len,"code","array");
      }
    }
  }
  pthread_mutex_lock(&timing_comm_lock);
  driver_msg_send(timingsock, &s_msg);
  driver_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *timing_set_trigger_offset(struct ControlProgram *control_program)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  if (control_program!=NULL) {
    if (control_program->parameters!=NULL) {
      if (control_program->state!=NULL) {
        driver_msg_set_command(&s_msg,SET_TRIGGER_OFFSET,"set_trigger_offset","NONE");
        driver_msg_add_var(&s_msg,&control_program->state->rx_trigger_offset_usec,sizeof(int32),"rx_trigger_offset_usec","int32");
        driver_msg_add_var(&s_msg,&control_program->state->dds_trigger_offset_usec,sizeof(int32),"dds_trigger_offset_usec","int32");
        driver_msg_add_var(&s_msg,&control_program->parameters->radar,sizeof(int32),"radar","int32");
        driver_msg_add_var(&s_msg,&control_program->parameters->channel,sizeof(int32),"channel","int32");

        pthread_mutex_lock(&timing_comm_lock);
        driver_msg_send(timingsock, &s_msg);
        driver_msg_recv(timingsock, &r_msg);
        pthread_mutex_unlock(&timing_comm_lock);

        driver_msg_free_buffer(&s_msg);
        driver_msg_free_buffer(&r_msg);
        pthread_exit(NULL);
      }
    }  
  }
}

void *timing_pretrigger(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,PRETRIGGER,"pretrigger","NONE");

  pthread_mutex_lock(&timing_comm_lock);
  driver_msg_send(timingsock, &s_msg);
  driver_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);

  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);

  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,GET_TRTIMES,"get_trtimes","NONE");

  pthread_mutex_lock(&timing_comm_lock);
  driver_msg_send(timingsock, &s_msg);
  driver_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);

  if(r_msg.status==1) {
    if(bad_transmit_times.start_usec!=NULL) free(bad_transmit_times.start_usec);
    if(bad_transmit_times.duration_usec!=NULL) free(bad_transmit_times.duration_usec);
    bad_transmit_times.start_usec=NULL;
    bad_transmit_times.duration_usec=NULL;
    driver_msg_get_var_by_name(&r_msg,"num_tr_windows",&bad_transmit_times.length);
    if (bad_transmit_times.length>0) {
      bad_transmit_times.start_usec=malloc(sizeof(unsigned int)*bad_transmit_times.length);
      bad_transmit_times.duration_usec=malloc(sizeof(unsigned int)*bad_transmit_times.length);
      driver_msg_get_var_by_name(&r_msg,"tr_windows_start_usec",&bad_transmit_times.length);
      driver_msg_get_var_by_name(&r_msg,"tr_windows_duration_usec",&bad_transmit_times.length);
    } else {
      bad_transmit_times.length=0;	
      bad_transmit_times.start_usec=NULL;
      bad_transmit_times.duration_usec=NULL;
    }
  } else {
      bad_transmit_times.length=0;	
      bad_transmit_times.start_usec=NULL;
      bad_transmit_times.duration_usec=NULL;
  }
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
};

void *timing_trigger(int *trigger_type_p)
{
  struct DriverMsg s_msg,r_msg;
  int trigger_type;
  trigger_type=*trigger_type_p;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  switch(trigger_type) {
    case 0:
      driver_msg_set_command(&s_msg,TRIGGER,"trigger","NONE");
      break;
    case 1:
      driver_msg_set_command(&s_msg,TRIGGER,"trigger","NONE");
      break;
    case 2:
      driver_msg_set_command(&s_msg,EXTERNAL_TRIGGER,"external_trigger","NONE");
      break;
  }
  pthread_mutex_lock(&timing_comm_lock);
  driver_msg_send(timingsock, &s_msg);
  driver_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_wait(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,WAIT,"wait","NONE");
  pthread_mutex_lock(&timing_comm_lock);
  driver_msg_send(timingsock, &s_msg);
  driver_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_posttrigger(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,POSTTRIGGER,"posttrigger","NONE");
  pthread_mutex_lock(&timing_comm_lock);
  driver_msg_send(timingsock, &s_msg);
  driver_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};


