#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"
#include "site_defaults.h"
extern int timingsock;
extern pthread_mutex_t timing_comm_lock;

extern int verbose;
extern struct TRTimes bad_transmit_times;
extern struct SeqBuf *pulseseqs[MAX_RADARS+1][MAX_CHANNELS+1][MAX_SEQS+1];

void *timing_ready_controlprogram(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,CtrlProg_READY,"ctrlprog_ready","NONE");
  pthread_mutex_lock(&timing_comm_lock);
  if (control_program!=NULL) {
     if (control_program->parameters!=NULL) {
       ros_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
       ros_msg_send(timingsock, &s_msg);
       ros_msg_recv(timingsock, &r_msg);
     }
  }
  pthread_mutex_unlock(&timing_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);

};

void *timing_end_controlprogram(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  pthread_mutex_lock(&timing_comm_lock);
  ros_msg_set_command(&s_msg,CtrlProg_END,"ctrlprog_end","NONE");
  ros_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
  ros_msg_send(timingsock, &s_msg);
  ros_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);

};

void *timing_register_seq(struct ControlProgram *control_program)
{
 struct ROSMsg s_msg,r_msg;
  int32 *index;
  index=control_program->parameters->pulseseq_index;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,REGISTER_SEQ,"register_seq","TIMING");
  if (control_program!=NULL) {
    if (control_program->parameters!=NULL) {
      if (control_program->state!=NULL) {
        ros_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
        ros_msg_add_var(&s_msg,index,sizeof(int32)*3,"index","int32 * 3");
        ros_msg_add_var(&s_msg,&pulseseqs[index[0]][index[1]][index[2]]->prm,sizeof(struct SeqPRM),"prm","struct SeqPRM");
        ros_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]]->rep,sizeof(unsigned char)*pulseseqs[index[0]][index[1]][index[2]]->prm.len,"rep","array");
        ros_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]]->code,sizeof(unsigned char)*pulseseqs[index[0]][index[1]][index[2]]->prm.len,"code","array");
        ros_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]]->ptab,sizeof(int)*pulseseqs[index[0]][index[1]][index[2]]->prm.mppul,"ptab","int array");
      }
    }
  }
  pthread_mutex_lock(&timing_comm_lock);
  ros_msg_send(timingsock, &s_msg);
  ros_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *timing_set_trigger_offset(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  if (control_program!=NULL) {
    if (control_program->parameters!=NULL) {
      if (control_program->state!=NULL) {
        ros_msg_set_command(&s_msg,SET_TRIGGER_OFFSET,"set_trigger_offset","NONE");
        ros_msg_add_var(&s_msg,&control_program->state->rx_trigger_offset_usec,sizeof(int32),"rx_trigger_offset_usec","int32");
        ros_msg_add_var(&s_msg,&control_program->state->dds_trigger_offset_usec,sizeof(int32),"dds_trigger_offset_usec","int32");
        ros_msg_add_var(&s_msg,&control_program->parameters->radar,sizeof(int32),"radar","int32");
        ros_msg_add_var(&s_msg,&control_program->parameters->channel,sizeof(int32),"channel","int32");

        pthread_mutex_lock(&timing_comm_lock);
        ros_msg_send(timingsock, &s_msg);
        ros_msg_recv(timingsock, &r_msg);
        pthread_mutex_unlock(&timing_comm_lock);

        ros_msg_free_buffer(&s_msg);
        ros_msg_free_buffer(&r_msg);
        pthread_exit(NULL);
      }
    }  
  }
}

void *timing_pretrigger(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,PRETRIGGER,"pretrigger","NONE");

  pthread_mutex_lock(&timing_comm_lock);
  ros_msg_send(timingsock, &s_msg);
  ros_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);

  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);

  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,GET_TRTIMES,"get_trtimes","NONE");

  pthread_mutex_lock(&timing_comm_lock);
  ros_msg_send(timingsock, &s_msg);
  ros_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);

  if(r_msg.status==1) {
    if(bad_transmit_times.start_usec!=NULL) free(bad_transmit_times.start_usec);
    if(bad_transmit_times.duration_usec!=NULL) free(bad_transmit_times.duration_usec);
    bad_transmit_times.start_usec=NULL;
    bad_transmit_times.duration_usec=NULL;
    ros_msg_get_var_by_name(&r_msg,"num_tr_windows",&bad_transmit_times.length);
    if (bad_transmit_times.length>0) {
      bad_transmit_times.start_usec=malloc(sizeof(unsigned int)*bad_transmit_times.length);
      bad_transmit_times.duration_usec=malloc(sizeof(unsigned int)*bad_transmit_times.length);
      ros_msg_get_var_by_name(&r_msg,"tr_windows_start_usec",&bad_transmit_times.length);
      ros_msg_get_var_by_name(&r_msg,"tr_windows_duration_usec",&bad_transmit_times.length);
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
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
};

void *timing_trigger(int *trigger_type_p)
{
  struct ROSMsg s_msg,r_msg;
  int trigger_type;
  trigger_type=*trigger_type_p;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  switch(trigger_type) {
    case 0:
      ros_msg_set_command(&s_msg,TRIGGER,"trigger","NONE");
      break;
    case 1:
      ros_msg_set_command(&s_msg,TRIGGER,"trigger","NONE");
      break;
    case 2:
      ros_msg_set_command(&s_msg,EXTERNAL_TRIGGER,"external_trigger","NONE");
      break;
  }
  pthread_mutex_lock(&timing_comm_lock);
  ros_msg_send(timingsock, &s_msg);
  ros_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_wait(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,WAIT,"wait","NONE");
  pthread_mutex_lock(&timing_comm_lock);
  ros_msg_send(timingsock, &s_msg);
  ros_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_posttrigger(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,POSTTRIGGER,"posttrigger","NONE");
  pthread_mutex_lock(&timing_comm_lock);
  ros_msg_send(timingsock, &s_msg);
  ros_msg_recv(timingsock, &r_msg);
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};


