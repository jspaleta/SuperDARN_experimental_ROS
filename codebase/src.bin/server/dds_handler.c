#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include "control_program.h"
#include "global_server_variables.h"

extern int verbose;
extern int ddssock;
extern pthread_mutex_t dds_comm_lock;
void dds_exit(void *arg)
{
/*
   int *sockfd = (int *) arg;
   pthread_t tid;
   tid = pthread_self();
*/
}

void *dds_register_seq(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  struct ControlProgram *control_program;
  int32 index;

  control_program=arg;
  pthread_mutex_lock(&dds_comm_lock);

  s_msg.type=REGISTER_SEQ;
  s_msg.status=1;
  send_data(ddssock, &s_msg, sizeof(struct DriverMsg));
  recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) { 
    send_data(ddssock, control_program->parameters, sizeof(struct ControlPRM));
    index=control_program->parameters->current_pulseseq_index;
    send_data(ddssock, &index, sizeof(index)); //requested index
    send_data(ddssock,control_program->state->pulseseqs[index], sizeof(struct TSGbuf)); // requested pulseseq
//    send_data(ddssock,control_program->state->pulseseqs[index]->prm, sizeof(struct TSGprm)); // requested pulseseq

    send_data(ddssock,control_program->state->pulseseqs[index]->rep, 
    sizeof(unsigned char)*control_program->state->pulseseqs[index]->len); // requested pulseseq
    send_data(ddssock,control_program->state->pulseseqs[index]->code, 
    sizeof(unsigned char)*control_program->state->pulseseqs[index]->len); // requested pulseseq
    recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
  }
  pthread_mutex_unlock(&dds_comm_lock);
  pthread_exit(NULL);
}

void *dds_site_settings(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  struct SiteSettings *site_settings;

  site_settings=arg;
  pthread_mutex_lock(&dds_comm_lock);
  if (site_settings!=NULL) {
    s_msg.type=SITE_SETTINGS;
    s_msg.status=1;
    send_data(ddssock, &s_msg, sizeof(struct DriverMsg));
    recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
    if(r_msg.status==1) {
      send_data(ddssock, &site_settings->ifmode, sizeof(site_settings->ifmode));
      send_data(ddssock, &site_settings->rf_settings, sizeof(struct RXFESettings));
      send_data(ddssock, &site_settings->if_settings, sizeof(struct RXFESettings));
      recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
    }
  }
  pthread_mutex_unlock(&dds_comm_lock);
  pthread_exit(NULL);
}

void *dds_ready_controlprogram(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  struct ControlProgram *control_program;
  control_program=arg;
  pthread_mutex_lock(&dds_comm_lock);
   if (control_program!=NULL) {
     if (control_program->state->pulseseqs[control_program->parameters->current_pulseseq_index]!=NULL) {
       s_msg.type=CtrlProg_READY;
       s_msg.status=1;
       send_data(ddssock, &s_msg, sizeof(struct DriverMsg));
       recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
       if(r_msg.status==1) {
         send_data(ddssock, control_program->parameters, sizeof(struct ControlPRM));
         recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
       }
     } 
   }
   pthread_mutex_unlock(&dds_comm_lock);
   pthread_exit(NULL);
}

void *dds_end_controlprogram(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  struct ControlProgram *control_program;
  control_program=arg;
  pthread_mutex_lock(&dds_comm_lock);
  s_msg.type=CtrlProg_END;
  s_msg.status=1;
  send_data(ddssock, &s_msg, sizeof(struct DriverMsg));
  recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) {
         send_data(ddssock, control_program->parameters, sizeof(struct ControlPRM));
         recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
  }
  pthread_mutex_unlock(&dds_comm_lock);
  pthread_exit(NULL);
}

void *dds_pretrigger(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&dds_comm_lock);
  s_msg.type=PRETRIGGER;
  s_msg.status=1;
  send_data(ddssock, &s_msg, sizeof(struct DriverMsg));
  recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&dds_comm_lock);
  pthread_exit(NULL);

}

void *dds_posttrigger(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&dds_comm_lock);
  s_msg.type=POSTTRIGGER;
  s_msg.status=1;
  send_data(ddssock, &s_msg, sizeof(struct DriverMsg));
  recv_data(ddssock, &r_msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&dds_comm_lock);
  pthread_exit(NULL);
}


