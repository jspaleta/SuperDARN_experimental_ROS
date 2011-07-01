#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"

extern int verbose;
extern int ddssock;
extern pthread_mutex_t dds_comm_lock;
extern struct TSGbuf *pulseseqs[MAX_RADARS][MAX_CHANNELS][MAX_SEQS];

void dds_exit(void *arg)
{
/*
   int *sockfd = (int *) arg;
   pthread_t tid;
   tid = pthread_self();
*/
}

void *dds_register_seq(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  int32 *index;
  index=control_program->parameters->pulseseq_index;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,REGISTER_SEQ,"register_seq","DDS");
  if (control_program!=NULL) {
    if (control_program->parameters!=NULL) {
      if (control_program->state!=NULL) {
        driver_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
        driver_msg_add_var(&s_msg,index,sizeof(index)*3,"index","int32 * 3");
        driver_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]],sizeof(struct TSGbuf),"pulseseq","struct TDGBuf");
        driver_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]]->rep,sizeof(unsigned char)*pulseseqs[index[0]][index[1]][index[2]]->len,"rep","array");
        driver_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]]->code,sizeof(unsigned char)*pulseseqs[index[0]][index[1]][index[2]]->len,"code","array");
      }
    }
  }
  pthread_mutex_lock(&dds_comm_lock);
  driver_msg_send(ddssock, &s_msg);
  driver_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);

}

void *dds_site_settings(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  struct SiteSettings *site_settings;
  site_settings=arg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,SITE_SETTINGS,"site_settings","NONE");
  if (site_settings!=NULL) {
    driver_msg_add_var(&s_msg,&site_settings->ifmode,sizeof(int32),"ifmode","int32");
    driver_msg_add_var(&s_msg,&site_settings->rf_settings,sizeof(struct RXFESettings),"rf_rxfe_settings","RFXESetting");
    driver_msg_add_var(&s_msg,&site_settings->if_settings,sizeof(struct RXFESettings),"if_rxfe_settings","RXFESetting");
  }
  pthread_mutex_lock(&dds_comm_lock);
  driver_msg_send(ddssock, &s_msg);
  driver_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *dds_get_trigger_offset(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,GET_TRIGGER_OFFSET,"get_trigger_offset","NONE");
  pthread_mutex_lock(&dds_comm_lock);
  if (control_program!=NULL) {
     if (control_program->parameters!=NULL) {
       driver_msg_add_var(&s_msg,&control_program->parameters->radar,sizeof(int32),"radar","int32");
       driver_msg_add_var(&s_msg,&control_program->parameters->channel,sizeof(int32),"channel","int32");
       driver_msg_send(ddssock, &s_msg);
       driver_msg_recv(ddssock, &r_msg);
     } 
  }
  pthread_mutex_unlock(&dds_comm_lock);
  driver_msg_get_var_by_name(&r_msg,"dds_trigger_offset_usec",&control_program->state->dds_trigger_offset_usec);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}
void *dds_ready_controlprogram(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,CtrlProg_READY,"ctrlprog_ready","NONE");
  pthread_mutex_lock(&dds_comm_lock);
  if (control_program!=NULL) {
     if (control_program->parameters!=NULL) {
       driver_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
       driver_msg_send(ddssock, &s_msg);
       driver_msg_recv(ddssock, &r_msg);
     } 
  }
  pthread_mutex_unlock(&dds_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *dds_end_controlprogram(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  pthread_mutex_lock(&dds_comm_lock);
  driver_msg_set_command(&s_msg,CtrlProg_END,"ctrlprog_end","NONE");
  driver_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
  driver_msg_send(ddssock, &s_msg);
  driver_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *dds_pretrigger(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,PRETRIGGER,"pretrigger","NONE");
  pthread_mutex_lock(&dds_comm_lock);
  driver_msg_send(ddssock, &s_msg);
  driver_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *dds_posttrigger(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,PRETRIGGER,"pretrigger","NONE");
  pthread_mutex_lock(&dds_comm_lock);
  driver_msg_send(ddssock, &s_msg);
  driver_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}


