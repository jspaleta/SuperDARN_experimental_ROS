#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"
#include "site_defaults.h"

extern int verbose;
extern int ddssock;
extern pthread_mutex_t dds_comm_lock;
extern struct SeqBuf *pulseseqs[MAX_RADARS+1][MAX_CHANNELS+1][MAX_SEQS+1];

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
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,REGISTER_SEQ,"register_seq","DDS");
  if (control_program!=NULL) {
    if (control_program->parameters!=NULL) {
      if (control_program->state!=NULL) {
        ros_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
        ros_msg_add_var(&s_msg,index,sizeof(index)*3,"index","int32 * 3");
        ros_msg_add_var(&s_msg,&pulseseqs[index[0]][index[1]][index[2]]->prm,sizeof(struct SeqPRM),"prm","struct SeqPRM");
        ros_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]]->rep,sizeof(unsigned char)*pulseseqs[index[0]][index[1]][index[2]]->prm.len,"rep","array");
        ros_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]]->code,sizeof(unsigned char)*pulseseqs[index[0]][index[1]][index[2]]->prm.len,"code","array");
        ros_msg_add_var(&s_msg,pulseseqs[index[0]][index[1]][index[2]]->ptab,sizeof(int)*pulseseqs[index[0]][index[1]][index[2]]->prm.mppul,"ptab","int array");
      }
    }
  }
  pthread_mutex_lock(&dds_comm_lock);
  ros_msg_send(ddssock, &s_msg);
  ros_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);

}

void *dds_site_settings(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  struct SiteSettings *site_settings;
  site_settings=arg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,SITE_SETTINGS,"site_settings","NONE");
  if (site_settings!=NULL) {
    ros_msg_add_var(&s_msg,&site_settings->ifmode,sizeof(int32),"ifmode","int32");
    ros_msg_add_var(&s_msg,&site_settings->rf_settings,sizeof(struct RXFESettings),"rf_rxfe_settings","RFXESetting");
    ros_msg_add_var(&s_msg,&site_settings->if_settings,sizeof(struct RXFESettings),"if_rxfe_settings","RXFESetting");
  }
  pthread_mutex_lock(&dds_comm_lock);
  ros_msg_send(ddssock, &s_msg);
  ros_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *dds_get_trigger_offset(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,GET_TRIGGER_OFFSET,"get_trigger_offset","NONE");
  pthread_mutex_lock(&dds_comm_lock);
  if (control_program!=NULL) {
     if (control_program->parameters!=NULL) {
       ros_msg_add_var(&s_msg,&control_program->parameters->radar,sizeof(int32),"radar","int32");
       ros_msg_add_var(&s_msg,&control_program->parameters->channel,sizeof(int32),"channel","int32");
       ros_msg_send(ddssock, &s_msg);
       ros_msg_recv(ddssock, &r_msg);
     } 
  }
  pthread_mutex_unlock(&dds_comm_lock);
  ros_msg_get_var_by_name(&r_msg,"dds_trigger_offset_usec",&control_program->state->dds_trigger_offset_usec);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}
void *dds_ready_controlprogram(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,CtrlProg_READY,"ctrlprog_ready","NONE");
  pthread_mutex_lock(&dds_comm_lock);
  if (control_program!=NULL) {
     if (control_program->parameters!=NULL) {
       ros_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
       ros_msg_send(ddssock, &s_msg);
       ros_msg_recv(ddssock, &r_msg);
     } 
  }
  pthread_mutex_unlock(&dds_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *dds_end_controlprogram(struct ControlProgram *control_program)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  pthread_mutex_lock(&dds_comm_lock);
  ros_msg_set_command(&s_msg,CtrlProg_END,"ctrlprog_end","NONE");
  ros_msg_add_var(&s_msg,control_program->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
  ros_msg_send(ddssock, &s_msg);
  ros_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *dds_pretrigger(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,PRETRIGGER,"pretrigger","NONE");
  pthread_mutex_lock(&dds_comm_lock);
  ros_msg_send(ddssock, &s_msg);
  ros_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}

void *dds_posttrigger(void *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,PRETRIGGER,"pretrigger","NONE");
  pthread_mutex_lock(&dds_comm_lock);
  ros_msg_send(ddssock, &s_msg);
  ros_msg_recv(ddssock, &r_msg);
  pthread_mutex_unlock(&dds_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}


