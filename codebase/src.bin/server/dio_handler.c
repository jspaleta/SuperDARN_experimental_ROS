#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"
#include "dio_handler.h"
#include "utils.h"
#include "iniparser.h"

extern int diosock;
extern int verbose;
extern pthread_mutex_t dio_comm_lock;

void *DIO_ready_controlprogram(struct ControlProgram *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,CtrlProg_READY,"ctrlprog_ready","NONE");
  pthread_mutex_lock(&dio_comm_lock);
   if (arg!=NULL) {
     if (arg->parameters!=NULL) {
       ros_msg_add_var(&s_msg,arg->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
       ros_msg_send(diosock, &s_msg);
       ros_msg_recv(diosock, &r_msg);
     } 
   }
   pthread_mutex_unlock(&dio_comm_lock);
   ros_msg_free_buffer(&s_msg);
   ros_msg_free_buffer(&r_msg);
   pthread_exit(NULL);
};

void *DIO_pretrigger(void *arg)
{
  struct ROSMsg s_msg, r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,PRETRIGGER,"pretrigger","NONE");
  pthread_mutex_lock(&dio_comm_lock);
  ros_msg_send(diosock, &s_msg);
  ros_msg_recv(diosock, &r_msg);
  pthread_mutex_unlock(&dio_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
};


void *DIO_aux_msg(struct ROSMsg *msg_p)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  pthread_mutex_lock(&dio_comm_lock);
  memmove(&s_msg,msg_p,sizeof(struct ROSMsg));
  ros_msg_send(diosock, &s_msg);
  ros_msg_recv(diosock, &r_msg);
  memmove(msg_p,&r_msg,sizeof(struct ROSMsg));
  pthread_mutex_unlock(&dio_comm_lock);
  pthread_exit(NULL);
};

void *DIO_ready_clrsearch(struct ControlProgram *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,CLRSEARCH_READY,"clrsearch_ready","NONE");
  pthread_mutex_lock(&dio_comm_lock);
   if (arg!=NULL) {
     if (arg->parameters!=NULL) {
       ros_msg_add_var(&s_msg,&arg->clrfreqsearch,sizeof(struct CLRFreqPRM),"clrfreqsearch","struct CLRFreqRPM");
       ros_msg_add_var(&s_msg,&arg->parameters->radar,sizeof(int32),"radar","int32");
       ros_msg_add_var(&s_msg,&arg->parameters->channel,sizeof(int32),"channel","int32");
       ros_msg_send(diosock, &s_msg);
       ros_msg_recv(diosock, &r_msg);
     } 
   }
   pthread_mutex_unlock(&dio_comm_lock);
   ros_msg_free_buffer(&s_msg);
   ros_msg_free_buffer(&r_msg);
   pthread_exit(NULL);
};
void *DIO_pre_clrsearch(struct ControlProgram *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,PRE_CLRSEARCH,"pre_clrsearch","NONE");
  pthread_mutex_lock(&dio_comm_lock);
  if(arg!=NULL) {
     if(arg->parameters!=NULL) {
       ros_msg_send(diosock, &s_msg);
       ros_msg_recv(diosock, &r_msg);
     }
  }
  pthread_mutex_unlock(&dio_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
};

void *DIO_post_clrsearch(struct ControlProgram *arg)
{
  struct ROSMsg s_msg,r_msg;
  ros_msg_init(&s_msg);
  ros_msg_init(&r_msg);
  ros_msg_set_command(&s_msg,POST_CLRSEARCH,"post_clrsearch","NONE");
  pthread_mutex_lock(&dio_comm_lock);
  if(arg!=NULL) {
     if(arg->parameters!=NULL) {
       ros_msg_send(diosock, &s_msg);
       ros_msg_recv(diosock, &r_msg);
     }
  }
  pthread_mutex_unlock(&dio_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);

};

void *dio_site_settings(void *arg)
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
  pthread_mutex_lock(&dio_comm_lock);
  ros_msg_send(diosock, &s_msg);
  ros_msg_recv(diosock, &r_msg);
  pthread_mutex_unlock(&dio_comm_lock);
  ros_msg_free_buffer(&s_msg);
  ros_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}


