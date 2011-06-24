#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
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
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,CtrlProg_READY,"ctrlprog_ready","NONE");
  pthread_mutex_lock(&dio_comm_lock);
   if (arg!=NULL) {
     if (arg->parameters!=NULL) {
       driver_msg_add_var(&s_msg,arg->parameters,sizeof(struct ControlPRM),"parameters","ControlPRM");
       driver_msg_send(diosock, &s_msg);
       driver_msg_recv(diosock, &r_msg);
     } 
   }
   pthread_mutex_unlock(&dio_comm_lock);
   driver_msg_free_buffer(&s_msg);
   driver_msg_free_buffer(&r_msg);
   pthread_exit(NULL);
};

void *DIO_pretrigger(void *arg)
{
  struct DriverMsg s_msg, r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,PRETRIGGER,"pretrigger","NONE");
  pthread_mutex_lock(&dio_comm_lock);
  driver_msg_send(diosock, &s_msg);
  driver_msg_recv(diosock, &r_msg);
  pthread_mutex_unlock(&dio_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
};


void *DIO_aux_msg(struct DriverMsg *msg_p)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  pthread_mutex_lock(&dio_comm_lock);
  memmove(&s_msg,msg_p,sizeof(struct DriverMsg));
  driver_msg_send(diosock, &s_msg);
  driver_msg_recv(diosock, &r_msg);
  memmove(msg_p,&r_msg,sizeof(struct DriverMsg));
  pthread_mutex_unlock(&dio_comm_lock);
  pthread_exit(NULL);
};

void *DIO_pre_clrfreq(struct ControlProgram *arg)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,PRE_CLRFREQ,"pre_clrfreq","NONE");
  pthread_mutex_lock(&dio_comm_lock);
  if(arg!=NULL) {
     if(arg->parameters!=NULL) {
       driver_msg_send(diosock, &s_msg);
       driver_msg_recv(diosock, &r_msg);
     }
  }
  pthread_mutex_unlock(&dio_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
};

void *DIO_post_clrfreq(struct ControlProgram *arg)
{
  struct DriverMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  driver_msg_set_command(&s_msg,POST_CLRFREQ,"post_clrfreq","NONE");
  pthread_mutex_lock(&dio_comm_lock);
  if(arg!=NULL) {
     if(arg->parameters!=NULL) {
       driver_msg_send(diosock, &s_msg);
       driver_msg_recv(diosock, &r_msg);
     }
  }
  pthread_mutex_unlock(&dio_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);

};

void *dio_site_settings(void *arg)
{
  struct DriverMsg s_msg,r_msg;
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
  pthread_mutex_lock(&dio_comm_lock);
  driver_msg_send(diosock, &s_msg);
  driver_msg_recv(diosock, &r_msg);
  pthread_mutex_unlock(&dio_comm_lock);
  driver_msg_free_buffer(&s_msg);
  driver_msg_free_buffer(&r_msg);
  pthread_exit(NULL);
}


