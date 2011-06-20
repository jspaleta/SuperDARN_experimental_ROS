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
  pthread_mutex_lock(&dio_comm_lock);
   if (arg!=NULL) {
     if (arg->state->pulseseqs[arg->parameters->current_pulseseq_index]!=NULL) {
       s_msg.command_type=CtrlProg_READY;
       s_msg.status=1;
       send_data(diosock, &s_msg, sizeof(struct DriverMsg));
       recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
       if(r_msg.status==1) {
         send_data(diosock, arg->parameters, sizeof(struct ControlPRM));
         recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
       }
     } 
   }
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *DIO_pretrigger(void *arg)
{
  struct DriverMsg s_msg, r_msg;
  pthread_mutex_lock(&dio_comm_lock);

   s_msg.command_type=PRETRIGGER;
   s_msg.status=1;
   send_data(diosock, &s_msg, sizeof(struct DriverMsg));
   recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *DIO_aux_command(dictionary **dict_p)
{
  dictionary *aux_dict=NULL;
  struct DriverMsg s_msg,r_msg;
  char *dict_string=NULL;
  int32 bytes;

  pthread_mutex_lock(&dio_comm_lock);
  aux_dict=*dict_p;

  dict_string=iniparser_to_string(aux_dict);
  bytes=strlen(dict_string)+1;
  s_msg.command_type=AUX_COMMAND;
  s_msg.status=1;
  r_msg.status=0;
  send_data(diosock, &s_msg, sizeof(struct DriverMsg));
  recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) {
    send_aux_dict(diosock,aux_dict,0);
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;
    recv_aux_dict(diosock,&aux_dict,1);
    recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
  } else {
    fprintf(stderr,"DIO AUX Command is invalid\n");
  }
  //iniparser_dump(aux_dict,stderr);
  *dict_p=aux_dict;
  
  pthread_mutex_unlock(&dio_comm_lock);
  pthread_exit(NULL);
};



void *DIO_pre_clrfreq(struct ControlProgram *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&dio_comm_lock);

   if(arg!=NULL) {
     if(arg->parameters!=NULL) {
       s_msg.command_type=PRE_CLRFREQ;
       s_msg.status=1;
       send_data(diosock, &s_msg, sizeof(struct DriverMsg));
       recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
       if(r_msg.status==1) {
         send_data(diosock, arg->parameters, sizeof(struct ControlPRM));
         recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
       }
     }
  }
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *DIO_post_clrfreq(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&dio_comm_lock);

   s_msg.command_type=POST_CLRFREQ;
   s_msg.status=1;
   send_data(diosock, &s_msg, sizeof(struct DriverMsg));
   recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *dio_site_settings(void *arg)
{
  struct DriverMsg s_msg,r_msg;
  struct SiteSettings *site_settings;

  site_settings=arg;
  pthread_mutex_lock(&dio_comm_lock);
  if (site_settings!=NULL) {
    s_msg.command_type=SITE_SETTINGS;
    s_msg.status=1;
    send_data(diosock, &s_msg, sizeof(struct DriverMsg));
    recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
    if(r_msg.status==1) {
      send_data(diosock, &site_settings->ifmode, sizeof(site_settings->ifmode));
      send_data(diosock, &site_settings->rf_settings, sizeof(struct RXFESettings));
      send_data(diosock, &site_settings->if_settings, sizeof(struct RXFESettings));
      recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
    }
  }
  pthread_mutex_unlock(&dio_comm_lock);
  pthread_exit(NULL);
}


