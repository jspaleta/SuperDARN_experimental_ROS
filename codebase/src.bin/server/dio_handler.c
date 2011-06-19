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
extern struct tx_status txstatus[MAX_RADARS];
/*
struct FreqTable *FreqLoadTable(FILE *fp) {
  char line[1024];
  char *tkn;
  int i,j;
  int s,e,status;
  struct FreqTable *ptr;
  ptr=malloc(sizeof(struct FreqTable));
  if (ptr==NULL) return NULL;

  //start scanning records from the file

  ptr->dfrq=DEFAULT_FREQ;
  ptr->num=0;
  ptr->start=NULL;
  ptr->end=NULL;
  while (fgets(line,1024,fp) !=0) {
    for (i=0; (line[i] !=0) && 
              ((line[i] ==' ') || (line[i]=='\n'));i++);

    // ignore comments or empty lines 

    if ((line[i]==0) || (line[i]=='#')) continue;

    tkn=line+i; 
    if ((tkn[0]=='d') || (tkn[0]=='D')) { // default frequency 
      for (j=0;(tkn[j] !='=') && (tkn[j] !=0);j++);
      if (tkn[j] !=0) {
        ptr->dfrq=atoi(tkn+j+1);
        if (ptr->dfrq==0) ptr->dfrq=DEFAULT_FREQ;
      }
      continue;      
    }
    status=sscanf(tkn,"%d %d",&s,&e);
    if (status==2) {
      if (ptr->start==NULL) ptr->start=malloc(sizeof(int));
      else ptr->start=realloc(ptr->start,sizeof(int)*(ptr->num+1));
      if (ptr->end==NULL) ptr->end=malloc(sizeof(int));
      else ptr->end=realloc(ptr->end,sizeof(int)*(ptr->num+1));
      ptr->start[ptr->num]=s;
      ptr->end[ptr->num]=e;
      ptr->num++;  
    }
 
  }
  return ptr;
}
*/

void *DIO_ready_controlprogram(struct ControlProgram *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&dio_comm_lock);
   if (arg!=NULL) {
     if (arg->state->pulseseqs[arg->parameters->current_pulseseq_index]!=NULL) {
       s_msg.type=CtrlProg_READY;
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

   s_msg.type=PRETRIGGER;
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
  s_msg.type=AUX_COMMAND;
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
  iniparser_dump(aux_dict,stdout);
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
       s_msg.type=PRE_CLRFREQ;
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

   s_msg.type=POST_CLRFREQ;
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
    s_msg.type=SITE_SETTINGS;
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


