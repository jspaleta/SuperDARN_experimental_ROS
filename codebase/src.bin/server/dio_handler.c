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

struct FreqTable *FreqLoadTable(FILE *fp) {
  char line[1024];
  char *tkn;
  int i,j;
  int s,e,status;
  struct FreqTable *ptr;
  ptr=malloc(sizeof(struct FreqTable));
  if (ptr==NULL) return NULL;

  /*start scanning records from the file*/ 

  ptr->dfrq=DEFAULT_FREQ;
  ptr->num=0;
  ptr->start=NULL;
  ptr->end=NULL;
  while (fgets(line,1024,fp) !=0) {
    for (i=0; (line[i] !=0) && 
              ((line[i] ==' ') || (line[i]=='\n'));i++);

    /* ignore comments or empty lines */

    if ((line[i]==0) || (line[i]=='#')) continue;

    tkn=line+i; 
    if ((tkn[0]=='d') || (tkn[0]=='D')) { /* default frequency */
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

void *DIO_ready_controlprogram(struct ControlProgram *arg)
{
  struct DriverMsg s_msg,r_msg;
  pthread_mutex_lock(&dio_comm_lock);
   if (arg!=NULL) {
     if (arg->state->pulseseqs[arg->parameters->current_pulseseq_index]!=NULL) {
       s_msg.type=DIO_CtrlProg_READY;
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

   s_msg.type=DIO_PRETRIGGER;
   s_msg.status=1;
   send_data(diosock, &s_msg, sizeof(struct DriverMsg));
   recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *DIO_transmitter_status(int32 radar)
{
  struct DriverMsg s_msg,r_msg;
  dictionary *aux_dict=NULL;
  char *dict_string=NULL;
  char value[200];
  int32 bytes,i;
  char *data_string=NULL;
  struct tx_status *txp=NULL;  
  void *buf=NULL; // pointer into dictionary do not free
  pthread_mutex_lock(&dio_comm_lock);
  int32 temp_data=44; 
  
  aux_dict=dictionary_new(0);
  iniparser_set(aux_dict,"COMMAND",NULL);
  iniparser_set(aux_dict,"command","GET_TX_STATUS");
  iniparser_set(aux_dict,"DIO",NULL);
  sprintf(value,"%d",radar);
  iniparser_set(aux_dict,"DIO:radar",value);
  iniparser_set(aux_dict,"data",NULL);
  sprintf(value,"%d",sizeof(int32));
  iniparser_set(aux_dict,"data:bytes",value);
  dictionary_setbuf(aux_dict,"data",&temp_data,sizeof(int32));
  dict_string=iniparser_to_string(aux_dict);
  bytes=strlen(dict_string)+1;
/*
  s_msg.type=GET_TX_STATUS;
  s_msg.status=1;
  send_data(diosock, &s_msg, sizeof(struct DriverMsg));
  recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) {
    send_data(diosock, &radar, sizeof(radar));
    recv_data(diosock, &txstatus[radar-1], sizeof(struct tx_status));
    recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
  }
*/
  s_msg.type=AUX_COMMAND;
  s_msg.status=1;
  send_data(diosock, &s_msg, sizeof(struct DriverMsg));
  recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) {
    send_data(diosock, &bytes, sizeof(int32));
    send_data(diosock, dict_string, bytes*sizeof(char));
    if(iniparser_find_entry(aux_dict,"data")==1) {
      buf=dictionary_getbuf(aux_dict,"data",&bytes);
      printf("DIO: GET_TX_STATUS: %p %d\n",buf,bytes);
      send_data(diosock,buf,bytes);
    }
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;

    recv_data(diosock, &bytes, sizeof(int32));
    if(dict_string!=NULL) free(dict_string);
    dict_string=malloc(sizeof(char)*(bytes+10));
    recv_data(diosock, dict_string, bytes*sizeof(char));
    printf("String:\n%s",dict_string);
    recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;
    aux_dict=iniparser_load_from_string(aux_dict,dict_string);
    free(dict_string);
    printf("Dict:\n");
    iniparser_dump_ini(aux_dict,stdout);
    txp=malloc(iniparser_getint(aux_dict,"data_bytes",0));
    data_string=iniparser_getstring(aux_dict,"txstatus",NULL);
    memmove(txp,data_string,iniparser_getint(aux_dict,"data_bytes",0));
    iniparser_freedict(aux_dict);
  }
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
}


