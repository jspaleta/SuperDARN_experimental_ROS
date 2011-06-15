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

void *DIO_aux_command(struct AUXdata *auxdata)
{
  struct DriverMsg s_msg,r_msg;
  dictionary *aux_dict=NULL;
  char *dict_string=NULL;
  char value[200];
  int32 bytes;
  unsigned int bufsize;
  char *data_string=NULL;
  struct tx_status *txp=NULL;  
  void *dict_buf=NULL; // pointer into dictionary do not free
  void *temp_buf=NULL; // malloced buffer needs to be freed 
  int i,radar=1;
  pthread_mutex_lock(&dio_comm_lock);
  
  aux_dict=auxdata->aux_dict;
  printf("AUX Pre Dict pointer: %p\n",aux_dict);
  dict_string=iniparser_to_string(aux_dict);
  bytes=strlen(dict_string)+1;
  s_msg.type=AUX_COMMAND;
  s_msg.status=1;
  send_data(diosock, &s_msg, sizeof(struct DriverMsg));
  recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) {
    printf("AUX Command is valid\n");
    send_data(diosock, &bytes, sizeof(int32));
    send_data(diosock, dict_string, bytes*sizeof(char));
    /* Prepare to send arb. data buf */
    if(iniparser_find_entry(aux_dict,"data")==1) {
      dict_buf=dictionary_getbuf(aux_dict,"data",&bufsize);
      bytes=bufsize;
      send_data(diosock,dict_buf,bytes);
    }
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;

    printf("AUX Command send %p\n",aux_dict);
    recv_data(diosock, &bytes, sizeof(int32));
    if(dict_string!=NULL) free(dict_string);
    dict_string=malloc(sizeof(char)*(bytes+10));
    recv_data(diosock, dict_string, bytes*sizeof(char));
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;
    aux_dict=iniparser_load_from_string(aux_dict,dict_string);
    free(dict_string);
    /* Prepare to recv arb. buf data buf and place it into dict*/
    if(iniparser_find_entry(aux_dict,"data")==1) {
      bytes=iniparser_getint(aux_dict,"data:bytes",0);
      if(temp_buf!=NULL) free(temp_buf);
      temp_buf=malloc(bytes);
      recv_data(diosock,temp_buf,bytes);
      dictionary_setbuf(aux_dict,"data",temp_buf,bytes);
      if(temp_buf!=NULL) free(temp_buf);
      temp_buf=NULL;
    }
    
    recv_data(diosock, &r_msg, sizeof(struct DriverMsg));

  }
  printf("AUX POST Dict pointer: %p\n",aux_dict);
  auxdata->aux_dict=aux_dict;
  pthread_mutex_unlock(&dio_comm_lock);
  pthread_exit(NULL);
};

void *DIO_transmitter_status(int32 radar) {
  dictionary *aux_dict=NULL;
  struct AUXdata auxdata;
  void *temp_buf=NULL; // malloced buffer needs to be freed 
  void *dict_buf=NULL; // pointer into dictionary do not free
  unsigned int bufsize;
  char value[200];
  int32 temp_data=44; 
  int i,rc;
  pthread_t thread;
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
  auxdata.aux_dict=aux_dict;
  printf("Pre Dict pointer: %p %p\n",aux_dict,auxdata.aux_dict);
  rc = pthread_create(&thread, NULL, (void *) &DIO_aux_command, &auxdata);
  pthread_join(thread,NULL);
  aux_dict=auxdata.aux_dict;
  printf("Post Dict pointer: %p %p\n",aux_dict,auxdata.aux_dict);
  /* Process Dictionary */ 
  printf("Dict:\n");
  iniparser_dump_ini(aux_dict,stdout);

  dict_buf=dictionary_getbuf(aux_dict,"data",&bufsize);
  printf("Data Bufsize %d tx_status size: %d\n",bufsize,sizeof(struct tx_status));
  printf("TX Status for radar %d\n",radar);
  memmove(&txstatus[radar-1],dict_buf,bufsize);
  for (i=0;i<MAX_TRANSMITTERS;i++) {
      printf("%d : %d %d %d\n",i,
        txstatus[radar-1].LOWPWR[i],
        txstatus[radar-1].AGC[i],
        txstatus[radar-1].status[i]);
  }

  if(aux_dict!=NULL) iniparser_freedict(aux_dict);
  aux_dict=NULL;
  pthread_exit(NULL);

}


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


