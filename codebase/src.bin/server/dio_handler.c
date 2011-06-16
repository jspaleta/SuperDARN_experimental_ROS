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
  char entry[200];
  char secname_static[200];
  char *secname_in_dict;

  int32 nsecs,bytes;
  unsigned int bufsize;
  struct tx_status *txp=NULL;  
  void *dict_buf=NULL; // pointer into dictionary do not free
  void *temp_buf=NULL; // malloced buffer needs to be freed 
  int i,radar=1;
  pthread_mutex_lock(&dio_comm_lock);
  
  aux_dict=auxdata->aux_dict;
  printf("DIO AUX Pre Dict pointer: %p\n",aux_dict);
  iniparser_dump_ini(auxdata->aux_dict,stdout);

  dict_string=iniparser_to_string(aux_dict);
  bytes=strlen(dict_string)+1;
  s_msg.type=AUX_COMMAND;
  s_msg.status=1;
  r_msg.status=0;
  send_data(diosock, &s_msg, sizeof(struct DriverMsg));
  recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
  if(r_msg.status==1) {
    printf("DIO AUX Command is valid\n");
    send_data(diosock, &bytes, sizeof(int32));
    printf("%s",dict_string);
    send_data(diosock, dict_string, bytes*sizeof(char));
    /* Prepare to send arb. data buf */
    nsecs=iniparser_getnsec(aux_dict);
    send_data(diosock, &nsecs, sizeof(int32));
    for(i=0;i<nsecs;i++) {
      secname_in_dict=iniparser_getsecname(aux_dict,i);
      dict_buf=iniparser_getbuf(aux_dict,secname_in_dict,&bufsize);
      bytes=strlen(secname_in_dict)+1;
      send_data(diosock,&bytes,sizeof(int32));
      send_data(diosock,secname_in_dict,bytes);
      bytes=bufsize;
      send_data(diosock,dict_buf,bytes);
    }
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;

    printf("AUX dict and buffers sent %p\n",aux_dict);

    recv_data(diosock, &bytes, sizeof(int32));
    if(dict_string!=NULL) free(dict_string);
    dict_string=malloc(sizeof(char)*(bytes+10));
    sprintf(dict_string,"");
    recv_data(diosock, dict_string, bytes*sizeof(char));
    printf("%s",dict_string);
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;
    aux_dict=iniparser_load_from_string(NULL,dict_string);
    free(dict_string);
    /* Prepare to recv arb. buf data buf and place it into dict*/
    nsecs=0;
    recv_data(diosock,&nsecs,sizeof(int32));
    printf("DIO AUX Command nsecs %d\n",nsecs);
    for(i=0;i<nsecs;i++) {
      recv_data(diosock,&bytes,sizeof(int32));
      printf("%d :",bytes);
      recv_data(diosock,secname_static,bytes);
      printf(" %s :",secname_static);
      sprintf(entry,"%s:bytes",secname_static);
      bytes=iniparser_getint(aux_dict,entry,0);
      printf(" %d\n",bytes);
      if(temp_buf!=NULL) free(temp_buf);
      temp_buf=malloc(bytes);
      recv_data(diosock,temp_buf,bytes);
      iniparser_setbuf(aux_dict,secname_static,temp_buf,bytes);
      if(temp_buf!=NULL) free(temp_buf);
      temp_buf=NULL;
    }
    recv_data(diosock, &r_msg, sizeof(struct DriverMsg));
  } else {
    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=dictionary_new(0);
    printf("AUX Command is invalid\n");
  }
  printf("AUX POST Dict pointer: %p\n",aux_dict);
  auxdata->aux_dict=aux_dict;
  iniparser_dump_ini(auxdata->aux_dict,stdout);
  
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


