#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "dio_handler.h"
#include "utils.h"

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
  struct DriverMsg msg;
  pthread_mutex_lock(&dio_comm_lock);
   if (arg!=NULL) {
     if (arg->state->pulseseqs[arg->parameters->current_pulseseq_index]!=NULL) {
       msg.type=DIO_CtrlProg_READY;
       msg.status=1;
       send_data(diosock, &msg, sizeof(struct DriverMsg));
       send_data(diosock, arg->parameters, sizeof(struct ControlPRM));
       recv_data(diosock, &msg, sizeof(struct DriverMsg));
     } 
   }
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *DIO_pretrigger(void *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&dio_comm_lock);

   msg.type=DIO_PRETRIGGER;
   msg.status=1;
   send_data(diosock, &msg, sizeof(struct DriverMsg));
   recv_data(diosock, &msg, sizeof(struct DriverMsg));
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *DIO_transmitter_status(int radar)
{
  struct DriverMsg msg;
  int tx;
  pthread_mutex_lock(&dio_comm_lock);
//  printf("DIO: TX Status Start\n");
  msg.type=DIO_GET_TX_STATUS;
  msg.status=1;
  send_data(diosock, &msg, sizeof(struct DriverMsg));
  send_data(diosock, &radar, sizeof(radar));
  recv_data(diosock, &txstatus[radar-1], sizeof(struct tx_status));
  recv_data(diosock, &msg, sizeof(struct DriverMsg));
//  printf("DIO: TX Status End\n");
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};


void *DIO_clrfreq(struct ControlProgram *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&dio_comm_lock);

   if(arg!=NULL) {
     if(arg->parameters!=NULL) {
       //printf("DIO_CLRFREQ %p %p %d\n",arg, arg->parameters,arg->parameters->tbeam);
       msg.type=DIO_CLRFREQ;
       msg.status=1;
       send_data(diosock, &msg, sizeof(struct DriverMsg));
       send_data(diosock, arg->parameters, sizeof(struct ControlPRM));
       recv_data(diosock, &msg, sizeof(struct DriverMsg));
     }
  }
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *DIO_rxfe_reset(void *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&dio_comm_lock);

   msg.type=DIO_RXFE_RESET;
   msg.status=1;
   send_data(diosock, &msg, sizeof(struct DriverMsg));
   recv_data(diosock, &msg, sizeof(struct DriverMsg));
   pthread_mutex_unlock(&dio_comm_lock);
   pthread_exit(NULL);
};

void *dio_rxfe_settings(void *arg)
{
  struct DriverMsg msg;
  struct SiteSettings *site_settings;

  site_settings=arg;
  pthread_mutex_lock(&dio_comm_lock);
  if (site_settings!=NULL) {
    msg.type=DIO_RXFE_SETTINGS;
    msg.status=1;
    send_data(diosock, &msg, sizeof(struct DriverMsg));
    send_data(diosock, &site_settings->ifmode, sizeof(site_settings->ifmode));
    send_data(diosock, &site_settings->rf_settings, sizeof(struct RXFESettings));
    send_data(diosock, &site_settings->if_settings, sizeof(struct RXFESettings));
    recv_data(diosock, &msg, sizeof(struct DriverMsg));
  }
  pthread_mutex_unlock(&dio_comm_lock);
}


