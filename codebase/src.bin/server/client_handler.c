
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/socket.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "utils.h"
#include "coordination_handler.h"
#include "dio_handler.h"
#include "reciever_handler.h"
#include "timing_handler.h"
#include "dds_handler.h"
#include "settings_handler.h"
#include "priority.h"
#include "rtypes.h"
#include "iniparser.h"

extern pthread_mutex_t controlprogram_list_lock,exit_lock;
extern void *radar_channels[MAX_RADARS*MAX_CHANNELS];
extern struct Thread_List_Item *controlprogram_threads;
extern struct TRTimes bad_transmit_times;
extern int verbose;
extern struct tx_status txstatus[MAX_RADARS];
extern struct SiteSettings site_settings;
extern dictionary *Site_INI;

int unregister_radar_channel(struct ControlProgram *control_program)
{
  int i,status;
  status=0;
  if (control_program!=NULL) {
    for(i=0;i<MAX_RADARS*MAX_CHANNELS;i++) {
      if (radar_channels[i]==(void *)control_program) {
        printf("Unregistering: %d :: cp: %p\n",i,control_program);
        status++;
        radar_channels[i]=NULL;
        control_program->parameters->radar=0;
        control_program->parameters->channel=0;
      }
    }
  }
  return status;
}

struct ControlProgram* find_registered_controlprogram_by_radar_channel(int radar,int channel)
{
  int i,r,c;
  struct ControlProgram *control_program;
  control_program=NULL;
  for(i=0;i<MAX_RADARS*MAX_CHANNELS;i++) {
    r=(i / MAX_CHANNELS)+1;
    c=(i % MAX_CHANNELS)+1;
    if (radar==r && channel==c) {
      control_program=radar_channels[i]; 
      break;
    }
  }
  return  control_program;
}

struct ControlPRM controlprogram_fill_parameters(struct ControlProgram *control_program)
{
  struct ControlPRM control_parameters;      
  struct ControlProgram *cp,*best[MAX_RADARS];      
  int priority=99; //Lowest priority wins-- its like golf
  int r,c;
  //printf("Fill parameters input %d\n",control_program->parameters->current_pulseseq_index);
  if (TX_BEAM_PRIORITY |RX_BEAM_PRIORITY |TX_FREQ_PRIORITY|RX_FREQ_PRIORITY|TIME_SEQ_PRIORITY) {  
    for (r=1;r<=MAX_RADARS;r++) {
      priority=99;
      best[r-1]=NULL; 
      for (c=1;c<=MAX_CHANNELS;c++) {
        cp=find_registered_controlprogram_by_radar_channel(r,c);
        if (cp!=NULL) {
          if (cp->active!=0) {
            if (cp->parameters!=NULL) {
              if (cp->parameters->priority<priority) {
                best[r-1]=cp;
                priority=cp->parameters->priority;
              }
            }
          }
        }
      }
    }
  }
  if (control_program!=NULL) {
    if (control_program->parameters!=NULL) {
       //strcpy(control_parameters.name,control_program->parameters->name);
       //strcpy(control_parameters.description,control_program->parameters->description);
       control_parameters.radar=control_program->radarinfo->radar;
       control_parameters.channel=control_program->radarinfo->channel;
       r=control_parameters.radar-1;
       control_parameters.current_pulseseq_index=control_program->parameters->current_pulseseq_index;
       control_parameters.priority=control_program->parameters->priority;
       if (TX_BEAM_PRIORITY) {
         control_parameters.tbeam=best[r]->parameters->tbeam;
         control_parameters.tbeamcode=best[r]->parameters->tbeamcode;
         control_parameters.tbeamwidth=best[r]->parameters->tbeamwidth;
         control_parameters.tbeamazm=best[r]->parameters->tbeamazm;
       } else {
         control_parameters.tbeam=control_program->parameters->tbeam;
         control_parameters.tbeamcode=control_program->parameters->tbeamcode;
         control_parameters.tbeamwidth=control_program->parameters->tbeamwidth;
         control_parameters.tbeamazm=control_program->parameters->tbeamazm;
       }
       if (TX_FREQ_PRIORITY) {
         control_parameters.tfreq=best[r]->parameters->tfreq;
         control_parameters.trise=best[r]->parameters->trise;
       } else {
         control_parameters.tfreq=control_program->parameters->tfreq;
         control_parameters.trise=control_program->parameters->trise;
       }
       if (RX_BEAM_PRIORITY) {
         control_parameters.rbeam=best[r]->parameters->rbeam;
         control_parameters.rbeamcode=best[r]->parameters->rbeamcode;
         control_parameters.rbeamwidth=best[r]->parameters->rbeamwidth;
         control_parameters.rbeamazm=best[r]->parameters->rbeamazm;
       } else {
         control_parameters.rbeam=control_program->parameters->rbeam;
         control_parameters.rbeamcode=control_program->parameters->rbeamcode;
         control_parameters.rbeamwidth=control_program->parameters->rbeamwidth;
         control_parameters.rbeamazm=control_program->parameters->rbeamazm;
       }
       if (RX_FREQ_PRIORITY) {
         control_parameters.rfreq=best[r]->parameters->rfreq;
         control_parameters.number_of_samples=best[r]->parameters->number_of_samples;
       } else {
         control_parameters.rfreq=control_program->parameters->rfreq;
         control_parameters.number_of_samples=control_program->parameters->number_of_samples;
       }
       control_parameters.buffer_index=control_program->parameters->buffer_index;
       control_parameters.baseband_samplerate=control_program->parameters->baseband_samplerate;
       control_parameters.filter_bandwidth=control_program->parameters->filter_bandwidth;
       control_parameters.match_filter=control_program->parameters->match_filter;
       control_parameters.status=control_program->parameters->status;
    } else {
    }
  } else {
  }
  //printf("Fill parameters output %d\n",control_parameters.current_pulseseq_index);
  return control_parameters;
}

struct ControlPRM* controlprogram_link_parameters(struct ControlPRM *control_parameters)
{
  return control_parameters;

}
struct ControlPRM* controlprogram_verify_parameters(struct ControlPRM *control_parameters)
{
/*
       control_parameters.name=control_program->parameters->name;
       control_parameters.description=control_program->parameters->description;
       control_parameters.radar=control_program->radarinfo->radar;
       control_parameters.channel=control_program->radarinfo->channel;
       control_parameters.current_pulseseq_index=control_program->parameters->current_pulseseq_index;
       control_parameters.priority=control_program->parameters->priority;
       control_parameters.tbeam=control_program->parameters->tbeam;
       control_parameters.tbeamcode=control_program->parameters->tbeamcode;
       control_parameters.tbeamwidth=control_program->parameters->tbeamwidth;
       control_parameters.tbeamazm=control_program->parameters->tbeamazm;
       control_parameters.tfreq=control_program->parameters->tfreq;
       control_parameters.trise=control_program->parameters->trise;
       control_parameters.rbeam=control_program->parameters->rbeam;
       control_parameters.rbeamcode=control_program->parameters->rbeamcode;
       control_parameters.rbeamwidth=control_program->parameters->rbeamwidth;
       control_parameters.rbeamazm=control_program->parameters->rbeamazm;
       control_parameters.rfreq=control_program->parameters->rfreq;
       control_parameters.number_of_samples=control_program->parameters->number_of_samples;
       control_parameters.buffer_index=control_program->parameters->buffer_index;
       control_parameters.baseband_samplerate=control_program->parameters->baseband_samplerate;
       control_parameters.filter_bandwidth=control_program->parameters->filter_bandwidth;
       control_parameters.match_filter=control_program->parameters->match_filter;
       control_parameters.status=control_program->parameters->status;
*/
  return control_parameters;
}

int register_radar_channel(struct ControlProgram *control_program,int radar,int channel)
{
  int i,r,c,status;
  if (control_program!=NULL) unregister_radar_channel(control_program);
  status=-1;
  for(i=0;i<MAX_RADARS*MAX_CHANNELS;i++) {
    r=(i / MAX_CHANNELS)+1;
    c=(i % MAX_CHANNELS)+1;
    if (radar_channels[i]==NULL) {
      if (radar<=0) radar=r;
      if (channel<=0) channel=c;
      if (radar==r && channel==c) {
        printf("Registering: %d :: radar: %d channel: %d cp: %p\n",i,radar,channel,control_program);
        status=1;
        control_program->parameters->radar=radar; 
        control_program->parameters->channel=channel; 
        control_program->radarinfo->radar=radar; 
        control_program->radarinfo->channel=channel; 
        radar_channels[i]=control_program;
        break;
      }
    }
  }
  return status;
}

struct ControlProgram *control_init() {
       int i;
       struct ControlProgram *control_program;

       control_program=malloc(sizeof(struct ControlProgram));
       control_program->active=1;
       control_program->clrfreqsearch.start=0;
       control_program->clrfreqsearch.end=0;
       control_program->parameters=malloc(sizeof(struct ControlPRM));
       control_program->state=malloc(sizeof(struct ControlState));
       control_program->radarinfo=malloc(sizeof(struct RadarPRM));
       control_program->data=malloc(sizeof(struct DataPRM));
       control_program->main=NULL;
       control_program->back=NULL;
       control_program->main_address=NULL;
       control_program->back_address=NULL;
       strcpy(control_program->parameters->name,"Generic Control Program Name - 80");
       strcpy(control_program->parameters->description,"Generic  Control Program  Description - 120");
       control_program->parameters->radar=-1;
       control_program->parameters->channel=-1;
       control_program->parameters->current_pulseseq_index=-1;
       control_program->parameters->priority=50;
       control_program->parameters->tbeam=-1;
       control_program->parameters->tbeamcode=-1;
       control_program->parameters->tbeamwidth=-1;
       control_program->parameters->tbeamazm=-1;
       control_program->parameters->tfreq=-1;
       control_program->parameters->trise=10;
       control_program->parameters->rbeam=-1;
       control_program->parameters->rbeamcode=-1;
       control_program->parameters->rbeamwidth=-1;
       control_program->parameters->rbeamazm=-1;
       control_program->parameters->rfreq=-1;
       control_program->parameters->number_of_samples=-1;
       control_program->parameters->buffer_index=-1;
       control_program->parameters->baseband_samplerate=-1;
       control_program->parameters->filter_bandwidth=-1;
       control_program->parameters->match_filter=-1;
       control_program->parameters->status=-1;


//       control_program->parameters->phased=-1;
//       control_program->parameters->filter=-1;
//       control_program->parameters->gain=-1;
//       control_program->parameters->seq_no=-1;
//       control_program->parameters->seq_id=-1;
//       control_program->parameters->fstatus=-1;
//       control_program->parameters->center_freq=-1;

       control_program->state->cancelled=0;
       control_program->state->ready=0;
       control_program->state->linked=0;
       control_program->state->processing=0;
       control_program->state->best_assigned_freq=0;
       control_program->state->current_assigned_freq=0;
       control_program->state->freq_change_needed=0;
       control_program->state->thread=NULL;
       control_program->state->fft_array=NULL;
       control_program->radarinfo->site=-1;
       control_program->radarinfo->radar=-1;
       control_program->radarinfo->channel=-1;

       for (i=0;i<MAX_SEQS;i++) {
         control_program->state->pulseseqs[i]=NULL;
       }
       return control_program;
}

void controlprogram_exit(struct ControlProgram *control_program)
{
   pthread_t tid;
   pthread_t thread;
   int i,rc;
   int socket;
   struct ControlProgram *linker_program,*data;
   struct Thread_List_Item *thread_list,*thread_item,*thread_next,*thread_prev;
   pthread_t threads[10];
   if(control_program!=NULL) {
     fprintf(stderr,"Client: Exit Command\n");
     control_program->state->cancelled=1;
     pthread_mutex_lock(&exit_lock);
     thread_list=controlprogram_threads;
     tid = pthread_self();
     control_program->active=0;
     printf("Controlprogram Exit: run coordinator\n"); 
     rc = pthread_create(&thread, NULL, (void *)&coordination_handler,(void *) control_program);
     pthread_join(thread,NULL);
     printf("Controlprogram Exit: done coordinator\n"); 
     i=0;
     printf("Controlprogram Exit: timing wait\n"); 
     rc = pthread_create(&threads[i], NULL,(void *) &timing_wait, NULL);
     pthread_join(threads[0],NULL);
     printf("Controlprogram Exit: done timing wait\n"); 
     i=0;
     printf("Controlprogram Exit: DDS End\n"); 
     rc = pthread_create(&threads[i], NULL, (void *) &dds_end_controlprogram, (void *) control_program);
       pthread_join(threads[i],NULL);
     i++;
     printf("Controlprogram Exit: TIMIN END\n"); 
     rc = pthread_create(&threads[i], NULL, (void *) &timing_end_controlprogram, (void *) control_program);
       pthread_join(threads[i],NULL);
     i++;
     printf("Controlprogram Exit: RECV END\n"); 
     rc = pthread_create(&threads[i], NULL, (void *) &receiver_end_controlprogram, (void *) control_program);
       pthread_join(threads[i],NULL);
     for (;i>=0;i--) {
       pthread_join(threads[i],NULL);
     }
     printf("Closing Client Socket: %d\n",control_program->state->socket);
     close(control_program->state->socket);
     unregister_radar_channel(control_program);
/*     if (verbose>0) printf("Checking for programs linked to %p\n",control_program); 
     while (thread_list!=NULL) {
       linker_program=thread_list->data;
       if (verbose>0) printf("%p  %p %p\n",linker_program,linker_program->state->linked_program,control_program); 
       if (linker_program->state->linked_program==control_program) {
         if (verbose>0) printf("Found linked program %p that need to also die\n",linker_program); 
         pthread_cancel(thread_list->id);       
         pthread_join(thread_list->id,NULL);
         thread_item=thread_list;   
         thread_next=thread_item->next;
         thread_list=thread_item->prev;
         if (thread_next != NULL) thread_next->prev=thread_list;
         else controlprogram_threads=thread_list;
         if (thread_list != NULL) thread_list->next=thread_item->next;
         if(thread_item!=NULL) {
           free(thread_item);
           thread_item=NULL; 
         }
      } else {
         thread_list=thread_list->prev;
       }
     }
*/
     if (verbose>1) fprintf(stderr,"Client Exit: Freeing internal structures for %p\n",control_program); 
     for (i=0;i<MAX_SEQS;i++) {
       if(control_program->state->pulseseqs[i]!=NULL) TSGFree(control_program->state->pulseseqs[i]);
     }
     if (verbose>1) fprintf(stderr,"Client Exit: Freeing controlprogram state %p\n",control_program->state); 
     if(control_program->state!=NULL) {
       if(control_program->state->fft_array!=NULL) {
         free(control_program->state->fft_array);
         control_program->state->fft_array=NULL;
       }
       free(control_program->state);
       control_program->state=NULL;
     }
     if (verbose>1) fprintf(stderr,"Client Exit: Freed controlprogram state %p\n",control_program->state);
     if (verbose>1) fprintf(stderr,"Client Exit: Freeing controlprogram parameters %p ",control_program->parameters); 
     if(control_program->parameters!=NULL) {
       free(control_program->parameters);
       control_program->parameters=NULL;
     }
     if (verbose>1) fprintf(stderr," %p\n",control_program->parameters); 
     if (verbose>1) fprintf(stderr,"Client Exit: Freeing controlprogram data %p\n",control_program->data); 
     if(control_program->main!=NULL) munmap(control_program->main);
     if(control_program->back!=NULL) munmap(control_program->back);
     control_program->main=NULL;
     control_program->back=NULL;
     control_program->main_address=NULL;
     control_program->back_address=NULL;
     if(control_program->data!=NULL) {
       free(control_program->data);
       control_program->data=NULL;
     }
     control_program->active=0;
     if (verbose>1) fprintf(stderr,"Client Exit: Done with control program %p\n",control_program); 
   }
   if (verbose>1) fprintf(stderr,"Client Exit: Waiting on Coordination thread\n");
   if (verbose>1) fprintf(stderr,"Client Exit: Done Waiting\n");
   pthread_mutex_unlock(&exit_lock);
   //printf("Client Exit: List UnLock\n");
   //pthread_mutex_unlock(&controlprogram_list_lock);
   if (verbose>1) fprintf(stderr,"Client Exit: Done\n");
}

void *control_handler(struct ControlProgram *control_program)
{
   int tid,i,r=-1,c=-1,status,rc,tmp;
   fd_set rfds;
   char command;
   int retval,socket,oldv,socket_err,length=sizeof(int);
   int32 current_freq,radar=0,channel=0;
   struct timeval tv,current_time,last_report;
   struct ROSMsg msg; 
   struct DriverMsg dmsg; 
   struct DataPRM control_data; 
   struct ControlPRM control_parameters; 
   struct SiteSettings settings;
   struct TSGbuf *pulseseq;
//   struct TSGprm *tsgprm;
   struct SeqPRM tprm;
   int data_int;
   pthread_t thread,threads[10];
   struct timeval t0,t1,t2,t3,t4;
   unsigned long elapsed;
   unsigned long ultemp;
   int32 data_length;
   char entry_type,entry_name[80];
   int return_type,entry_exists;
   char *temp_strp;
   int32 temp_int32;
   double temp_double;
   int32 bytes;
   unsigned int bufsize;
   char *command_dict_string=NULL;
   dictionary *aux_dict=NULL;
   void *buf=NULL; // This is malloced and needs to be freed
   void *temp_buf=NULL; // This is malloced and needs to be freed
   void *dict_data_buf=NULL; // This is a pointer into a dict.. do not free or realloc 
   int32 nsecs;
   char  *secname_in_dict=NULL;
   char  secname_static[200];
   char  entry[200];
/*
*  Init the Control Program state
*/
   pthread_mutex_lock(&exit_lock);
   pthread_mutex_lock(&controlprogram_list_lock);

   setbuf(stdout, 0);
   setbuf(stderr, 0);
   tid = pthread_self();
/* set the cancellation parameters --
   - Enable thread cancellation 
   - Defer the action of the cancellation 
*/
   pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
   pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
   pthread_cleanup_push((void *)&controlprogram_exit,(void *)control_program);
     if(control_program!=NULL) {
       socket=control_program->state->socket;
     }
   pthread_mutex_unlock(&controlprogram_list_lock);
   pthread_mutex_unlock(&exit_lock);

   gettimeofday(&last_report,NULL);
   while (1) {
      if(control_program==NULL) {
        break;
      }
      retval=getsockopt(socket, SOL_SOCKET, SO_ERROR, &socket_err, &length);
      if ((retval!=0) || (socket_err!=0)) {
            printf("Error: socket error: %d : %d %d\n",socket,retval,socket_err);
        break;
      }
      /* Look for messages from external controlprogram process */
      FD_ZERO(&rfds);
      FD_SET(socket, &rfds);
           /* Wait up to five seconds. */
      tv.tv_sec = 5;
      tv.tv_usec = 0;
      retval = select(socket+1, &rfds, NULL, NULL, &tv);
           /* Don’t rely on the value of tv now! */
      if (retval == -1) perror("select()");
      else if (retval) {
        pthread_mutex_lock(&controlprogram_list_lock);
        r=control_program->radarinfo->radar-1;
        c=control_program->radarinfo->channel-1;
        if ((r<0) || (c<0)) control_program->data->status=-1;
 
       /* Read controlprogram msg */
        msg.status=-300;
        msg.type=0;
        recv_data(socket, &msg, sizeof(struct ROSMsg));
        gettimeofday(&current_time,NULL);
        if((current_time.tv_sec-last_report.tv_sec)>5) {
#ifdef __QNX__
          system("date -t > /tmp/server_cmd_time");
#else
          system("date +'%s' > /tmp/server_cmd_time");
#endif
          last_report=current_time;
        }
        if(msg.type!=0) {
          control_program->state->thread->last_seen=current_time;
        }
        pthread_mutex_unlock(&controlprogram_list_lock);
        /* Process controlprogram msg */
        switch(msg.type) {
          case 0:
            printf("Bad Message Sleeping 1 second\n");
            sleep(1);
            break;
	  case AUX_COMMAND:
            /* AUX_COMMAND: Site hardware specific commands which are not critical for operation, but
             *  controlprograms may optionally access to if they are site aware.
             */
            if (verbose > 1 ) printf("Client: AUX_COMMAND\n");
             /* Inform the ROS that this driver does not handle this command by sending 
              * msg back with msg.status=0.
              */
            msg.status=1;
            if(msg.status==1) {
                send_data(socket, &msg, sizeof(struct ROSMsg));
/*
                // Prepare to recv command dict and data buf  
                recv_data(socket, &bytes, sizeof(int32));
                if (verbose > 1 ) printf("  AUX_COMMAND: recv dict bytes %d\n",bytes);
                if(command_dict_string!=NULL) {
                  free(command_dict_string);
                  command_dict_string=NULL;
                }
                if(auxdata.aux_dict!=NULL) {
                  iniparser_freedict(auxdata.aux_dict);
                  auxdata.aux_dict=NULL;
                }
                command_dict_string=malloc((bytes+10)*sizeof(char));
                recv_data(socket,command_dict_string,bytes);
                auxdata.aux_dict=iniparser_load_from_string(NULL,command_dict_string);
                // Prepare to recv arb. buf data buf 
                recv_data(socket, &nsecs, sizeof(int32));
                for(i=0 ; i<nsecs;i++) {
                  recv_data(socket,&bytes,sizeof(int32));
                  recv_data(socket,secname_static,bytes);
                  sprintf(entry,"%s:bytes",secname_static);
                  bytes=iniparser_getint(auxdata.aux_dict,entry,0);
                  if(temp_buf!=NULL) free(temp_buf);
                  temp_buf=malloc(bytes);
                  recv_data(socket,temp_buf,bytes);
                  iniparser_setbuf(auxdata.aux_dict,secname_static,temp_buf,bytes);
                  if(temp_buf!=NULL) free(temp_buf);
                  temp_buf=NULL;
                }
*/
                if(aux_dict!=NULL) iniparser_freedict(aux_dict);
                recv_aux_dict(socket,&aux_dict,0);
                iniparser_dump_ini(aux_dict,stdout);
                /* process aux command dictionary here */
                process_aux_commands(&aux_dict,"DIO");
                send_aux_dict(socket,aux_dict,1);

                /* Prepare to send return dict and data buf */
/*
                if(command_dict_string!=NULL) free(command_dict_string);
                command_dict_string=iniparser_to_string(auxdata.aux_dict);
                printf("Client: AUX dict_string %p\n",command_dict_string);
                bytes=strlen(command_dict_string)+1;
                printf("Client: AUX bytes %d\n",bytes);
                send_data(socket, &bytes, sizeof(int32));
                send_data(socket,command_dict_string,bytes);
                nsecs=iniparser_getnsec(auxdata.aux_dict); 
                send_data(socket, &nsecs, sizeof(int32));
                for(i=0 ; i<nsecs;i++) {
                  secname_in_dict=iniparser_getsecname(auxdata.aux_dict,i);
                  dict_data_buf=iniparser_getbuf(auxdata.aux_dict,secname_in_dict,&bufsize);
                  bytes=strlen(secname_in_dict)+1;

                  send_data(socket,&bytes,sizeof(int32));
                  send_data(socket,secname_in_dict,bytes);
                  bytes=bufsize;
                  send_data(socket,dict_data_buf,bytes);
                }
                if(auxdata.aux_dict!=NULL) iniparser_freedict(auxdata.aux_dict);
                auxdata.aux_dict=NULL;
*/
            }
            send_data(socket, &msg, sizeof(struct DriverMsg));
            break;

          case PING:
            //printf("PING: START\n");
            gettimeofday(&t0,NULL);
            msg.status=1;
            send_data(socket, &msg, sizeof(struct ROSMsg));
            //printf("PING: END\n");
            break;
          case SET_INACTIVE:
//            printf("SET_RADAR_INACTIVE\n");
            gettimeofday(&t0,NULL);
            if ( (r < 0) || (c < 0)) {
              msg.status=-1;
              send_data(socket, &msg, sizeof(struct ROSMsg));
            } else {
              pthread_mutex_lock(&controlprogram_list_lock);
              if(control_program->active!=0) {
                control_program->active=-1;
                control_program->state->ready=0;
                printf("Controlprogram Inactive: run coordinator\n"); 
                rc = pthread_create(&thread, NULL, (void *)&coordination_handler,(void *)
control_program);
                pthread_join(thread,NULL);
              }
              pthread_mutex_unlock(&controlprogram_list_lock);
              msg.status=1;
              send_data(socket, &msg, sizeof(struct ROSMsg));
            }
//            printf("end SET_RADAR_INACTIVE\n");
            break;
          case SET_ACTIVE:
//            printf("SET_RADAR_ACTIVE\n");
            gettimeofday(&t0,NULL);
            if ( (r < 0) || (c < 0)) {
              msg.status=-1;
              send_data(socket, &msg, sizeof(struct ROSMsg));
            } else {
              pthread_mutex_lock(&controlprogram_list_lock);
              if(control_program->active!=0) {
                control_program->active=1;
                control_program->state->ready=0;
                printf("Controlprogram active: run coordinator\n"); 
                rc = pthread_create(&thread, NULL, (void *)&coordination_handler,(void *)
control_program);
                pthread_join(thread,NULL);
              }
              pthread_mutex_unlock(&controlprogram_list_lock);
              msg.status=1;
              send_data(socket, &msg, sizeof(struct ROSMsg));
            }
//            printf("end SET_RADAR_ACTIVE\n");
            break;
/*
          case UPDATE_SITE_SETTINGS:
            gettimeofday(&t0,NULL);
            SettingsInit(&settings);
            recv_data(socket, &settings, sizeof(struct SiteSettings));
            msg.status=-1;
            send_data(socket, &msg, sizeof(struct ROSMsg));
            SettingsCpy(&settings,&site_settings);
            rc = pthread_create(&thread, NULL, (void *)&settings_rxfe_update_rf,(void *)&site_settings.rf_settings);
            pthread_join(thread,NULL);
            rc = pthread_create(&thread, NULL, (void *)&settings_rxfe_update_if,(void *)&site_settings.if_settings);
            pthread_join(thread,NULL);
            break;
*/
          case QUERY_INI_SETTING:
            printf("start QUERY_INI_SETTING\n");
            recv_data(socket, &data_length, sizeof(int32));
            recv_data(socket, &entry_name, data_length*sizeof(char));
            recv_data(socket, &entry_type, sizeof(char));
            entry_exists=iniparser_find_entry(Site_INI,entry_name);
            printf(" entry_type: %c  entry_name: \"%s\"  exists: %d\n",entry_type,entry_name,entry_exists);
            msg.status=entry_exists;
            switch(entry_type) {
              case 'i':
                return_type='i';
                temp_int32=iniparser_getint(Site_INI,entry_name,-1);
                send_data(socket, &return_type, sizeof(char));
                data_length=1;
                send_data(socket, &data_length, sizeof(int32));
                send_data(socket, &temp_int32, data_length*sizeof(int32));
                break;
              case 'b':
                return_type='b';
                temp_int32=iniparser_getboolean(Site_INI,entry_name,-1);
                send_data(socket, &return_type, sizeof(char));
                data_length=1;
                send_data(socket, &data_length, sizeof(int32));
                send_data(socket, &temp_int32, data_length*sizeof(int32));
                break;
              case 's':
                return_type='s';
                temp_strp=iniparser_getstring(Site_INI,entry_name,NULL);
                send_data(socket, &return_type, sizeof(char));
                data_length=strlen(temp_strp)+1;
                send_data(socket, &data_length, sizeof(int32));
                send_data(socket, temp_strp, data_length*sizeof(char));
                break;
              default:
                return_type=' ';
                send_data(socket, &return_type, sizeof(char));
                data_length=0;
                send_data(socket, &data_length, sizeof(int32));
                send_data(socket, temp_strp, data_length*sizeof(char));
            }
            send_data(socket, &msg, sizeof(struct ROSMsg));
            printf("end QUERY_INI_SETTING\n");
            break;
          case GET_SITE_SETTINGS:
            gettimeofday(&t0,NULL);
            settings=site_settings;
            send_data(socket, &settings, sizeof(struct SiteSettings));
            msg.status=-1;
            send_data(socket, &msg, sizeof(struct ROSMsg));
            break;
          case SET_SITE_IFMODE:
            gettimeofday(&t0,NULL);
            settings=site_settings;
            recv_data(socket, &settings.ifmode, sizeof(settings.ifmode));
            msg.status=-1;
            send_data(socket, &msg, sizeof(struct ROSMsg));
            break;
          case SET_RADAR_CHAN:
              //printf("SET_RADAR_CHAN\n");
            gettimeofday(&t0,NULL);
              msg.status=1;
              recv_data(socket, &radar, sizeof(int32)); //requested radar
              recv_data(socket, &channel, sizeof(int32)); //requested channel
              //printf("Radar: %d Chan: %d\n",radar,channel);
              pthread_mutex_lock(&controlprogram_list_lock);
              status=register_radar_channel(control_program,radar,channel);
              if (status) {
              }
              else {
                if (verbose>-1) fprintf(stderr,"Control Program thread %p Bad status %d no radar channel registered\n", tid,status);
              }
              msg.status=status;
              pthread_mutex_unlock(&controlprogram_list_lock);
              send_data(socket, &msg, sizeof(struct ROSMsg));
              //printf(" END SET_RADAR_CHAN\n");
            break;
          case LINK_RADAR_CHAN:
            gettimeofday(&t0,NULL);
            msg.status=1;
            recv_data(socket, &r, sizeof(r)); //requested radar
            recv_data(socket, &c, sizeof(c)); //requested channel
            pthread_mutex_lock(&controlprogram_list_lock);
            control_program->state->linked_program=find_registered_controlprogram_by_radar_channel(r,c);
            control_program->state->linked=1;
            if (control_program->state->linked_program!=NULL) {
              status=1;
            }
            else {
              status=0;
            }
            msg.status=status;
            pthread_mutex_unlock(&controlprogram_list_lock);
            send_data(socket, &msg, sizeof(struct ROSMsg));
            break;
          case GET_PARAMETERS:
            //printf("GET_PARAMETERS: START %d\n", control_program->parameters->current_pulseseq_index);
            gettimeofday(&t0,NULL);
            if ( (r < 0) || (c < 0)) {
              send_data(socket, &control_parameters, sizeof(struct ControlPRM));
              msg.status=-1;
              send_data(socket, &msg, sizeof(struct ROSMsg));
            } else {
              pthread_mutex_lock(&controlprogram_list_lock);
              msg.status=status;
              control_parameters=controlprogram_fill_parameters(control_program);
              pthread_mutex_unlock(&controlprogram_list_lock);
              send_data(socket, &control_parameters, sizeof(struct ControlPRM));
              send_data(socket, &msg, sizeof(struct ROSMsg));
            }
            //printf("GET_PARAMETERS: END %d\n", control_program->parameters->current_pulseseq_index);
            break;
          case GET_DATA:
            //printf("GET_DATA: START %d\n", control_program->parameters->current_pulseseq_index);
            gettimeofday(&t0,NULL);
            if ( (r < 0) || (c < 0)) {
              control_program->data->status=-1;
              //printf("GET_DATA: Bad Radar Channel\n");
              //printf("GET_DATA: Send DataPRM to controlprogram\n");
              send_data(socket, control_program->data, sizeof(struct DataPRM));
              msg.status=-1;
              send_data(socket, &msg, sizeof(struct ROSMsg));
            } else {
              //printf("GET_DATA: Start get_data thread\n");
              msg.status=status;
              rc = pthread_create(&thread, NULL,(void *)&receiver_controlprogram_get_data,(void *) control_program);
              pthread_join(thread,NULL);
              //printf("GET_DATA: Finish get_data thread\n");
//JDS: TODO do some GPS timestamp checking here
              pthread_mutex_lock(&controlprogram_list_lock);
              control_program->data->event_secs=control_program->state->gpssecond;
              control_program->data->event_nsecs=control_program->state->gpsnsecond;
              //printf("GET_DATA: Send DataPRM to controlprogram\n");
              //printf("GET_DATA: controlprogram status: %d\n",control_program->data->status);
              send_data(socket, control_program->data, sizeof(struct DataPRM));
              if(control_program->data->status>0) {
                send_data(socket, control_program->main, sizeof(uint32)*control_program->data->samples);
                send_data(socket, control_program->back, sizeof(uint32)*control_program->data->samples);
                send_data(socket, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
                send_data(socket, bad_transmit_times.start_usec, sizeof(uint32)*bad_transmit_times.length);
                send_data(socket, bad_transmit_times.duration_usec, sizeof(uint32)*bad_transmit_times.length);
                //temp_int32=MAX_TRANSMITTERS;
                //send_data(socket,&temp_int32,sizeof(int32));
                //send_data(socket,&txstatus[r].AGC,sizeof(int32)*temp_int32);
                //send_data(socket,&txstatus[r].LOWPWR,sizeof(int32)*temp_int32);
              } else {
                printf("CLIENT:GET_DATA: Bad status %d\n",control_program->data->status);
              } 
              //printf("GET_DATA:Send return message\n");
              send_data(socket, &msg, sizeof(struct ROSMsg));
              pthread_mutex_unlock(&controlprogram_list_lock);
            }
            gettimeofday(&t1,NULL);
            if (verbose > 1) {
              elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
              elapsed+=(t1.tv_usec-t0.tv_usec);
              if (verbose > 1 ) printf("Client:  Get Data Elapsed Microseconds: %ld\n",elapsed);
            }
            //printf("GET_DATA: END %d\n", control_program->parameters->current_pulseseq_index);
            break;
          case SET_PARAMETERS:
            //printf("SET_PARAMETERS: START %d\n", control_program->parameters->current_pulseseq_index);
            gettimeofday(&t0,NULL);
            if ( (r < 0) || (c < 0)) {
              recv_data(socket, control_program->parameters, sizeof(struct ControlPRM));
              msg.status=-1;
              send_data(socket, &msg, sizeof(struct ROSMsg));
            } else {
              msg.status=1;
              pthread_mutex_lock(&controlprogram_list_lock);
              recv_data(socket, control_program->parameters, sizeof(struct ControlPRM));
              //printf("SET_PARAMETERS: recv'd parameters %d\n", control_program->parameters->current_pulseseq_index);
              if(control_program->parameters->rfreq<0) control_program->parameters->rfreq=control_program->parameters->tfreq;
              send_data(socket, &msg, sizeof(struct ROSMsg));
              pthread_mutex_unlock(&controlprogram_list_lock);
            }
            //printf("SET_PARAMETERS: END %d\n", control_program->parameters->current_pulseseq_index);
            break;
          case REGISTER_SEQ:
            printf("REGISTER_SEQ: START\n");
            gettimeofday(&t0,NULL);
            msg.status=1;
            recv_data(socket,&tprm, sizeof(struct SeqPRM)); // requested pulseseq
            pthread_mutex_lock(&controlprogram_list_lock);
            control_program->state->pulseseqs[tprm.index]=malloc(sizeof(struct TSGbuf));
            control_program->parameters->current_pulseseq_index=tprm.index;
            control_program->state->pulseseqs[tprm.index]->len=tprm.len;
            control_program->state->pulseseqs[tprm.index]->step=tprm.step;
            control_program->state->pulseseqs[tprm.index]->index=tprm.index;
            control_program->state->pulseseqs[tprm.index]->prm=NULL;
            control_program->state->pulseseqs[tprm.index]->rep=
                malloc(sizeof(unsigned char)*control_program->state->pulseseqs[tprm.index]->len);
            control_program->state->pulseseqs[tprm.index]->code=
                malloc(sizeof(unsigned char)*control_program->state->pulseseqs[tprm.index]->len);
            control_program->state->pulseseqs[tprm.index]->prm=malloc(sizeof(struct TSGprm));
            recv_data(socket,control_program->state->pulseseqs[tprm.index]->rep, 
                sizeof(unsigned char)*control_program->state->pulseseqs[tprm.index]->len); // requested pulseseq
            recv_data(socket,control_program->state->pulseseqs[tprm.index]->code, 
                sizeof(unsigned char)*control_program->state->pulseseqs[tprm.index]->len); // requested pulseseq
            if ( (r < 0) || (c < 0)) {
              msg.status=-1;
            } else {
            //send on to timing socket
              rc = pthread_create(&threads[0], NULL, (void *)&timing_register_seq,(void *) control_program);
            //send on to dds socket
              rc = pthread_create(&threads[1], NULL, (void *)&dds_register_seq,(void *) control_program);
              printf("Waiting on Timing Thread\n");
              pthread_join(threads[0],NULL);
              printf("Waiting on DDS\n"); 
              pthread_join(threads[1],NULL);
            }
            pthread_mutex_unlock(&controlprogram_list_lock);
            printf("REGISTER_SEQ: SEND ROSMsg\n");
            send_data(socket, &msg, sizeof(struct ROSMsg));
            gettimeofday(&t1,NULL);
            if (verbose > 1) {
              elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
              elapsed+=(t1.tv_usec-t0.tv_usec);
              if (verbose > 1 ) printf("Client:  Reg Seq Elapsed Microseconds: %ld\n",elapsed);
            }
            printf("REGISTER_SEQ: END\n");
            break;
          case SET_READY_FLAG:
            //printf("SET READY : START\n");
            gettimeofday(&t0,NULL);
            if ( (r < 0) || (c < 0)) {
              msg.status=-1;
            } else {
              msg.status=0;
              pthread_mutex_lock(&controlprogram_list_lock);
              if (control_program->active!=0) control_program->active=1;
              pthread_mutex_unlock(&controlprogram_list_lock);
              i=0;
              rc = pthread_create(&threads[i], NULL,(void *) &timing_wait, NULL);
              pthread_join(threads[0],NULL);
              pthread_mutex_lock(&controlprogram_list_lock);
              //printf("SET READY : Inside Lock controlprogram list\n");
              control_program->state->ready=1;
              i=0;
              //printf("SET READY : Start with ready threads\n");
              //printf("SET READY : Start DIO thread\n");
              rc = pthread_create(&threads[i], NULL, (void *) &DIO_ready_controlprogram, control_program);
              i++;
              //printf("SET READY : Start timing thread\n");
              rc = pthread_create(&threads[i], NULL, (void *) &timing_ready_controlprogram, control_program);
              i++;
              //printf("SET READY : Start dds thread\n");
              rc = pthread_create(&threads[i], NULL, (void *) &dds_ready_controlprogram, control_program);
              i++;
              //printf("SET READY : Start receiver thread\n");
              rc = pthread_create(&threads[i], NULL, (void *) &receiver_ready_controlprogram, control_program);
              for (;i>=0;i--) {
                gettimeofday(&t2,NULL);
                pthread_join(threads[i],NULL);
                gettimeofday(&t3,NULL);
                if (verbose > 1) {
                   elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
                   elapsed+=(t3.tv_usec-t2.tv_usec);
                   if (verbose > 1 ) printf("Client:Set Ready: %d Elapsed Microseconds: %ld\n",i,elapsed);
                }
              }
              //printf("SET READY : Done with ready threads\n");
              gettimeofday(&t2,NULL);
              //printf("SET READY : Start coord thread\n");
              rc = pthread_create(&thread, NULL, (void *)&coordination_handler,(void *) control_program);
              pthread_join(thread,NULL);
              //printf("SET READY : Done with coord thread\n");
              gettimeofday(&t3,NULL);
              if (verbose > 1) {
                   elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
                   elapsed+=(t3.tv_usec-t2.tv_usec);
                   if (verbose > 1 ) printf("Client:Set Ready: Coord Elapsed Microseconds: %ld\n",elapsed);
              }
              pthread_mutex_unlock(&controlprogram_list_lock);
            }
            //printf("SET READY : send return msg\n");
            send_data(socket, &msg, sizeof(struct ROSMsg));
            gettimeofday(&t1,NULL);
            if (verbose > 1) {
              elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
              elapsed+=(t1.tv_usec-t0.tv_usec);
              if (verbose > 1 ) printf("Client:  Set Ready Elapsed Microseconds: %ld\n",elapsed);
            }
            //printf("SET READY : END\n");
            break;

          case REQUEST_CLEAR_FREQ_SEARCH:
            printf("REQUEST_CLEAR_FREQ_SEARCH: START\n");
            gettimeofday(&t0,NULL);
            pthread_mutex_lock(&controlprogram_list_lock);
            recv_data(socket,&control_program->clrfreqsearch, sizeof(struct CLRFreqPRM)); // requested search parameters
            printf("Client: Requst CLRSearch: %d %d\n",control_program->clrfreqsearch.start,control_program->clrfreqsearch.end);
            if ( (r < 0) || (c < 0)) {
              msg.status=-1;
            } else {
              rc = pthread_create(&threads[0], NULL, (void *) &DIO_pre_clrfreq,control_program);
              pthread_join(threads[0],NULL);
              rc = pthread_create(&threads[0], NULL, (void *) &receiver_clrfreq,control_program);
              pthread_join(threads[0],NULL);
              rc = pthread_create(&threads[0], NULL, (void *) &DIO_post_clrfreq,NULL);
              pthread_join(threads[0],NULL);
              msg.status=control_program->state->freq_change_needed;
            }
            send_data(socket, &msg, sizeof(struct ROSMsg));
            pthread_mutex_unlock(&controlprogram_list_lock);
            gettimeofday(&t1,NULL);
            if (verbose > 1) {
              elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
              elapsed+=(t1.tv_usec-t0.tv_usec);
              if (verbose > 1 ) printf("Client:  CLR Elapsed Microseconds: %ld\n",elapsed);
            }
            printf("REQUEST_CLEAR_FREQ_SEARCH: END\n");
            break;
          case REQUEST_ASSIGNED_FREQ:
            gettimeofday(&t0,NULL);
            pthread_mutex_lock(&controlprogram_list_lock);
            if ( (r < 0) || (c < 0)) {
              msg.status=-1;
              control_program->state->current_assigned_freq=0;
              control_program->state->current_assigned_noise=0;
            } else {
              rc = pthread_create(&threads[0], NULL, (void *) &receiver_assign_frequency,(void *)  control_program);
              pthread_join(threads[0],NULL);
              msg.status=control_program->state->best_assigned_freq!=control_program->state->current_assigned_freq;
            }
            //control_program->state->current_assigned_noise=1;
            current_freq=control_program->state->current_assigned_freq; 
            send_data(socket, &current_freq, sizeof(int32));
            send_data(socket, &control_program->state->current_assigned_noise, sizeof(float));
            send_data(socket, &msg, sizeof(struct ROSMsg));
            pthread_mutex_unlock(&controlprogram_list_lock);
            gettimeofday(&t1,NULL);
            if (verbose > 1) {
              elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
              elapsed+=(t1.tv_usec-t0.tv_usec);
              if (verbose > 1 ) printf("Client:  Request Freq Elapsed Microseconds: %ld\n",elapsed);
            }
            break;

          case QUIT:
            gettimeofday(&t0,NULL);
            printf("Client QUIT\n");
            msg.status=0;
            send_data(socket, &msg, sizeof(struct ROSMsg));
            //controlprogram_exit(control_program);
            pthread_exit(NULL);
            break;
          default:
            msg.status=1;
            send_data(socket, &msg, sizeof(struct ROSMsg));
        }
          /* FD_ISSET(0, &rfds) will be true. */
      } //else printf("No data within five seconds.\n");
//        if (verbose>1) printf("Client: test cancel\n");
        
        pthread_testcancel();
   }
   pthread_testcancel();
   pthread_cleanup_pop(0);
   controlprogram_exit(control_program);
   pthread_exit(NULL);
};

void *controlprogram_free(struct ControlProgram *control_program)
{

}
