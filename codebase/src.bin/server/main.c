/*"global_server_variables.h"*/

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include "global_server_variables.h"
#include "control_program.h"
#include "client_handler.h"
#include "timeout_handler.h"
#include "status_handler.h"
#include "settings_handler.h"
#include "dio_handler.h"
#include "dds_handler.h"
#include "reciever_handler.h"
#include "dummy_handler.h"
#include "utils.h"
#include "tsg.h"
#include "iniparser.h"

/* Required Driver Global Variables */
char *ros_ip=ROS_IP;
int ros_port=ROS_PORT;
char *diohostip=DIO_HOST_IP;
int dioport=DIO_HOST_PORT;
int diosock=-1;
char *timinghostip=TIMING_HOST_IP;
int timingport=TIMING_HOST_PORT;
int timingsock=-1;
char *gpshostip=GPS_HOST_IP;
int gpsport=GPS_HOST_PORT;
int gpssock=-1;
char *ddshostip=DDS_HOST_IP;
int ddsport=DDS_HOST_PORT;
int ddssock=-1;
char *recvhostip=RECV_HOST_IP;
int recvport=RECV_HOST_PORT;
int recvsock=-1;

/* Thread Management Global Variables */
struct Thread_List_Item *controlprogram_threads;
pthread_mutex_t controlprogram_list_lock,ros_state_lock,coord_lock,exit_lock;
pthread_mutex_t dds_comm_lock,timing_comm_lock,gps_comm_lock,timing_comm_lock,recv_comm_lock,dio_comm_lock;
pthread_mutex_t thread_list_lock,settings_lock;
pthread_cond_t ready_flag;
pthread_t status_thread=0,timeout_thread=0;

/* State Global Variables */
dictionary *Site_INI;
int num_radars=0,num_channels=0;
void* **radar_channels;
int *trigger_state_pointer; 
int trigger_type;
int *ready_state_pointer,*ready_count_pointer;
struct SiteSettings site_settings;
struct TRTimes bad_transmit_times;
int32 gpsrate=REFRESHRATE;
int verbose=2;
int die_on_socket_failure=0;
int clear_frequency_request;
struct BlackList *blacklist=NULL;
int *blacklist_count_pointer;

struct ClrPwr* *latest_clr_fft;
int full_clr_start=FULL_CLR_FREQ_START;
int full_clr_end=FULL_CLR_FREQ_END;
int max_freqs=0;
int sockfd;
unsigned long error_count=0,collection_count=0;


void graceful_cleanup(int signum)
{
  int t=0,i=0;
  struct Thread_List_Item *thread_list,*thread_item,*thread_next;

  if (verbose>1) fprintf(stderr,"Attempting graceful clean up of control program threads\n");
  thread_list=controlprogram_threads;
  t=0;
  if (thread_list!=NULL) {
    while(thread_list!=NULL){
      if (verbose>1) fprintf(stderr,"Cancelling thread %d\n",t);
      pthread_cancel(thread_list->id);       
      if (verbose>1) fprintf(stderr,"Done Cancelling thread %d\n",t);
      if (verbose>1) fprintf(stderr,"Joining thread %d\n",t);
      pthread_join(thread_list->id,NULL);
      if (verbose>1) fprintf(stderr,"Done Joining thread %d\n",t);
      pthread_mutex_lock(&controlprogram_list_lock);
      thread_item=thread_list;   
      if (verbose>1) fprintf(stderr,"thread item %p\n",thread_item);
      if (thread_item!=NULL) {
        thread_next=thread_item->next;
        thread_list=thread_item->prev;
        if (thread_next != NULL) thread_next->prev=thread_list;
        else controlprogram_threads=thread_list;
        if (thread_list != NULL) thread_list->next=thread_item->next;
        if (verbose>1) fprintf(stderr,"freeing thread item\n");
        free(thread_item);
        if (verbose>1) fprintf(stderr,"freed thread item\n");
        thread_item=NULL;
      }
      pthread_mutex_unlock(&controlprogram_list_lock);
      t++;
    }
  }
  if (verbose>1) fprintf(stderr,"Done with control program threads now lets do worker threads\n");
  if (status_thread!=0) {
    if (verbose>1) fprintf(stderr,"Cancelling Status thread\n");
    pthread_cancel(status_thread);       
    if (verbose>1) fprintf(stderr,"  Status thread cancelled\n");
    if (verbose>1) fprintf(stderr,"  Waiting for Status thread to end\n");
    pthread_join(status_thread,NULL);
    if (verbose>1) fprintf(stderr,"  Status thread done\n");
  }
  if (timeout_thread!=0) {
    if (verbose>1) fprintf(stderr,"Cancelling Timeout thread\n");
    pthread_cancel(timeout_thread);
    if (verbose>1) fprintf(stderr,"  Timeout thread cancelled\n");
    if (verbose>1) fprintf(stderr,"  joining Timeout thread\n");
    pthread_join(timeout_thread,NULL);
    if (verbose>1) fprintf(stderr,"  Back from joining Timeout thread\n");
  }
  errno=ECANCELED;
  if (verbose>1) fprintf(stderr,"Done with worker threads now lets exit\t");
  perror( "--> Stopping the ROS server process");
  if(verbose > 1 ) fprintf(stderr,"Closing Main socket: %d\n",sockfd);
  close(sockfd);
  if(verbose > 1 )   fprintf(stderr,"Closing DIO socket: %d\n",diosock);
  close(diosock);   
  if(verbose > 1 )   fprintf(stderr,"Closing timing socket: %d\n",timingsock);
  close(timingsock);   
  if(verbose > 1 )   fprintf(stderr,"Closing RECV socket: %d\n",recvsock);
  close(recvsock);   
  if(verbose > 1 )   fprintf(stderr,"Closing DDS socket: %d\n",ddssock);
  close(ddssock);   
  if(verbose > 1 )   fprintf(stderr,"Closing GPS socket: %d\n",gpssock);
  close(gpssock);   
  pthread_mutex_destroy(&controlprogram_list_lock);
  pthread_mutex_destroy(&coord_lock);
  pthread_mutex_destroy(&exit_lock);
  pthread_mutex_destroy(&timing_comm_lock);
  pthread_mutex_destroy(&dds_comm_lock);
  pthread_mutex_destroy(&gps_comm_lock);
  pthread_mutex_destroy(&dio_comm_lock);
  pthread_mutex_destroy(&recv_comm_lock);
  pthread_exit(NULL);
  exit(6);
};

void graceful_socket_cleanup(int signum)
{
  if (verbose>-1) fprintf(stderr,"Socket Error of some kind\n\n");
  exit(0);
}

int main()
{
  struct sockaddr_un cli_addr;
  struct Thread_List_Item *thread_list;
  struct ControlProgram *control_program;
  struct DriverMsg s_msg,r_msg;
  int newsockfd, rc,i,j,r;
  unsigned int clilen;
  int num_threads;
  int restrict_count,blacklist_count,start,end;
  char restrict_file[120],hmm[120];
  pthread_t thread;
  struct timeval current_time,default_timeout;
  char ini_name[120];
  FILE *fd;
  char *s,*line,*field;

  fprintf(stderr,"Size of Struct ROSMsg  %lu\n",(unsigned long) sizeof(struct ROSMsg));
  fprintf(stderr,"Size of Struct DriverMsg  %lu\n",(unsigned long) sizeof(struct DriverMsg));
  fprintf(stderr,"Size of Struct int32  %lu\n",(unsigned long) sizeof(int32));
  fprintf(stderr,"Size of Struct float  %lu\n",(unsigned long) sizeof(float));
  fprintf(stderr,"Size of Struct unsigned char  %lu\n",(unsigned long) sizeof(unsigned char));
  fprintf(stderr,"Size of Struct ControlPRM  %lu\n",(unsigned long) sizeof(struct ControlPRM));
  fprintf(stderr,"Size of Struct CLRFreqPRM  %lu\n",(unsigned long) sizeof(struct CLRFreqPRM));
  fprintf(stderr,"Size of Struct SeqPRM  %lu\n",(unsigned long) sizeof(struct SeqPRM));
  fprintf(stderr,"Size of Struct DataPRM  %lu\n",(unsigned long) sizeof(struct DataPRM));
  fprintf(stderr,"Size of Struct SiteSettings  %lu\n",(unsigned long) sizeof(struct SiteSettings));
/* Put in Commandline arg parsing here */

/* Load Site INI file */
  if(Site_INI!=NULL) {
    iniparser_freedict(Site_INI);
    Site_INI=NULL;
  }
  sprintf(ini_name,"%s/site.ini",SITE_DIR);
  fprintf(stderr, "parsing file: %s\n", ini_name);
  Site_INI=iniparser_load(ini_name);
  if (Site_INI==NULL) {
     fprintf(stderr, "cannot parse file: %s\n", ini_name);
  }
/* Load important Variable Values */
  num_radars=iniparser_getint(Site_INI,"site_settings:num_radars",MAX_RADARS);
  num_channels=iniparser_getint(Site_INI,"site_settings:num_channels",MAX_CHANNELS);
  default_timeout.tv_sec=iniparser_getint(Site_INI,"site_settings:client_timeout_secs",10);
  radar_channels=malloc(num_radars*sizeof(void *));
  for(r=0;r<num_radars;r++){
    radar_channels[r]=malloc(num_channels*sizeof(void *));
  }
  latest_clr_fft=malloc(num_radars*sizeof(void *));


/* Set up global CLR_Frequecy */
  max_freqs=(full_clr_end-full_clr_start);
  for(r=0;r<num_radars;r++){
    latest_clr_fft[r]=calloc(max_freqs,sizeof(struct ClrPwr));
    if (latest_clr_fft[r]!=NULL) {
      for(i=0;i<max_freqs;i++){
        latest_clr_fft[r][i].freq=full_clr_start+i;
        latest_clr_fft[r][i].pwr=0;
      }
    }  
  }

/*
 * Init Thread State Variables
 * 
 */

  pthread_mutex_init(&thread_list_lock, NULL);
  pthread_mutex_init(&settings_lock, NULL);
  pthread_mutex_init(&controlprogram_list_lock, NULL);
  pthread_mutex_init(&coord_lock, NULL);
  pthread_mutex_init(&exit_lock, NULL);
  pthread_mutex_init(&ros_state_lock, NULL);
  pthread_mutex_init(&timing_comm_lock, NULL);
  pthread_mutex_init(&dds_comm_lock, NULL);
  pthread_mutex_init(&dio_comm_lock, NULL);
  pthread_mutex_init(&gps_comm_lock, NULL);
  pthread_mutex_init(&recv_comm_lock, NULL);
  pthread_cond_init (&ready_flag, NULL);

/*
 * Init State Variables
 * 
 */
  controlprogram_threads=NULL;
  clear_frequency_request=0;
  ready_state_pointer=malloc(sizeof(int));
  ready_count_pointer=malloc(sizeof(int));
  trigger_state_pointer=malloc(sizeof(int));
  *ready_state_pointer=0; //no control programs ready
  *ready_count_pointer=0; //no control programs ready
  *trigger_state_pointer=0; //pre-trigger state
  trigger_type=0; //strict control program ready trigger type
  for(i=0;i<num_radars;i++) {
    for(j=0;j<num_channels;j++) {
      radar_channels[i][j]=NULL ;
    }
  }
  bad_transmit_times.length=0;
  bad_transmit_times.start_usec=NULL;
  bad_transmit_times.duration_usec=NULL;
  blacklist_count_pointer=malloc(sizeof(int));
  blacklist_count=0;
  sprintf(restrict_file,"%s/restrict.dat",SITE_DIR);
  fprintf(stderr,"Opening restricted file: %s\n",restrict_file);
  fd=fopen(restrict_file,"r+");
  restrict_count=0;
  s=fgets(hmm,120,fd);
  while (s!=NULL) {
    if(s[0]!='#') 
      restrict_count++;
    s=fgets(hmm,120,fd);
  }
  fprintf(stderr,"Number of restricted windows in file: %d\n",restrict_count);
  fprintf(stderr,"max blacklist: %d\n",restrict_count+Max_Control_THREADS*4);
  fclose(fd);
  blacklist = (struct BlackList*) malloc(sizeof(struct BlackList) * (restrict_count+Max_Control_THREADS*4));
  if( verbose > 1 ) fprintf(stderr,"Blacklist : %p\n",blacklist);
  sprintf(restrict_file,"%s/restrict.dat",SITE_DIR);
  fd=fopen(restrict_file,"r+");
  s=fgets(hmm,120,fd);
  while (s!=NULL) {
      if(s[0]=='#') {
//        //printf("found a comment :%s\n",s);
      } else {
        if ((s[0]>='0') && (s[0] <='9')) {
//          //printf("found a freq :%s\n",s);
          field=strtok_r(s," ",&line);
          start=atoi(field);
          field=strtok_r(NULL," ",&line);
          end=atoi(field);
          blacklist[blacklist_count].start=start;
          blacklist[blacklist_count].end=end;
          blacklist[blacklist_count].program=(uint64)NULL;
          blacklist_count++;
        } else {
//        //printf("found something else: %s\n",s);
        }
      }
      s=fgets(hmm,120,fd);
  }
  fclose(fd);

  *blacklist_count_pointer=blacklist_count; 
/*
 * Setup Radar Setting State Variables
 *
*/
  if (verbose > 1) fprintf(stderr,"Default IF Enable flag : %d\n",IF_ENABLED);
  Site_INI=NULL;
  sprintf(site_settings.name,"%s",SITE_NAME);
  site_settings.ifmode=IF_ENABLED;
  site_settings.rf_settings.ifmode=0;
  site_settings.rf_settings.amp1=0;
  site_settings.rf_settings.amp2=0;
  site_settings.rf_settings.amp3=0;
  site_settings.rf_settings.att1=0;
  site_settings.rf_settings.att2=0;
  site_settings.rf_settings.att3=0;
  site_settings.rf_settings.att4=0;
  site_settings.if_settings.ifmode=1;
  site_settings.if_settings.amp1=0;
  site_settings.if_settings.amp2=0;
  site_settings.if_settings.amp3=0;
  site_settings.if_settings.att1=0;
  site_settings.if_settings.att2=0;
  site_settings.if_settings.att3=0;
  site_settings.rf_settings.att4=0;

  rc = pthread_create(&thread, NULL, (void *) &settings_parse_ini_file,(void *)&site_settings);
  pthread_join(thread,NULL);
  rc = pthread_create(&thread, NULL, (void *) &settings_rxfe_update_rf,(void *)&site_settings.rf_settings);
  pthread_join(thread,NULL);
  rc = pthread_create(&thread, NULL, (void *) &settings_rxfe_update_if,(void *)&site_settings.if_settings);
  pthread_join(thread,NULL);
  if(verbose > 1 )fprintf(stderr,"Configured IF Enable flag : %d\n",site_settings.ifmode);
/*
 * Set up the signal handling

 * 
 */

  signal(SIGINT, graceful_cleanup);
  signal(SIGPIPE, SIG_IGN);
  setbuf(stdout, 0);
  setbuf(stderr, 0);

/******************* TCP Socket Connection ***********/
  if (verbose>0) fprintf(stderr,"Opening DIO Socket %s %d\n",diohostip,dioport);
  diosock=opentcpsock(diohostip, dioport);
  if (diosock < 0) {

    if (verbose>0) fprintf(stderr,"Dio Socket failure %d\n",diosock);
    if (die_on_socket_failure)
      graceful_socket_cleanup(1);
  } else if (verbose>0) fprintf(stderr,"Dio Socket %d\n",diosock);
  if (verbose>0) fprintf(stderr,"Opening DDS Socket\n");
  ddssock=opentcpsock(ddshostip, ddsport);
  if (ddssock < 0) {
    if (verbose>0) fprintf(stderr,"DDS Socket failure %d\n",ddssock);
    if (die_on_socket_failure)
      graceful_socket_cleanup(1);
  } else  if (verbose>0) fprintf(stderr,"DDS Socket %d\n",ddssock);
  if (verbose>0) fprintf(stderr,"Opening Recv Socket\n");
  recvsock=opentcpsock(recvhostip, recvport);
  if (recvsock < 0) {
    if (verbose>0)fprintf(stderr,"RECV Socket failure %d\n",recvsock);
    if (die_on_socket_failure)
      graceful_socket_cleanup(1);
  } else  if (verbose>0) fprintf(stderr,"RECV Socket %d\n",recvsock);
  if (verbose>0) fprintf(stderr,"Opening Timing Socket\n");
  timingsock=opentcpsock(timinghostip, timingport);
  if (timingsock < 0) {
    if (verbose>0) fprintf(stderr,"Timing Socket failure %d\n",timingsock);
    if (die_on_socket_failure)
      graceful_socket_cleanup(1);
  } else  if (verbose>0) fprintf(stderr,"Timing Socket %d\n",timingsock);
  if (verbose>0) fprintf(stderr,"Opening GPS Socket\n");
  gpssock=opentcpsock(gpshostip, gpsport);
  if (gpssock < 0) {
    if (verbose>0) fprintf(stderr,"GPS Socket failure %d\n",gpssock);
    if (die_on_socket_failure)
      graceful_socket_cleanup(1);
  } else {
    if (verbose>0) fprintf(stderr,"GPS Socket %d\n",gpssock);
    s_msg.command_type=GPS_SET_TRIGGER_RATE;
    strcpy(s_msg.command_name,"GPS_SET_TRIGGER_RATE");
    s_msg.status=1;
    send_data(gpssock, &s_msg, sizeof(struct DriverMsg));
    recv_data(gpssock, &r_msg, sizeof(struct DriverMsg));
    if(r_msg.status==1) {
      send_data(gpssock, &gpsrate, sizeof(gpsrate));
      recv_data(gpssock, &r_msg, sizeof(struct DriverMsg));
    }
  }
  if (verbose>1) fprintf(stderr,"Done with Sockets\n");
/*
 * Set up all main worker threads here
 * 
 */
///* Hdw Status handler */
//  rc = pthread_create(&status_thread, NULL, status_handler, NULL);
/* Control Program Activity Timeout handler */
  if (verbose>1) fprintf(stderr,"Create timeout thread\n");
  rc = pthread_create(&timeout_thread, NULL, timeout_handler, NULL);

  if (verbose>1) fprintf(stderr,"start reciever RXFE settings thread\n");
  rc = pthread_create(&thread, NULL, &receiver_site_settings,(void *)&site_settings);
  pthread_join(thread,NULL);
  if (verbose>1) fprintf(stderr,"start dds RXFE settings thread\n");
  rc = pthread_create(&thread, NULL, &dds_site_settings,(void *)&site_settings);
  pthread_join(thread,NULL);
  if (verbose>1) fprintf(stderr,"start dio RXFE settings thread\n");
  rc = pthread_create(&thread, NULL, &dio_site_settings,(void *)&site_settings);
  pthread_join(thread,NULL);

/******************* Init transmitter status arrays ***********/
//rc = pthread_create(&thread, NULL, DIO_transmitter_status, &tstatus);
/******************* Unix Socket Connection ***********/
 fprintf(stderr,"main: create socket server\n");
/*
 * Create the server socket
 */
  if ((sockfd=tcpsocket(ros_port)) == -1) {
          perror("arby server: socket creation failed");
          exit(0);
  }
 fprintf(stderr,"main: listen for sockets %d\n",sockfd);

 fprintf(stderr,"main: listen for clients on socket %d\n",sockfd);
/*
 * Call listen() to enable reception of connection requests
 * (listen() will silently change given backlog 0, to be 1 instead)
 */
     if ((listen(sockfd, 5)) == -1) {
          perror("ROS socket - listen failed");
          exit(0);
     }
/*
 * For loop for controlprogram/viewer connections
 * (listen() will silently change given backlog 0, to be 1 instead)
 */
     num_threads=0;
     while(num_threads<Max_Control_THREADS){
       fprintf(stderr,"ROS Main: Listening for new client connections\n");
       clilen = sizeof(cli_addr);
       newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
       if(newsockfd < 0) {
         perror("arby server: accept error\n");
         exit(0);
       }
       if (verbose > 1) fprintf(stderr,"ROS Main: New Client!\n");
       gettimeofday(&current_time,NULL);
       pthread_mutex_lock(&controlprogram_list_lock);

       /* create and init new control program instance */
       control_program=control_init();
       control_program->state->socket=newsockfd;
       if (verbose > 1) fprintf(stderr,"main: new client socket %d %d\n",control_program->state->socket=newsockfd,newsockfd);
       /* create a new thread to process the incomming request */
       if (verbose > 1) fprintf(stderr,"Main: Client Thread\n");
       rc = pthread_create(&thread, NULL, (void *)control_handler,(void *)control_program);
       if (controlprogram_threads == NULL) {
         fprintf(stderr," No existing threads\n");
         thread_list=malloc(sizeof(struct Thread_List_Item));
         thread_list->next=NULL;
         thread_list->prev=NULL;
         controlprogram_threads=thread_list;
       } else{
         fprintf(stderr," Existing threads\n");
         thread_list=malloc(sizeof(struct Thread_List_Item));
         thread_list->next=NULL;
         thread_list->prev=NULL;
         controlprogram_threads->next=thread_list;
         thread_list->prev=controlprogram_threads;
         controlprogram_threads=thread_list;
       }
       if (verbose > 1 ) {
         fprintf(stderr," New thread: %p  next: %p  prev: %p\n",thread_list, thread_list->next, thread_list->prev);
         fflush(stderr);
       }
       /* set thread linkages */
       controlprogram_threads->id=thread;
       controlprogram_threads->last_seen=current_time;
       controlprogram_threads->timeout=default_timeout;
       controlprogram_threads->data=control_program;
       thread_list=controlprogram_threads; 
       control_program->state->thread=thread_list;
       num_threads=0;
       while(thread_list!=NULL){
         thread_list=thread_list->prev;
         num_threads++;
       }
       pthread_mutex_unlock(&controlprogram_list_lock);

       /* the server is now free to accept another socket request */
     }
     if (verbose>-1) fprintf(stderr,"Too many Control Program Threads\n");
     graceful_cleanup(0);
     return 0;
}
