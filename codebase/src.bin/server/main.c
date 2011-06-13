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

char *ros_ip=ROS_IP;
int ros_port=ROS_PORT;
/* Required Driver Global Variables */
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
pthread_t status_thread=NULL,timeout_thread=NULL;

/* State Global Variables */
dictionary *Site_INI;
void *radar_channels[MAX_RADARS*MAX_CHANNELS];
int *trigger_state_pointer; 
int trigger_type;
int *ready_state_pointer,*ready_count_pointer;
struct tx_status txstatus[MAX_RADARS];
int txread[MAX_RADARS];
struct SiteSettings site_settings;
struct GPSStatus gpsstatus;
struct TRTimes bad_transmit_times;
int32 gpsrate=REFRESHRATE;
int verbose=10;
int die_on_socket_failure=0;
int clear_frequency_request;
struct BlackList *blacklist=NULL;
int *blacklist_count_pointer;
struct ClrPwr *latest_clr_fft[MAX_RADARS];
struct timeval latest_full_clr_time;
int full_clr_wait=MAX_CLR_WAIT;
int full_clr_start=FULL_CLR_FREQ_START;
int full_clr_end=FULL_CLR_FREQ_END;
int max_freqs=0;
int sockfd;
unsigned long error_count=0,collection_count=0;


void graceful_cleanup(int signum)
{
  int t=0,i=0;
  struct Thread_List_Item *thread_list,*thread_item,*thread_next;
  struct ControlProgram *data;

  if (verbose>-1) fprintf(stderr,"Attempting graceful clean up of control program threads\n");
  thread_list=controlprogram_threads;
  t=0;
  if (thread_list!=NULL) {
    while(thread_list!=NULL){
      if (verbose>0) fprintf(stderr,"Cancelling thread %ld\n",t);
      pthread_cancel(thread_list->id);       
      if (verbose>0) fprintf(stderr,"Done Cancelling thread %ld\n",t);
      if (verbose>0) fprintf(stderr,"Joining thread %ld\n",t);
      pthread_join(thread_list->id,NULL);
      if (verbose>0) fprintf(stderr,"Done Joining thread %ld\n",t);
      pthread_mutex_lock(&controlprogram_list_lock);
      thread_item=thread_list;   
      if (verbose>0) fprintf(stderr,"thread item %p\n",thread_item);
      if (thread_item!=NULL) {
        thread_next=thread_item->next;
        thread_list=thread_item->prev;
        if (thread_next != NULL) thread_next->prev=thread_list;
        else controlprogram_threads=thread_list;
        if (thread_list != NULL) thread_list->next=thread_item->next;
        if (verbose>0) fprintf(stderr,"freeing thread item\n");
        free(thread_item);
        if (verbose>0) fprintf(stderr,"freed thread item\n");
        thread_item=NULL;
      }
      pthread_mutex_unlock(&controlprogram_list_lock);
      t++;
    }
  }
  if (verbose>0) fprintf(stderr,"Done with control program threads now lets do worker threads\n");
  if (status_thread!=NULL) {
    if (verbose>0) fprintf(stderr,"Cancelling Status thread\n");
    pthread_cancel(status_thread);       
    if (verbose>0) fprintf(stderr,"  Status thread cancelled\n");
    if (verbose>0) fprintf(stderr,"  Waiting for Status thread to end\n");
    pthread_join(status_thread,NULL);
    if (verbose>0) fprintf(stderr,"  Status thread done\n");
  }
  if (timeout_thread!=NULL) {
    if (verbose>0) fprintf(stderr,"Cancelling Timeout thread\n");
    pthread_cancel(timeout_thread);
    if (verbose>0) fprintf(stderr,"  Timeout thread cancelled\n");
    if (verbose>0) fprintf(stderr,"  joining Timeout thread\n");
    pthread_join(timeout_thread,NULL);
    if (verbose>0) fprintf(stderr,"  Back from joining Timeout thread\n");
  }
  errno=ECANCELED;
  if (verbose>0) fprintf(stderr,"Done with worker threads now lets exit\t");
  perror( "--> Stopping the ROS server process");
  fprintf(stderr,"Closing Main socket: %d\n",sockfd);
  close(sockfd);
  fprintf(stderr,"Closing DIO socket: %d\n",diosock);
  close(diosock);   
  fprintf(stderr,"Closing timing socket: %d\n",timingsock);
  close(timingsock);   
  fprintf(stderr,"Closing RECV socket: %d\n",recvsock);
  close(recvsock);   
  fprintf(stderr,"Closing DDS socket: %d\n",ddssock);
  close(ddssock);   
  fprintf(stderr,"Closing GPS socket: %d\n",gpssock);
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
  struct sockaddr_un serv_addr;
  struct sockaddr_un cli_addr;
  struct Thread_List_Item *thread_list;
  struct ControlProgram *control_program;
  struct DriverMsg s_msg,r_msg;
  int newsockfd, clilen,rc,i,r;
  int num_threads;
  int restrict_count,blacklist_count,start,end;
  char restrict_file[120],hmm[120];
  pthread_t thread;
  struct timeval t0,current_time,default_timeout;
/* DIO Variables may need to move */
  char *frq_name=NULL;
  char *cnf_name=NULL;
  struct FreqTable *fptr=NULL;
  FILE *fp,*fd;
  char *s,*line,*field;
  int dfrq=13000;
/* end DIO */
  printf("Size of Struct ROSMsg  %d\n",sizeof(struct ROSMsg));
  printf("Size of Struct int32  %d\n",sizeof(int32));
  printf("Size of Struct float  %d\n",sizeof(float));
  printf("Size of Struct unsigned char  %d\n",sizeof(unsigned char));
  printf("Size of Struct ControlPRM  %d\n",sizeof(struct ControlPRM));
  printf("Size of Struct CLRFreqPRM  %d\n",sizeof(struct CLRFreqPRM));
  printf("Size of Struct SeqPRM  %d\n",sizeof(struct SeqPRM));
  printf("Size of Struct DataPRM  %d\n",sizeof(struct DataPRM));
  printf("Size of Struct SiteSettings  %d\n",sizeof(struct SiteSettings));

  gettimeofday(&t0,NULL);
  default_timeout.tv_sec=10;
/* Set up global CLR_Frequecy */
  max_freqs=(full_clr_end-full_clr_start);
  latest_full_clr_time=t0;
  for(r=0;r<MAX_RADARS;r++){
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
  for(i=0;i<MAX_RADARS*MAX_CHANNELS;i++) {
    radar_channels[i]=NULL ;
  }
  bad_transmit_times.length=0;
  bad_transmit_times.start_usec=NULL;
  bad_transmit_times.duration_usec=NULL;
  blacklist_count_pointer=malloc(sizeof(int));
  blacklist_count=0;
  sprintf(restrict_file,"%s/restrict.dat",SITE_DIR);
  fprintf(stdout,"Opening restricted file: %s\n",restrict_file);
  fd=fopen(restrict_file,"r+");
  restrict_count=0;
  s=fgets(hmm,120,fd);
  while (s!=NULL) {
    restrict_count++;
    s=fgets(hmm,120,fd);
  }
  fprintf(stdout,"Number of restricted windows: %d\n",restrict_count);
  fprintf(stdout,"max blacklist: %d\n",restrict_count+Max_Control_THREADS*4);
  fclose(fd);
  blacklist = (struct BlackList*) malloc(sizeof(struct BlackList) * (restrict_count+Max_Control_THREADS*4));
  fprintf(stdout,"Blacklist : %p\n",blacklist);
  
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
          blacklist[blacklist_count].program=NULL;
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
 * Setup DIO State Variables
 * 
 */
  if (frq_name !=NULL) {
    fp=fopen(frq_name,"r");
    if (fp==NULL) {
      perror("Could not load frequency table");
      exit(1);
    }
    fptr=FreqLoadTable(fp);
    dfrq=fptr->dfrq;
    fclose(fp);
  }
/*
 * Setup Radar Setting State Variables
 *
*/
  fprintf(stdout,"Default IF Enable flag : %d\n",IF_ENABLED);
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
  fprintf(stdout,"Configured IF Enable flag : %d\n",site_settings.ifmode);
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
    s_msg.type=GPS_SET_TRIGGER_RATE;
    s_msg.status=1;
    send_data(gpssock, &s_msg, sizeof(struct DriverMsg));
    recv_data(gpssock, &r_msg, sizeof(struct DriverMsg));
    if(r_msg.status==1) {
      send_data(gpssock, &gpsrate, sizeof(gpsrate));
      recv_data(gpssock, &r_msg, sizeof(struct DriverMsg));
    }
  }
  if (verbose>0) fprintf(stderr,"Done with Sockets\n");
/*
 * Set up all main worker threads here
 * 
 */
///* Hdw Status handler */
//  rc = pthread_create(&status_thread, NULL, status_handler, NULL);
/* Control Program Activity Timeout handler */
  if (verbose>0) fprintf(stderr,"Create timeout thread\n");
  rc = pthread_create(&timeout_thread, NULL, timeout_handler, NULL);

  if (verbose>0) fprintf(stderr,"start reciever RXFE settings thread\n");
  rc = pthread_create(&thread, NULL, &receiver_site_settings,(void *)&site_settings);
  pthread_join(thread,NULL);
  if (verbose>0) fprintf(stderr,"start dds RXFE settings thread\n");
  rc = pthread_create(&thread, NULL, &dds_site_settings,(void *)&site_settings);
  pthread_join(thread,NULL);
  if (verbose>0) fprintf(stderr,"start dio RXFE settings thread\n");
  rc = pthread_create(&thread, NULL, &dio_site_settings,(void *)&site_settings);
  pthread_join(thread,NULL);
  if (verbose>0) fprintf(stderr,"Done with RXFE settings threads\n");
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
       if (verbose > 1) fprintf(stdout,"ROS Main: New Client!\n");
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
