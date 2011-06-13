#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "dio_handler.h"
#include "timing_handler.h"
#include "reciever_handler.h"
#include "dds_handler.h"

extern struct Thread_List_Item *controlprogram_threads;
extern pthread_mutex_t coord_lock;
extern int *trigger_state_pointer; //0-no activity,1-pre-trigger, 2-triggering 
extern int *ready_state_pointer; //0-no cntrolprograms ready,1-some controlprograms ready, 2-all controlprograms ready 
extern int *ready_count_pointer;
extern int trigger_type;  //0-strict controlprogram ready  1-elasped software time  2-external gps event 
extern int txread[MAX_RADARS];
extern int verbose;
extern int gpssock;
int oldv;
void *coordination_handler(struct ControlProgram *control_program)
{
   int numcontrolprograms=0,numready=0,numprocessing=0,rc,i,temp;
   char *timestr;
   pthread_t threads[4];
   struct Thread_List_Item *thread_list;
   int gps_event,gpssecond,gpsnsecond;
   struct DriverMsg msg;
   struct ControlProgram *cprog;
   int ready_state,trigger_state,ready_count;
   struct timeval t0,t1,t2,t3,t4,t5,t6;
   unsigned long elapsed;
   if(verbose > 1 ) gettimeofday(&t0,NULL);
   pthread_mutex_lock(&coord_lock); //lock the global structures

   ready_state=*ready_state_pointer;
   trigger_state=*trigger_state_pointer;
   ready_count=*ready_count_pointer;
   if (control_program->active==1) {
     ready_count++;
     //control_program->state->ready=1;
     control_program->state->processing=0;
     txread[control_program->parameters->radar-1]=1;
   } else {
   }
/*Calculate Ready State*/
   if(trigger_state<2) { 
     thread_list=controlprogram_threads;
     while(thread_list!=NULL){
       cprog=thread_list->data;
         if (cprog!=NULL) {
           if (cprog->active==1) {
             numcontrolprograms++;
             if (cprog->state->ready==1) {
               numready++;
             }
             if (cprog->state->processing==1) {
               numprocessing++;
             }

           }
         } else {
       }
       thread_list=thread_list->prev;
     }
     if (numready==0) {
       ready_state=0;
     } else if ((numready!=numcontrolprograms) && (numready > 0)) {
       ready_state=1;
     } else if ((numready==numcontrolprograms) && (numready > 0)) {
       ready_state=2;
     } else {
     }
     //printf("Coord: numready %d numprograms %d readystate %d\n",numready,numcontrolprograms,ready_state); 
   }

/*Coordinate Trigger events*/
      switch(ready_state) {
        case 0:
        //printf("Coord: No control programs ready\n"); 
        /*no control program ready for trigger*/
          break;
        case 1:
        /*some control programs ready for trigger*/
          break;
        case 2:
        //printf("Coord: All control programs ready\n"); 
        if(verbose > 1 ) gettimeofday(&t1,NULL);
          if (verbose > 1) { 
              elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
              elapsed+=(t1.tv_usec-t0.tv_usec);
              printf("Coord: Start Ready State Case 2 Elapsed Microseconds: %ld\n",elapsed);
          }
        /*all control programs ready for trigger*/
          trigger_state=1;
          thread_list=controlprogram_threads;


          thread_list=controlprogram_threads;
          while(thread_list!=NULL){
            cprog=thread_list->data;
            if (cprog!=NULL) {
              if (cprog->active==1) {
                if (cprog->state->ready==1) {
                }
                if (cprog->state->processing==1) {
                }
              }
            } else {
            }
            thread_list=thread_list->prev;
          }

          if(verbose > 1 ) gettimeofday(&t2,NULL);
          if (verbose > 1) { 
              elapsed=(t2.tv_sec-t1.tv_sec)*1E6;
              elapsed+=(t2.tv_usec-t1.tv_usec);
              printf("Coord: Pre-Trigger Active Check %d Elapsed Microseconds: %ld\n",i,elapsed);
          }
            if(verbose > -1 ) gettimeofday(&t4,NULL);
          i=0;
          if(verbose > 1) {
            printf("Coord: Start DDS Pre-Trigger\n");
            gettimeofday(&t4,NULL);
          }
          //printf("Coord: DDS Pre-trigger\n"); 
          rc = pthread_create(&threads[i], NULL, (void *) &dds_pretrigger, NULL);
          if(verbose > 1 ) {
              gettimeofday(&t5,NULL);
              elapsed=(t5.tv_sec-t4.tv_sec)*1E6;
              elapsed+=(t5.tv_usec-t4.tv_usec);
              printf("Coord: Pre-Trigger DDS Thread Elapsed Microseconds: %ld\n",elapsed);
          }
          i++;
          if(verbose > 1) {
            printf("Coord: Start Recv Pre-Trigger\n");
            gettimeofday(&t4,NULL);
          }
          rc = pthread_create(&threads[i], NULL, (void *) &receiver_pretrigger, NULL);
          if(verbose > 1 ) {
              gettimeofday(&t5,NULL);
              elapsed=(t5.tv_sec-t4.tv_sec)*1E6;
              elapsed+=(t5.tv_usec-t4.tv_usec);
              printf("Coord: Pre-Trigger Recv Thread Elapsed Microseconds: %ld\n",elapsed);
          }
          i++;
          if(verbose > 1) {
            printf("Coord: Start DIO Pre-Trigger\n");
            gettimeofday(&t4,NULL);
          }
          rc = pthread_create(&threads[i], NULL, (void *) &DIO_pretrigger, NULL);
          if(verbose > 1 ) {
              gettimeofday(&t5,NULL);
              elapsed=(t5.tv_sec-t4.tv_sec)*1E6;
              elapsed+=(t5.tv_usec-t4.tv_usec);
              printf("Coord: Pre-Trigger DIO Thread Elapsed Microseconds: %ld\n",elapsed);
          }
          i++;
          if(verbose > 1) {
            printf("Coord: Start Timing Pre-Trigger\n");
            gettimeofday(&t4,NULL);
          }
          rc = pthread_create(&threads[i], NULL, (void *) &timing_pretrigger, NULL);
          if(verbose > 1 ) {
              gettimeofday(&t5,NULL);
              elapsed=(t5.tv_sec-t4.tv_sec)*1E6;
              elapsed+=(t5.tv_usec-t4.tv_usec);
              printf("Coord: Pre-Trigger Timing Thread Elapsed Microseconds: %ld\n",elapsed);
          }
          for (;i>=0;i--) {
            pthread_join(threads[i],NULL);
          }
            if(verbose > 1 ) gettimeofday(&t5,NULL);
            if (verbose > 1) { 
              elapsed=(t5.tv_sec-t4.tv_sec)*1E6;
              elapsed+=(t5.tv_usec-t4.tv_usec);
              printf("Coord: Pre-Trigger Thread %d Elapsed Microseconds: %ld\n",i,elapsed);
            }
          if (verbose > 1) { 
            gettimeofday(&t3,NULL);
            elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
            elapsed+=(t3.tv_usec-t2.tv_usec);
            printf("Coord: Pre-Trigger Elapsed Microseconds: %ld\n",elapsed);
          }
          //printf("Coord: Pre-trigger done\n"); 
          trigger_state=2; //trigger
/*
 *             trigger_type:  0: free run  1: elapsed-time  2: gps
 */       
          usleep(1000);
          rc = pthread_create(&threads[0], NULL, (void *) &timing_trigger, (void *)trigger_type);
          pthread_join(threads[0],NULL);
          if (verbose > 1) { 
            gettimeofday(&t4,NULL);
            elapsed=(t4.tv_sec-t3.tv_sec)*1E6;
            elapsed+=(t4.tv_usec-t3.tv_usec);
            printf("Coord: Trigger Elapsed Microseconds: %ld\n",elapsed);
          }
          trigger_state=3;//post-trigger
          msg.type=GPS_GET_EVENT_TIME;
          msg.status=1;
          send_data(gpssock, &msg, sizeof(struct DriverMsg));
          recv_data(gpssock,&gps_event, sizeof(int));
          recv_data(gpssock,&gpssecond, sizeof(int));
          recv_data(gpssock,&gpsnsecond, sizeof(int));
          recv_data(gpssock, &msg, sizeof(struct DriverMsg));
          i=0;
          if(control_program->active==1) { 
            control_program->state->gpssecond=gpssecond;
            control_program->state->gpsnsecond=gpsnsecond;
            if (txread[control_program->parameters->radar-1]){
              rc = pthread_create(&threads[i], NULL, (void *) &DIO_transmitter_status, (void *)control_program->parameters->radar);
              pthread_join(threads[i],NULL);
              txread[control_program->parameters->radar-1]=0;
            }
          }
          i=0;
          rc = pthread_create(&threads[i], NULL, (void *) &receiver_posttrigger, NULL);
          i++;
          rc = pthread_create(&threads[i], NULL, (void *) &timing_posttrigger, NULL);
          for (;i>=0;i--) {
            pthread_join(threads[i],NULL);
          }
          thread_list=controlprogram_threads;
          while(thread_list!=NULL){
            cprog=thread_list->data;
            if (cprog!=NULL) {
                if (cprog->state!=NULL) {
                  if (cprog->state->ready==1) {
                    cprog->state->ready=0;
                    cprog->state->processing=1;
                  }
                }
            }
            thread_list=thread_list->prev;
          }
          trigger_state=0;//post-trigger
          ready_count=0;
          ready_state=0;
          if(verbose > 1 ) gettimeofday(&t6,NULL);
          if (verbose > 1) { 
            elapsed=(t6.tv_sec-t4.tv_sec)*1E6;
            elapsed+=(t6.tv_usec-t4.tv_usec);
            printf("Coord: Post Trigger Elapsed Microseconds: %ld\n",elapsed);
          }
          if (verbose > 1) { 
            elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
            elapsed+=(t6.tv_usec-t0.tv_usec);
            printf("Coord: Total Elapsed Microseconds: %ld\n",elapsed);
          }
          break; 
   } // end of ready_state switch
   *ready_state_pointer=ready_state;
   *trigger_state_pointer=trigger_state;
   *ready_count_pointer=ready_count;

   pthread_mutex_unlock(&coord_lock); //unlock 
   pthread_exit(NULL);
};


