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
extern int verbose;
extern int gpssock;
int oldv;
void *coordination_handler(struct ControlProgram *control_program)
{
   int numcontrolprograms=0,numready=0,numprocessing=0,rc,i;
   pthread_t threads[4];
   struct Thread_List_Item *thread_list;
   int32 gps_event,gpssecond,gpsnsecond;
   struct DriverMsg s_msg,r_msg;
   struct ControlProgram *cprog;
   int ready_state,trigger_state,ready_count;

   pthread_mutex_lock(&coord_lock); //lock the global structures

   ready_state=*ready_state_pointer;
   trigger_state=*trigger_state_pointer;
   ready_count=*ready_count_pointer;
   if (control_program->state->active==1) {
     ready_count++;
     //control_program->state->ready=1;
     control_program->state->processing=0;
   } else {
   }

   /*Calculate global Ready State*/
   if(trigger_state<1) { 
     thread_list=controlprogram_threads;
     while(thread_list!=NULL){
       cprog=thread_list->data;
       if (cprog!=NULL) {
         if (cprog->state!=NULL) {
           if (cprog->state->active==1) {
             numcontrolprograms++;
             if (cprog->state->ready==1) {
               numready++;
             }
             if (cprog->state->processing==1) {
               numprocessing++;
             }
           }
         }
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
        /*all control programs ready for trigger*/

          //TODO: Find the highest priority control_program that is ready
          
          //TODO: Fill parameters of other controlprograms based on the priority channel

	  //Inform the drivers that all the active controlprograms are ready
          thread_list=controlprogram_threads;
          while(thread_list!=NULL){
            cprog=thread_list->data;
            if (cprog!=NULL) {
              if (cprog->state!=NULL) {
                if (cprog->state->active==1) {
                  if (cprog->parameters!=NULL) {
                    i=0;
                    rc = pthread_create(&threads[i], NULL, (void *) &DIO_ready_controlprogram, cprog);
                    i++;
                    rc = pthread_create(&threads[i], NULL, (void *) &timing_ready_controlprogram, cprog);
                    i++;
                    rc = pthread_create(&threads[i], NULL, (void *) &dds_ready_controlprogram, cprog);
                    i++;
                    rc = pthread_create(&threads[i], NULL, (void *) &receiver_ready_controlprogram, cprog);
                    for (;i>=0;i--) {
                      pthread_join(threads[i],NULL);
                    }
                  }
                 }
              }
            }
            thread_list=thread_list->prev;
          }

	  //TODO: Get calculated trigger offsets

          thread_list=controlprogram_threads;
          while(thread_list!=NULL){
            cprog=thread_list->data;
            if (cprog!=NULL) {
              if (cprog->state!=NULL) {
                if (cprog->state->active==1) {
                  if (cprog->parameters!=NULL) {
                    i=0;
                    rc = pthread_create(&threads[i], NULL, (void *) &dds_get_trigger_offset, cprog);
                    i++;
                    rc = pthread_create(&threads[i], NULL, (void *) &receiver_get_trigger_offset, cprog);
                    for (;i>=0;i--) {
                      pthread_join(threads[i],NULL);
                    }
                  }
                 }
              }
            }
            thread_list=thread_list->prev;
          }

          //TODO: Use priority channel to force offsets

	  //Set calculated trigger offsets
          thread_list=controlprogram_threads;
          while(thread_list!=NULL){
            cprog=thread_list->data;
            if (cprog!=NULL) {
              if (cprog->state!=NULL) {
                if (cprog->state->active==1) {
                  if (cprog->parameters!=NULL) {
                    i=0;
                    rc = pthread_create(&threads[i], NULL, (void *) &timing_set_trigger_offset, cprog);
                    for (;i>=0;i--) {
                      pthread_join(threads[i],NULL);
                    }
                  }
                 }
              }
            }
            thread_list=thread_list->prev;
          }
          trigger_state=1; //pretrigger
          i=0;
          rc = pthread_create(&threads[i], NULL, (void *) &dds_pretrigger, NULL);
          i++;
          rc = pthread_create(&threads[i], NULL, (void *) &receiver_pretrigger, NULL);
          i++;
          rc = pthread_create(&threads[i], NULL, (void *) &DIO_pretrigger, NULL);
          i++;
          rc = pthread_create(&threads[i], NULL, (void *) &timing_pretrigger, NULL);
          for (;i>=0;i--) {
            pthread_join(threads[i],NULL);
          }

          trigger_state=2; //trigger
/*
 *             trigger_type:  0: free run  1: elapsed-time  2: gps
 */       
          rc = pthread_create(&threads[0], NULL, (void *) &timing_trigger, (void *)&trigger_type);
          pthread_join(threads[0],NULL);
          trigger_state=3;//post-trigger

          driver_msg_init(&s_msg);
          driver_msg_init(&r_msg);
          driver_msg_set_command(&s_msg,GET_EVENT_TIME,"get_event_time","GPS");
          driver_msg_send(gpssock, &s_msg);
          driver_msg_recv(gpssock, &r_msg);
          driver_msg_get_var_by_name(&r_msg,"gps_event",&gps_event);
          driver_msg_get_var_by_name(&r_msg,"gps_second",&gpssecond);
          driver_msg_get_var_by_name(&r_msg,"gps_nsecond",&gpsnsecond);
          driver_msg_free_buffer(&r_msg);
          driver_msg_free_buffer(&s_msg);

          i=0;
          if(control_program->state->active==1) { 
            control_program->state->gpssecond=gpssecond;
            control_program->state->gpsnsecond=gpsnsecond;
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
          trigger_state=0;//not processing a triggered event
          ready_count=0;
          ready_state=0;
          break; 
   } // end of ready_state switch
   *ready_state_pointer=ready_state;
   *trigger_state_pointer=trigger_state;
   *ready_count_pointer=ready_count;

   pthread_mutex_unlock(&coord_lock); //unlock 
   pthread_exit(NULL);
};


