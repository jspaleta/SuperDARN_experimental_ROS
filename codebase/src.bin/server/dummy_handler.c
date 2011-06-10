#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include "global_server_variables.h"


void dummy_exit(void *arg)
{
   int *sockfd = (int *) arg;
   pthread_t tid;
/* get the calling thread's ID */
   tid = pthread_self();
/* print where the thread was in its search when it was cancelled */
   printf("Dummy thread %p has been cancelled\n", tid); 
}

void *dummy_handler(void *arg)
{

/* set the cancellation parameters --
   - Enable thread cancellation 
   - Defer the action of the cancellation 
*/
   pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
   pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
   pthread_cleanup_push(dummy_exit,NULL);

   printf("Inside the dummy handler\n");
   while (1) {
     printf("Inside the Dummy Handler.\n");
     pthread_testcancel();
     sleep(60);
     
   }
   printf("dummy handler: outside of While Loop\n");
   pthread_cleanup_pop(0);
   pthread_exit(NULL);
};


