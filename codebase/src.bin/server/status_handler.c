#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include "global_server_variables.h"

extern int verbose;

void status_exit(void *arg)
{
   pthread_t tid;
/* get the calling thread's ID */
   tid = pthread_self();
}

void *status_handler(void *arg)
{
/* set the cancellation parameters --
   - Enable thread cancellation 
   - Defer the action of the cancellation 
*/
   pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
   pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
   pthread_cleanup_push(&status_exit,NULL);  // call status_exit if pthread cancelled

   while (1) {
     if (verbose > 0) fprintf(stderr,"Inside the Status Handler while loop.\n");
     sleep(1);
   }
   pthread_cleanup_pop(0);
   pthread_exit(NULL);
};


