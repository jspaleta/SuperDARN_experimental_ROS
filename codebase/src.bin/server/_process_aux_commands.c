#include <pthread.h>
#include "iniparser.h" 
#include "dictionary.h" 
#include "rosmsg.h"
#include "global_server_variables.h"
#include "dio_handler.h"
#include "gps_handler.h"

int process_aux_msg(struct ROSMsg incoming_msg,struct ROSMsg *outbound_msg) {
        struct ROSMsg msg;
	pthread_t thread;
        
        driver_msg_init(&msg);
        printf("AUX: in: %s: %s\n",incoming_msg.driver,incoming_msg.command_name);
        printf("AUX: msg: %s: %s\n",msg.driver,msg.command_name);
        memmove(&msg,&incoming_msg,sizeof(struct ROSMsg));

	if (strcmp(incoming_msg.driver,"DIO")==0) {
          printf("AUX: %s: %s\n",msg.driver,msg.command_name);
          pthread_create(&thread,NULL,(void *) &DIO_aux_msg,&msg);
          pthread_join(thread,NULL);
	}
	if (strcmp(incoming_msg.driver,"GPS")==0) {
          printf("AUX: %s: %s\n",msg.driver,msg.command_name);
          pthread_create(&thread,NULL,(void *) &GPS_aux_msg,&msg);
          pthread_join(thread,NULL);
	}

        memmove(outbound_msg,&msg,sizeof(struct ROSMsg));
        return 0;
    
}
