#include <pthread.h>
#include "iniparser.h" 
#include "dictionary.h" 
#include "global_server_variables.h"
#include "dio_handler.h"

int process_aux_commands(dictionary **dict_p,char * driver_type) {
        dictionary *aux=NULL; 
	char *command=NULL;  // pointer into dict do not free or remap
        char entry[200];       
	pthread_t thread;

        aux=*dict_p;
	sprintf(entry,"aux:command");
        command=iniparser_getstring(aux,entry,NULL);
	if (strcmp(driver_type,"DIO")==0) {
		/* First check to see if this command is meant for this driver */
		sprintf(entry,"%s","DIO");
		if(iniparser_find_entry(aux, entry)==1) { 
                  pthread_create(&thread,NULL,(void *) &DIO_aux_command,&aux);
		  pthread_join(thread,NULL);
		}	
                *dict_p=aux; 
        	return 1;
	}
        *dict_p=aux; 
        return 0;
}

