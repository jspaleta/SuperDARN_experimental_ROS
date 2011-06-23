#include <ctype.h>
#include "iniparser.h" 
#include "dictionary.h" 
#include "global_server_variables.h"


int process_aux_msg(struct DriverMsg in_msg,struct DriverMsg *out_msg_p) {
        int retval;
	int i,r,radar=0;	
        int bytes;
        struct tx_status txstatus[MAX_RADARS];
        struct DriverMsg return_msg;
	driver_msg_init(&return_msg);

        printf("AUX: %s: %s\n",in_msg.driver,in_msg.command_name);
	if (strcmp(in_msg.driver,"DIO")==0) {
                /* First check to see if this command is meant for this driver */
                  if(strcmp(in_msg.command_name,"GET_TX_STATUS")==0) {
		    printf("H 1\n");
                    for (r=0;r<MAX_RADARS;r++) {
                      for (i=0;i<MAX_TRANSMITTERS;i++) {
                        txstatus[r].LOWPWR[i]=100*(r+1)+i;
                        txstatus[r].AGC[i]=1000*(r+1)+i;
                        txstatus[r].status[i]=10000*(r+1)+i;
                      }
                    }
		    printf("H 2\n");
		    retval=driver_msg_get_var_by_name(&in_msg,"radar",&radar);
		    printf("H 3\n");
                    r=radar-1;
                    if((r<0) || (r >=MAX_RADARS) || (retval < 0)) {
                      printf("AUX: DIO: GET_TX_STATUS: bad radar number %d %d\n",radar,MAX_RADARS);
                    } else {
                      printf("AUX: DIO: GET_TX_STATUS: valid radar %d\n",radar);
                      bytes=sizeof(struct tx_status);
                      retval=driver_msg_add_var(&return_msg,&txstatus,bytes,"txstatus","struct tx_status"); 
                      if((retval < 0)) {
                        printf("AUX: DIO: GET_TX_STATUS: unable to add txstatus to return_msg\n");
		      }	
                    }
                  }
	}
        memmove(out_msg_p,&return_msg,sizeof(struct DriverMsg));
        return 0;
}

int process_aux_commands(dictionary **dict_p,char * driver_type) {
        dictionary *aux;
	char *command=NULL;  // pointer into dict do not free or remap
	//char *buf=NULL;  // pointer into dict do not free or remap
	char *obuf=NULL;  // malloced pointer that needs to be free'd 
        char entry[200];       
        char value[200];       
	int  i,radar=-10,r=-1,bytes;
	unsigned int bufsize; 
        struct tx_status txstatus[MAX_RADARS];
        aux=*dict_p;
	sprintf(entry,"aux:command");
        command=iniparser_getstring(aux,entry,NULL);

	if (strcmp(driver_type,"DIO")==0) {
		/* First check to see if this command is meant for this driver */
		sprintf(entry,"%s","dio");
		if(iniparser_find_entry(aux, entry)==1) { 
		  printf("AUX: DIO command\n");
		  if(strcmp(command,"GET_TX_STATUS")==0) {
                    for (r=0;r<MAX_RADARS;r++) {
                      for (i=0;i<MAX_TRANSMITTERS;i++) {
                        txstatus[r].LOWPWR[i]=100*(r+1)+i;
                        txstatus[r].AGC[i]=1000*(r+1)+i;
                        txstatus[r].status[i]=10000*(r+1)+i;
                      }
		    } 	
		    printf("AUX: DIO: GET_TX_STATUS command\n");
		    sprintf(entry,"%s:radar","dio");
		    radar=iniparser_getint(aux,entry,0);
		    r=radar-1;
		    if((r<0) || (r >=MAX_RADARS)) {
		      printf("AUX: DIO: GET_TX_STATUS: bad radar number %d %d\n",radar,MAX_RADARS);
		    } else {
		      printf("AUX: DIO: GET_TX_STATUS: valid radar %d\n",radar);
		      bytes=sizeof(struct tx_status);	
		      if (obuf!=NULL) free(obuf);
                      obuf=malloc(bytes); 
		      memmove(obuf,&txstatus[r],bytes);
		      iniparser_setbuf(aux,"dio",obuf,bytes);
		      if (obuf!=NULL) free(obuf);
                      obuf=NULL;
              	    }
		    printf("AUX: DIO: GET_TX_STATUS Done %d\n",iniparser_getbufsize(aux,"dio"));
		  }
		}	
          *dict_p=aux;
          return 1;
	}
        *dict_p=aux;
        return 0;
}

