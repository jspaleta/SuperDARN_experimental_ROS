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

	if (strcmp(in_msg.driver,"DIO")==0) {
                /* First check to see if this command is meant for this driver */
                  if(strcmp(in_msg.command_name,"GET_TX_STATUS")==0) {
                    for (r=0;r<MAX_RADARS;r++) {
                      for (i=0;i<MAX_TRANSMITTERS;i++) {
                        txstatus[r].LOWPWR[i]=100*(r+1)+i;
                        txstatus[r].AGC[i]=1000*(r+1)+i;
                        txstatus[r].status[i]=10000*(r+1)+i;
                      }
                    }
		    retval=driver_msg_get_var_by_name(&in_msg,"radar",&radar);
                    r=radar-1;
                    if((r<0) || (r >=MAX_RADARS) || (retval < 0)) {
                      fprintf(stderr,"AUX: DIO: GET_TX_STATUS: bad radar number %d %d\n",radar,MAX_RADARS);
                    } else {
                      bytes=sizeof(struct tx_status);
                      retval=driver_msg_add_var(&return_msg,&txstatus,bytes,"txstatus","struct tx_status"); 
                      if((retval < 0)) {
                        fprintf(stderr,"AUX: DIO: GET_TX_STATUS: unable to add txstatus to return_msg\n");
		      }	
                    }
                  }
	}
        memmove(out_msg_p,&return_msg,sizeof(struct DriverMsg));
        return 0;
}


