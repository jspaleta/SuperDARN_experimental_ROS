#include <ctype.h>
#include "iniparser.h" 
#include "dictionary.h" 
#include "global_server_variables.h"

int process_aux_commands(dictionary *aux,char * driver_type) {
	char *command=NULL;  // pointer into dict do not free or remap
	//char *buf=NULL;  // pointer into dict do not free or remap
	char *obuf=NULL;  // malloced pointer that needs to be free'd 
        char entry[200];       
        char value[200];       
	int  i,radar=-10,r=-1,bytes;
	unsigned int bufsize; 
        struct tx_status txstatus[MAX_RADARS];
        //struct tx_status *txp=NULL;
	sprintf(entry,"command");
        command=iniparser_getstring(aux,entry,NULL);

	if (strcmp(driver_type,"DIO")==0) {
		/* First check to see if this command is meant for this driver */
		sprintf(entry,"%s","dio");
		if(iniparser_find_entry(aux, entry)==1) { 
		  printf("AUX: DIO command\n");
		  if(iniparser_find_entry(aux, "data")==1) { 
		    printf("AUX: DIO command has a data buffer\n");
                    //buf=dictionary_getbuf(aux,"data",&bufsize);  
		    /* Removing the data entry now that we have pulled the buffer */
                    iniparser_unset(aux,"data:bytes");
                    iniparser_unset(aux,"data");
		  }
                  //iniparser_dump_ini(aux,stdout);
		  /* Process different command strings here */
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
		      sprintf(entry,"%s:bytes","dio");
		      sprintf(value,"%d",bytes);
		      iniparser_set(aux,entry,value,NULL);
		      iniparser_setbuf(aux,"dio",obuf,bytes);
                      //txp=dictionary_getbuf(aux,"dio",&bufsize);
 
		      //for (i=0;i<MAX_TRANSMITTERS;i++) {
                      //  printf("%d : %d %d %d\n",i,
                      //    txp->LOWPWR[i],
                      //    txp->AGC[i],
                      //    txp->status[i]);
                      //}

		      if (obuf!=NULL) free(obuf);
                      obuf=NULL;
              	    }
		  }
		}	
          return 1;
	}
        return 0;
}

