#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "utils.h"
#include "iniparser.h"

#define MAX_TSG 16
#define	MAX_TIME_SEQ_LEN 1048576
#define MAX_PULSES 100

int verbose=10;
int sock,msgsock;

dictionary *Site_INI;

void graceful_cleanup(int signum)
{
  close(msgsock);
  close(sock);
  exit(0);
}


int main(){
    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES
        int     maxclients=MAX_RADARS*MAX_CHANNELS;
        struct  ControlPRM  clients[maxclients],client ;
        struct  TSGbuf *pulseseqs[MAX_RADARS][MAX_CHANNELS][MAX_SEQS];
	unsigned int	*seq_buf[MAX_RADARS][MAX_CHANNELS];
        int seq_count[MAX_RADARS][MAX_CHANNELS];
        int ready_index[MAX_RADARS][MAX_CHANNELS];
	unsigned int	*master_buf;
        int old_seq_id=-10;
        int new_seq_id=-1;
        struct TRTimes bad_transmit_times;

	int	max_seq_count;

	// socket and message passing variables
	char	datacode;
	int	rval;
        fd_set rfds,efds;
	// counter and temporary variables
	int	i,j,r,c,buf,index;
	int 	temp;


	// function specific message variables
        int     numclients=0;
        struct  DriverMsg msg;
	// timing related variables
        struct timeval select_timeout;
        
        unsigned long counter;

        signal(SIGINT, graceful_cleanup);
        signal(SIGTERM, graceful_cleanup);

	Site_INI=NULL;
	/* Pull the site ini file */ 
	temp=_open_ini_file();
        if(temp < 0 ) {
                fprintf(stderr,"Error opening Site ini file, exiting driver\n");
                exit(temp);
        }

	/* Initialize the internal driver state variables */
	if (verbose > 1) printf("Zeroing arrays\n");

	/* These are useful for all drivers */
        max_seq_count=0;
	for (r=0;r<MAX_RADARS;r++){
	  for (c=0;c<MAX_CHANNELS;c++){
	    if (verbose > 1) printf("%d %d\n",r,c);
	    for (i=0;i<MAX_SEQS;i++) pulseseqs[r][c][i]=NULL;
            ready_index[r][c]=-1; 
            seq_buf[r][c]=malloc(4*MAX_TIME_SEQ_LEN);
           
          } 
        }
	/* These are only potentially needed for drivers that unpack 
 	*  that unpack the timing sequence. See the provided fake timing driver
 	*/
        master_buf=malloc(4*MAX_TIME_SEQ_LEN);
        bad_transmit_times.length=0;
        bad_transmit_times.start_usec=malloc(sizeof(unsigned int)*MAX_PULSES);
        bad_transmit_times.duration_usec=malloc(sizeof(unsigned int)*MAX_PULSES);
       

    // OPEN TCP SOCKET AND START ACCEPTING CONNECTIONS 
	sock=tcpsocket(TIMING_HOST_PORT);
	listen(sock, 5);
	while (1) {
                rval=1;
		msgsock=accept(sock, 0, 0);
		if (verbose > 0) printf("accepting socket!!!!!\n");
		if( (msgsock==-1) ){
			perror("accept FAILED!");
			return EXIT_FAILURE;
		}
		else while (rval>=0){
                  /* Look for messages from external client process */
                  FD_ZERO(&rfds);
                  FD_SET(msgsock, &rfds); //Add msgsock to the read watch
                  FD_ZERO(&efds);
                  FD_SET(msgsock, &efds);  //Add msgsock to the exception watch
                  /* Wait up to five seconds. */
                  select_timeout.tv_sec = 5;
                  select_timeout.tv_usec = 0;
		  if (verbose > 1) printf("%d Entering Select\n",msgsock);
                  rval = select(msgsock+1, &rfds, NULL, &efds, &select_timeout);
		  if (verbose > 1) printf("%d Leaving Select %d\n",msgsock,rval);
                  /* Don’t rely on the value of select_timeout now! */
                  if (FD_ISSET(msgsock,&efds)){
                    if (verbose > 1) printf("Exception on msgsock %d ...closing\n",msgsock);
                    break;
                  }
                  if (rval == -1) perror("select()");
                  rval=recv(msgsock, &buf, sizeof(int), MSG_PEEK); 
                  if (verbose>1) printf("%d PEEK Recv Msg %d\n",msgsock,rval);
		  if (rval==0) {
                    if (verbose > 1) printf("Remote Msgsock %d client disconnected ...closing\n",msgsock);
                    break;
                  } 
		  if (rval<0) {
                    if (verbose > 0) printf("Msgsock %d Error ...closing\n",msgsock);
                    break;
                  } 
                  if ( FD_ISSET(msgsock,&rfds) && rval>0 ) {
                    if (verbose>1) printf("Data is ready to be read\n");
		    if (verbose > 1) printf("%d Recv Msg\n",msgsock);
                    rval=recv_data(msgsock,&msg,sizeof(struct DriverMsg));
                    datacode=msg.type;
		    if (verbose > 1) printf("\nmsg code is %c\n", datacode);
		/* Process commands that have come in on the socket */	
		    switch( datacode ){
		      case REGISTER_SEQ:
		        if (verbose > 0) printf("\nRegister new timing sequence for timing card\n");	
		        msg.status=0;
                        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        r=client.radar-1; 
                        c=client.channel-1; 
		        rval=recv_data(msgsock,&index,sizeof(index));
		        if (verbose > 1) printf("Requested index: %d\n",index);	

			/*Prepare the memory pointers*/
                        if (pulseseqs[r][c][index]!=NULL) {
                          if (pulseseqs[r][c][index]->rep!=NULL)  free(pulseseqs[r][c][index]->rep);
                          if (pulseseqs[r][c][index]->code!=NULL) free(pulseseqs[r][c][index]->code);
                          free(pulseseqs[r][c][index]);
                        }
			/* Fill memory pointers */
                        pulseseqs[r][c][index]=malloc(sizeof(struct TSGbuf));
                        rval=recv_data(msgsock,pulseseqs[r][c][index], 
			  sizeof(struct TSGbuf)); // requested pulseseq
                        pulseseqs[r][c][index]->rep=
                          malloc(sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        pulseseqs[r][c][index]->code=
                          malloc(sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->rep, 
                          sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->code, 
                          sizeof(unsigned char)*pulseseqs[r][c][index]->len);

                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			/* Reset the local sequence state */
                        old_seq_id=-10;
                        new_seq_id=-1;
                        break;

		      case CtrlProg_END:
		        if (verbose > 0) printf("\nA client is done\n");	
                        msg.status=0;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			/* Reset the seq counters */
                        old_seq_id=-10;
                        new_seq_id=-1;
                        break;

		      case CtrlProg_READY:
		        if (verbose > 0) printf("\nAsking to set up driver info for client that is ready\n");	
                        msg.status=0;
		        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        r=client.radar-1; 
                        c=client.channel-1; 
                        if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
                          clients[ready_index[r][c]]=client;
                        } else {
                          clients[numclients]=client;
                          ready_index[r][c]=numclients;
                          numclients=(numclients+1);
                        }
			if (verbose > 1) printf("Radar: %d, Channel: %d Beamnum: %d Status %d\n",
			  client.radar,client.channel,client.tbeam,msg.status);	
                        index=client.current_pulseseq_index; 

                        if (numclients >= maxclients) msg.status=-2;
		        if (verbose > 1) printf("\nclient ready done\n");	
                        numclients=numclients % maxclients;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break; 

		      case PRETRIGGER:
			if(verbose > 1 ) printf("Setup Driver for next trigger\n");	
			/* Calculate sequence index */
                          msg.status=0;
                          new_seq_id=-1;
	                  for( i=0; i<numclients; i++) {
                            r=clients[i].radar-1;
                            c=clients[i].channel-1;
                            new_seq_id+=r*1000 +
                              c*100 +
                              clients[i].current_pulseseq_index+1;
                            if (verbose > 1) printf("%d %d %d\n",i,new_seq_id,clients[i].current_pulseseq_index); 
                          }
                          if (verbose > 1) printf("Timing Driver: %d %d\n",new_seq_id,old_seq_id);

			/* If sequence index has changed..repopulate the 
 			* master sequence.  Not needed for all drivers. */
                          if (new_seq_id!=old_seq_id) { 
                            max_seq_count=0;
                            for (i=0;i<numclients;i++) {
                              r=clients[i].radar-1;
                              c=clients[i].channel-1;
                              if (seq_count[r][c]>=max_seq_count) max_seq_count=seq_count[r][c];
                              counter=0;
                              for (j=0;j<seq_count[r][c];j++) {
                                if (i==0) {
                                  master_buf[j]=seq_buf[r][c][j];
                                  counter++;
                                }
                                else  {
			  	  master_buf[j]|=seq_buf[r][c][j];
				}
                              } 
			    }
                          }
                        

                        if (new_seq_id < 0 ) {
                          old_seq_id=-10;
                        }  else {
                          old_seq_id=new_seq_id;
                        }
                        new_seq_id=-1;
			/* If Timing Driver */
                        send_data(msgsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
                        send_data(msgsock, bad_transmit_times.start_usec, 
                                  sizeof(unsigned int)*bad_transmit_times.length);
                        send_data(msgsock, bad_transmit_times.duration_usec, 
                                  sizeof(unsigned int)*bad_transmit_times.length);

			msg.status=0;
                        if (verbose > 1)  printf("Ending Pretrigger Setup\n");
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break; 

		      case TRIGGER:
			if (verbose > 1 ) printf("Send Master Trigger\n");	
                        msg.status=0;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;

                      case EXTERNAL_TRIGGER:
                        if (verbose > 1 ) printf("Setup for external trigger\n");
                        msg.status=0;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      case WAIT:
			if (verbose > 1 ) printf("Driver Wait\n");	
                        msg.status=0;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			break;
		      case POSTTRIGGER:
                        numclients=0;
                        for (r=0;r<MAX_RADARS;r++){
                          for (c=0;c<MAX_CHANNELS;c++){
                            ready_index[r][c]=-1;
                          }
                        }
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      default:
			if (verbose > -10) fprintf(stderr,"BAD CODE: %c : %d\n",datacode,datacode);
			break;
		    }
		  }	
		} 
		if (verbose > 0 ) fprintf(stderr,"Closing socket\n");
		close(msgsock);
	};

        return 1;
}
