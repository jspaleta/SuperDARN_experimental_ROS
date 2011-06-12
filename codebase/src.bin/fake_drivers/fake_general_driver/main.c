#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <getopt.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "utils.h"
#include "iniparser.h"

#define MAX_TSG 16
#define	MAX_TIME_SEQ_LEN 1048576
#define MAX_PULSES 100

int verbose=0;
int sock,msgsock;

dictionary *Site_INI;

void graceful_cleanup(int signum)
{
  close(msgsock);
  close(sock);
  exit(0);
}

     
     
     

int main ( int argc, char **argv){
    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES
        int     maxclients=MAX_RADARS*MAX_CHANNELS;  //maximum number of clients which can be tracked
        struct  ControlPRM  clients[maxclients];	//parameter array for tracked clients
	struct  ControlPRM  client;			//parameter structure for temp use		
        struct  TSGbuf *pulseseqs[MAX_RADARS][MAX_CHANNELS][MAX_SEQS]; //timing sequence table
        int seq_count[MAX_RADARS][MAX_CHANNELS];			//unpacked seq length
	int	max_seq_count;				// maximum unpackage seq length	
	unsigned int	*seq_buf[MAX_RADARS][MAX_CHANNELS];		//unpacked sequence table
	unsigned int	*master_buf;			// master buffer for all sequences
        int ready_index[MAX_RADARS][MAX_CHANNELS];	//table to indicate if client is ready for trigger
        int old_seq_id=-10;				// old and new seq_id used to know if seqs have changed 
        int new_seq_id=-1;				//  and unpacking needs to be done again
        struct TRTimes bad_transmit_times;		// actual TR windows used
        int     numclients=0;				// number of active clients

	// socket and message passing variables
	char	datacode;	//command character
	int	rval;		//use for return values which indicate errors
        fd_set rfds,efds;	//TCP socket status
        struct timeval select_timeout;	// timeout for select function
	// counter and temporary variables
	int	i,j,r,c,buf,index;
	int 	temp;

        unsigned long counter;


	// function specific message variables
        struct  DriverMsg msg;	//msg structure which indicates command status and other things
        // argument parsing variables 
        int arg;		//command line option parsing
	char driver_type[20]="";
	int port_number=-1;

        signal(SIGINT, graceful_cleanup);
        signal(SIGTERM, graceful_cleanup);

	strcpy(driver_type,"TIMING");

	/* Process arguments */
        while (1)
        {
           static struct option long_options[] =
             {
               /* These options don't set a flag.
		* We distinguish them by their indices. */
               {"verbose",	no_argument,		0, 	'v'},
               {"driver",	required_argument, 	0, 	'd'},
               {"port",		required_argument, 	0, 	'p'},
               {"help",		required_argument, 	0, 	'h'},
               {0, 0, 0, 0}
             };
           /* getopt_long stores the option index here. */
           int option_index = 0;
     
           arg = getopt_long (argc, argv, "vhp:d:",
                            long_options, &option_index);
     
           /* Detect the end of the options. */
           if (arg == -1)
             break;
     
           switch (arg)
             {
             case 'd':
               printf ("option -d with value `%s'\n", optarg);
	       strcpy(driver_type,optarg);
              break;
             case 'p':
               port_number=atoi(optarg); 
               printf ("option -p with value `%s' %d\n", optarg,port_number);
               break;
     
             case 'v':
		verbose++; 	
               break;
     
	     case 'h':	
             case '?':
               printf ("Usage:\n" );
               printf ("Generic ROS driver for prototyping\n" );
               printf ("-h / --help : show this message\n");
               printf ("-v / --verbose : increase verbosity by 1\n");
               printf ("-d DRIVER / --driver DRIVER  : Select type of driver to mimic\n");
               printf ("DRIVER values: DDS,RECV,TIMING,DIO,GPS\n");
	       exit(0);	 
               break;
     
             default:
               abort();
             }
        }
	if(port_number <= 0 ) {
          if(strcmp(driver_type,"DDS")==0) {
		port_number=DDS_HOST_PORT;
	  }
          if(strcmp(driver_type,"TIMING")==0) {
		port_number=TIMING_HOST_PORT;
	  }
          if(strcmp(driver_type,"DIO")==0) {
		port_number=DIO_HOST_PORT;
	  }
          if(strcmp(driver_type,"GPS")==0) {
		port_number=GPS_HOST_PORT;
	  }
          if(strcmp(driver_type,"RECV")==0) {
		port_number=RECV_HOST_PORT;
	  }
        }
	     
	printf("Driver Type:  %s Port: %d \n",driver_type,port_number);
       /* Instead of reporting ‘--verbose’
 *           and ‘--brief’ as they are encountered,
 *                     we report the final status resulting from them. */

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
			/* REGISTER_SEQ: Control programs request pulse sequences to use and pass them
 			* to the ROS server process. The ROS server process in turn passes the sequences
 			* to each driver to use as needed for hardware configuration */ 
		        if (verbose > 0) printf("\nDriver: Register new timing sequence\n");	
			/* Inform the ROS that this driver does handle this command, and its okay to send the
 			* sequence data by sending msg back with msg.status=1.
 			*/
		        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			/* Now recv the expected data as per the documented API for this command */
                        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        r=client.radar-1; 
                        c=client.channel-1; 
		        rval=recv_data(msgsock,&index,sizeof(index));
		        if (verbose > 1) printf("Driver: Requested sequence index: %d\n",index);	

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

			/* Inform the ROS that this driver recv all data without error
 			* by sending msg back with msg.status=1.
 			*/
		        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			/* Reset the local sequence state */
                        old_seq_id=-10;
                        new_seq_id=-1;
                        break;

		      case CtrlProg_END:
			/* CtrlProg_END: When Control Programs disconnect from the ROS server, the
 			* the ROS server informs each driver so drivers can reset internal state variables.
 			*/  
		        if (verbose > 0) printf("\nDriver: client is done\n");	
			/* Inform the ROS that this driver does handle this command, and its okay to send the
 			* associated data by sending msg back with msg.status=1.
 			*/
                        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			/* Reset the seq counters */
                        old_seq_id=-10;
                        new_seq_id=-1;
                        break;

		      case CtrlProg_READY:
			/* CtrlProg_READY: When Control Programs inform the ROS server they are ready for
 			* the next trigger, the ROS server informs each driver so drivers can reset internal 
 			* state variables.
 			*/  
		        if (verbose > 0) printf("\nDriver: client that is ready\n");	
			/* Inform the ROS that this driver does handle this command, and its okay to send the
 			* associated data by sending msg back with msg.status=1.
 			*/
                        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));

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
                        numclients=numclients % maxclients;

			/* Inform the ROS that this driver recv all data without error
 			* by sending msg back with msg.status=1.
 			*/
			msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break; 

		      case PRETRIGGER:
			/* PRETRIGGER: When all controlprograms are ready, the ROS server will instruct all drivers 
 			* to do whatever pretrigger operations are necessary prior to the next trigger event.
 			*/  
			if(verbose > 1 ) printf("Driver: Pre-trigger setup\n");	
			/* Inform the ROS that this driver does handle this command, and its okay to send the
 			* associated data by sending msg back with msg.status=1.
 			*/
                        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			/* Calculate sequence index */
                          new_seq_id=-1;
	                  for( i=0; i<numclients; i++) {
                            r=clients[i].radar-1;
                            c=clients[i].channel-1;
                            new_seq_id+=r*1000 +
                              c*100 +
                              clients[i].current_pulseseq_index+1;
                            if (verbose > 1) printf("%d %d %d\n",i,new_seq_id,clients[i].current_pulseseq_index); 
                          }
                          if (verbose > 1) printf("Driver: %d %d\n",new_seq_id,old_seq_id);

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

                        msg.status=1;
                        if (verbose > 1)  printf("Driver: Ending Pretrigger Setup\n");
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break; 

		      case TRIGGER:
			/* TRIGGER: The ROS may instruct drivers to issue a trigger event. Typically only
 			* the timing driver will be expected to do something when this command is issued.
 			* Other drivers will typically ignore this command and will rely on signals from
 			* the timing card.
 			*/  
			if (verbose > 1 ) printf("Driver: Send Master Trigger\n");	
                        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;

                      case EXTERNAL_TRIGGER:
			/* EXTERNAL_TRIGGER: The ROS may instruct drivers to setup for an external trigger event.
 			*  Typically only the timing driver will be expected to do something when 
 			*  this command is issued.  Other drivers will typically ignore this command and will relyi
 			*   on signals from the timing card.
 			*/  
                        if (verbose > 1 ) printf("Driver: Setup for external trigger\n");
                        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      case WAIT:
			/* WAIT: After a trigger or external trigger command has been issued. The ROS 
			*  may issue this command to a driver as a way to wait for pulse sequence operations
			*  to be complete. Drivers are expected to block until pulse sequence operations are done.
 			*/  
			if (verbose > 1 ) printf("Driver: Wait\n");	
			/* Driver would put the logic necessary to block waiting for a sequence operation to complete*/
                        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			break;
		      case POSTTRIGGER:
			/* POSTTRIGGER: After a trigger or external trigger event, the ROS server may 
 			* instruct all drivers to do whatever posttrigger operations are necessary to clean 
 			* internal driver state.
 			*/  
                        if (verbose > 1)  printf("Driver: Post-trigger Setup\n");
                        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        numclients=0;
                        for (r=0;r<MAX_RADARS;r++){
                          for (c=0;c<MAX_CHANNELS;c++){
                            ready_index[r][c]=-1;
                          }
                        }
                        msg.status=1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        if (verbose > 1)  printf("Driver: Ending Post-trigger Setup\n");
                        break;
		      case GET_TRTIMES:
			/* GET_TRTIMES: After a trigger or external trigger command has been issued. The ROS 
			*  may issue this command to a driver as a way to retrieve information about the
			*  actual TR time windows that were used. 
			*  The timing driver is the only driver that should respond to this command
 			*/  
			if (verbose > 1 ) printf("Driver: GET_TRTIMES\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
                        msg.status=0;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			break;
		      case GET_DATA:
			/* GET_DATA: After a trigger or external trigger command has been issued. The ROS 
			*  may issue this command to a driver as a way to retrieve sample data. 
			*  The receiver driver is the only driver that should respond to this command
 			*/  
			if (verbose > 1 ) printf("Driver: GET_DATA\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
                        msg.status=0;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			break;
		      case AUX_COMMAND:
			/* AUX_COMMAND: Site hardware specific commands which are not critical for operation, but
 			*  controlprograms may optionally access to if they are site aware.  
 			*/  
			if (verbose > 1 ) printf("Driver: AUX_COMMAND\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
                        msg.status=0;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
		      default:
			/* NOOPs: ROS commands that are not understood by the driver should send a msg.status=0  
			* Some drivers will not need to process all of the named commands listed above. 
			* For those driver the default case can be used and the ROS will deal with it accordingly.
 			*/  
			if (verbose > -10) printf("Driver: BAD CODE: %c : %d\n",datacode,datacode);
                        msg.status=-1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			break;
		    }
		  }	
		} 
		if (verbose > 0 ) fprintf(stderr,"Closing socket\n");
		close(msgsock);
	};

        return 1;
}
