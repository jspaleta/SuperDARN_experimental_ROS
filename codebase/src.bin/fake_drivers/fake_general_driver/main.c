#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <getopt.h>
#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"
#include "site_defaults.h"
#include "utils.h"
#include "iniparser.h"
#include "dictionary.h"
#include "driver_structures.h"


int verbose=0;
int sock,msgsock;
dictionary *diagnostic_INI=NULL;


char* elapsed_string(struct timeval t0,struct timeval t1) {
  static char elapsed_secs_string[80]; 
  double elapsed_secs;
  elapsed_secs=t1.tv_sec+(double)t1.tv_usec/1E6;
  elapsed_secs-=t0.tv_sec+(double)t0.tv_usec/1E6;
  sprintf(elapsed_secs_string,"%g",elapsed_secs);
  return elapsed_secs_string;
}

void graceful_cleanup(int signum){
  fprintf(stdout,"\n\n");
  iniparser_dump_ini(diagnostic_INI,stdout);
  close(msgsock);
  close(sock);
  exit(0);
}

int main ( int argc, char **argv){
    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES
	/* Commonly needed variables */
	int32	bufnum=0,radar=0,channel=0,data_status=0;
	struct SiteSettings site_settings;	
        dictionary *Site_INI=NULL;
        char ini_filename[80]="";
	t_driver  dinfo;
        int     maxclients=MAX_RADARS*MAX_CHANNELS;  //maximum number of clients which can be tracked
        struct  ControlPRM  clients[maxclients];	//parameter array for tracked clients
	struct  ControlPRM  client;			//parameter structure for temp use		
	/* CLRSEARCH related variables */
	struct  CLRFreqPRM clrfreqsearch;
	double *pwr=NULL;
	/* GET_DATA related variables */
        struct  DataPRM	    data;
        uint32  *main_data=NULL,*back_data=NULL;
	/* REGISTER_SEQ related variables */
        struct  TSGbuf *pulseseqs[MAX_RADARS+1][MAX_CHANNELS+1][MAX_SEQS+1]; //timing sequence table
        int seq_count[MAX_RADARS+1][MAX_CHANNELS+1];			//unpacked seq length
	int	max_seq_count;				// maximum unpackage seq length	
	unsigned int	*seq_buf[MAX_RADARS+1][MAX_CHANNELS+1];		//unpacked sequence table
	unsigned int	*master_buf;			// master buffer for all sequences

	/* READY related variables */
        int ready_index[MAX_RADARS+1][MAX_CHANNELS+1];	//table to indicate if client is ready for trigger
        int old_seq_id=-10;				// old and new seq_id used to know if seqs have changed 
        int new_seq_id=-1;				//  and unpacking needs to be done again
        int numclients=0;				// number of active clients

	/* TRTimes related variables */
        struct TRTimes transmit_times;		// actual TR windows used
	/* GPS AUX variables */
	int32 gps_event,gpssecond,gpsnsecond;
 
	// socket and message passing variables
	char	datacode;	//command character
	int	rval;		//use for return values which indicate errors
        fd_set rfds,efds;	//TCP socket status
        struct timeval select_timeout;	// timeout for select function
	static char   command_string[40];
	static char   entry_string[80];
	// counter and temporary variables
	int	i,j,r,c;
        int32   index[3];
	int 	temp;
	int32 	temp32;
        unsigned long counter;
	struct timeval t0,t1;
	int strlength=0;
	char *time_string;
	char value[120];
	char err_string[120]="";
	int err_val=0;


	// function specific message variables
        struct  ROSMsg msg;	//msg structure which indicates command status and other things
	struct	ROSMsg r_msg;
        // argument parsing variables 
        int arg;		//command line option parsing

        static struct option long_options[] =
        {
               /* These options don't set a flag.
		* We distinguish them by their indices. */
               {"verbose",	no_argument,		0, 	'v'},
               {"driver",	required_argument, 	0, 	'd'},
               {"config",	required_argument, 	0, 	'c'},
               {"help",		required_argument, 	0, 	'h'},
               {0, 0, 0, 0}
        };

        signal(SIGINT, graceful_cleanup);
        signal(SIGTERM, graceful_cleanup);

	strcpy(dinfo.type,"TIMING");

	/* Process arguments */
        while (1)
        {
           /* getopt_long stores the option index here. */
           int option_index = 0;
     
           arg = getopt_long (argc, argv, "vhc:d:",
                            long_options, &option_index);
     
           /* Detect the end of the options. */
           if (arg == -1)
             break;
     
           switch (arg)
             {
             case 'd':
               printf ("option -d with value `%s'\n", optarg);
	       strcpy(dinfo.type,optarg);
              break;
             case 'c':
               if(strlen(optarg) < 80 ) { 
                 strncpy(ini_filename,optarg,80); 
                 printf ("option -c with value `%s'\n", optarg);
               } else printf ("option -c config filename longer than 80 characters\n");
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
               printf ("-c FILE / --config FILE  : Filepath for ini file\n");
               printf ("-p PORT / --port PORT  : port number to use\n");
               printf ("-d DRIVER / --driver DRIVER  : Select type of driver to mimic\n");
               printf ("DRIVER values: DDS,RECV,TIMING,DIO,GPS\n");
	       exit(0);	 
               break;
     
             default:
               abort();
             }
        }
	Site_INI=NULL;
	/* Pull the site ini file */ 
        if(strlen(ini_filename)==0) { 
          fprintf(stdout,"Using default site.ini location\n");
          sprintf(ini_filename,"%s/site.ini",SITE_DIR);
        }
	temp=_open_ini_file(&Site_INI,ini_filename);
        if(temp < 0 ) {
                fprintf(stderr,"Error opening Site ini file, exiting driver\n");
                exit(temp);
        }
        if(verbose > 0) {
          fprintf(stdout,"Site wide settings:\n");
          _dump_ini_section(stdout,Site_INI,"site");
          fprintf(stdout,"Frequency assignment settings:\n");
          _dump_ini_section(stdout,Site_INI,"frequency_assignment");
          fprintf(stdout,"%s Driver settings:\n",dinfo.type);
          _dump_ini_section(stdout,Site_INI,dinfo.type);
        }
	/* Driver specific values from Ini settings */
        _configure_driver(Site_INI,&dinfo);

	printf("Driver Type:  %s Port: %d \n",dinfo.type,dinfo.tcp.port_number);
	printf("Verbose Level:  %d \n",verbose);


        diagnostic_INI=dictionary_new(0);
        iniparser_set(diagnostic_INI,dinfo.type, NULL,NULL);

	/* Initialize the internal driver state variables */
	if (verbose > 1) printf("Zeroing arrays\n");

	/* These are useful for all drivers */
        max_seq_count=0;
	for (r=0;r<=MAX_RADARS;r++){
	  for (c=0;c<=MAX_CHANNELS;c++){
	    if (verbose > 1) printf("%d %d\n",r,c);
	    for (i=0;i<=MAX_SEQS;i++) pulseseqs[r][c][i]=NULL;
            ready_index[r][c]=-1; 
            seq_buf[r][c]=malloc(4*dinfo.max_seq_length);
            memset(seq_buf[r][c],0,4*dinfo.max_seq_length); 
            seq_count[r][c]=0;
          } 
        }

	/* These are only potentially needed for drivers that unpack 
 	*  that unpack the timing sequence. See the provided fake timing driver
 	*/
        master_buf=malloc(4*dinfo.max_seq_length);
        transmit_times.length=0;
        transmit_times.start_usec=malloc(sizeof(unsigned int)*dinfo.max_pulses);
        transmit_times.duration_usec=malloc(sizeof(unsigned int)*dinfo.max_pulses);
       
    // OPEN TCP SOCKET AND START ACCEPTING CONNECTIONS 
	sock=tcpsocket(dinfo.tcp.port_number);
	listen(sock, 5);
	while (1) {
                ros_msg_init(&msg);
                ros_msg_init(&r_msg);
		sprintf(command_string,"UNKNOWN");
	  	r_msg.status=0;
                rval=1;
		msgsock=accept(sock, 0, 0);
		if (verbose > 0) printf("accepting socket!!!!!\n");
		if( (msgsock==-1) ){
			perror("accept FAILED!");
			return EXIT_FAILURE;
		}
		else while (rval>=0){
	          strcpy(err_string,"");
	          err_val=0;
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
                  rval=recv(msgsock, &temp, sizeof(int), MSG_PEEK); 
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
                    ros_msg_recv(msgsock,&msg);
                    datacode=msg.command_type;
		    if (verbose > 1) printf("\nmsg code is %c\n", datacode);
                    gettimeofday(&t0,NULL);
		/* Process commands that have come in on the socket */	
		    switch( datacode ){

/* Commands which can be serviced by multiple drivers */

		      case REGISTER_SEQ:
		        sprintf(command_string,"REGISTER_SEQ");
			/* REGISTER_SEQ: Control programs request pulse sequences to use and pass them
 			* to the ROS server process. The ROS server process in turn passes the sequences
 			* to each driver to use as needed for hardware configuration */ 
		        if (verbose > 0) printf("\nDriver: Register new timing sequence\n");	
			/* Inform the ROS that this driver does handle this command, and its okay to send the
 			* sequence data by sending msg back with msg.status=1.
 			*/
		        r_msg.status=0;
			rval=ros_msg_get_var_by_name(&msg,"parameters",&client);
			rval=ros_msg_get_var_by_name(&msg,"index",index);
		        if (verbose > 1) printf("Driver: Requested sequence index: %d %d %d\n",index[0],index[1],index[2]);	
			/*Prepare the memory pointers*/
                        if (pulseseqs[index[0]][index[1]][index[2]]!=NULL) {
                          if (pulseseqs[index[0]][index[1]][index[2]]->rep!=NULL)free(pulseseqs[index[0]][index[1]][index[2]]->rep);
                          if (pulseseqs[index[0]][index[1]][index[2]]->code!=NULL)free(pulseseqs[index[0]][index[1]][index[2]]->code);
                          free(pulseseqs[index[0]][index[1]][index[2]]);
                        }
			/* Fill memory pointers */
                        pulseseqs[index[0]][index[1]][index[2]]=malloc(sizeof(struct TSGbuf));
			rval=ros_msg_get_var_by_name(&msg,"pulseseq",pulseseqs[index[0]][index[1]][index[2]]);
                        pulseseqs[index[0]][index[1]][index[2]]->rep=
                            malloc(sizeof(unsigned char)*pulseseqs[index[0]][index[1]][index[2]]->len);
                        pulseseqs[index[0]][index[1]][index[2]]->code=
                            malloc(sizeof(unsigned char)*pulseseqs[index[0]][index[1]][index[2]]->len);
		        if (verbose > 1) printf("Driver: Done Filling the memory\n");	
			rval=ros_msg_get_var_by_name(&msg,"rep",pulseseqs[index[0]][index[1]][index[2]]->rep);
			rval=ros_msg_get_var_by_name(&msg,"code",pulseseqs[index[0]][index[1]][index[2]]->code);
			/* Inform the ROS that this driver recv all data without error
 			* by sending msg back with msg.status=1.
 			*/
		        r_msg.status=1;
			/* Reset the local sequence state */
                        old_seq_id=-10;
                        new_seq_id=-1;
                        break;

		      case CtrlProg_END:
		        sprintf(command_string,"CtrlProg_END");
			/* CtrlProg_END: When Control Programs disconnect from the ROS server, the
 			* the ROS server informs each driver so drivers can reset internal state variables.
 			*/  
		        if (verbose > 0) printf("\nDriver: client is done\n");	
			rval=ros_msg_get_var_by_name(&msg,"parameters",&client);
			/* Inform the ROS that this driver does handle this command, and its okay to send the
 			* associated data by sending msg back with msg.status=1.
 			*/
                        old_seq_id=-10;
                        new_seq_id=-1;
                        break;

		      case CtrlProg_READY:
		        sprintf(command_string,"CtrlProg_READY");
			/* CtrlProg_READY: When Control Programs inform the ROS server they are ready for
 			* the next trigger, the ROS server informs each driver so drivers can reset internal 
 			* state variables.
 			*/  
		        if (verbose > 0) printf("\nDriver: client that is ready\n");	
			/* Inform the ROS that this driver does handle this command, and its okay to send the
 			* associated data by sending msg back with msg.status=1.
 			*/
			rval=ros_msg_get_var_by_name(&msg,"parameters",&client);
			r_msg.status=1;
			if (verbose > 1) printf("Radar: %d, Channel: %d Beamnum: %d Status %d\n",
			    client.radar,client.channel,client.tbeam,msg.status);	
                        r=client.radar-1; 
                        c=client.channel-1; 
                        if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
                            clients[ready_index[r][c]]=client;
                        } else {
                            clients[numclients]=client;
                            ready_index[r][c]=numclients;
                            numclients++;
                        }

                        if (numclients >= maxclients) msg.status=-2;
                        numclients=numclients % maxclients;
			/* Inform the ROS that this driver recv all data without error
 			* by sending msg back with msg.status=1.
 			*/
                        break; 

		      case PRETRIGGER:
		        sprintf(command_string,"CtrlProg_PRETRIGGER");
			/* PRETRIGGER: When all controlprograms are ready, the ROS server will instruct all drivers 
 			* to do whatever pretrigger operations are necessary prior to the next trigger event.
 			*/  
			if(verbose > 1 ) printf("Driver: Pre-trigger setup\n");	
			/* Inform the ROS that this driver does handle this command, and its okay to send the
 			* associated data by sending msg back with msg.status=1.
 			*/
                        r_msg.status=1;
			/* Calculate sequence index */
                        new_seq_id=-1;
	                for( i=0; i<numclients; i++) {
                          memmove(index,clients[i].pulseseq_index,sizeof(int32)*3); 
                          new_seq_id+=index[0]*1000 +
                            index[1]*100 +
                            index[0]+1;
                          if (verbose > 1) printf("%d : %d : %d %d %d\n",i,new_seq_id,index[0],index[1],index[2]); 
                        }
                        if (verbose > 1) printf("Driver: %d %d\n",new_seq_id,old_seq_id);

			/* If sequence index has changed..repopulate the 
 			* master sequence.  Not needed for all drivers. */
			/* TODO : Fill the seq_buf here */
                        if (new_seq_id!=old_seq_id) { 
			    if (verbose > 0) printf("\nDriver: Unpack the pulse sequences \n");	
                            max_seq_count=0;
                            for (i=0;i<numclients;i++) {
                              r=clients[i].radar-1;
                              c=clients[i].channel-1;
			      seq_count[r][c]=unpack_sequence(r,c,dinfo,pulseseqs[index[0]][index[1]][index[2]],seq_buf[r][c]);	
                              if (seq_count[r][c]>=max_seq_count) max_seq_count=seq_count[r][c];
                              counter=0;
                              for (j=0;j<seq_count[r][c];j++) {
                                if (i==0) {
                                  master_buf[j]=seq_buf[r][c][j];
                                  counter++;
                                } else {
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

                        if (verbose > 1)  printf("Driver: Ending Pretrigger Setup\n");
                        break; 

		      case TRIGGER:
		        sprintf(command_string,"CtrlProg_TRIGGER");
			/* TRIGGER: The ROS may instruct drivers to issue a trigger event. Typically only
 			* the timing driver will be expected to do something when this command is issued.
 			* Other drivers will typically ignore this command and will rely on signals from
 			* the timing card.
 			*/  
			if (verbose > 1 ) printf("Driver: Send Master Trigger\n");	
          		if(strcmp(dinfo.type,"TIMING")==0) r_msg.status=1;
	  		else r_msg.status=0;
                        break;

                      case EXTERNAL_TRIGGER:
		        sprintf(command_string,"EXTERNAL_TRIGGER");
			/* EXTERNAL_TRIGGER: The ROS may instruct drivers to setup for an external trigger event.
 			*  Typically only the timing driver will be expected to do something when 
 			*  this command is issued.  Other drivers will typically ignore this command and will relyi
 			*   on signals from the timing card.
 			*/  
                        if (verbose > 1 ) printf("Driver: Setup for external trigger\n");
          		if(strcmp(dinfo.type,"TIMING")==0) r_msg.status=1;
	  		else r_msg.status=0;
                        break;

		      case WAIT:
		        sprintf(command_string,"WAIT");
			/* WAIT: After a trigger or external trigger command has been issued. The ROS 
			*  may issue this command to a driver as a way to wait for pulse sequence operations
			*  to be complete. Drivers responding to this command are expected to block until 
			*  pulse sequence operations are done.
 			*/  
			if (verbose > 1 ) printf("Driver: Wait\n");	
			/* Driver would put the logic necessary to block waiting for a sequence operation to complete*/
                        r_msg.status=1;
			break;

		      case POSTTRIGGER:
		        sprintf(command_string,"POSTTRIGGER");
			/* POSTTRIGGER: After a trigger or external trigger event, the ROS server may 
 			* instruct all drivers to do whatever posttrigger operations are necessary to clean 
 			* internal driver state.
 			*/  
                        if (verbose > 1)  printf("Driver: Post-trigger Setup\n");
                        r_msg.status=1;
                        numclients=0;
                        for (r=0;r<MAX_RADARS;r++){
                          for (c=0;c<MAX_CHANNELS;c++){
                            ready_index[r][c]=-1;
                          }
                        }
                        if (verbose > 1)  printf("Driver: Ending Post-trigger Setup\n");
                        break;

		      case SITE_SETTINGS:
		        sprintf(command_string,"SITE_SETTINGS");
			/* SITE_SETTINGS: The ROS may issue this command to a driver. 
			*   
 			*/  
			if (verbose > 1 ) printf("Driver: SITE_SETTINGS\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"DIO")==0) {
				rval=ros_msg_get_var_by_name(&msg,"ifmode",&site_settings.ifmode);
				rval=ros_msg_get_var_by_name(&msg,"rf_rxfe_settings",&site_settings.rf_settings);
				rval=ros_msg_get_var_by_name(&msg,"if_rxfe_settings",&site_settings.if_settings);
			}
	  		else msg.status=0;
			break;

		      case CLRSEARCH_READY:
		        sprintf(command_string,"CLRSEARCH_READY");
			/* CLRSEARCH_READY: The ROS may issue this command to all drivers, 
			*  prior to doing a clear frequency search to hand over clear search
			*  parameters for a specific channel. 
 			*/  
			if (verbose > 1 ) printf("Driver: CLRFREQ_READY\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"DIO")==0) {
				r_msg.status=1;
				rval=ros_msg_get_var_by_name(&msg,"clrfreqsearch",&clrfreqsearch);
				rval=ros_msg_get_var_by_name(&msg,"radar",&radar);
				rval=ros_msg_get_var_by_name(&msg,"channel",&channel);
			}
          		else if(strcmp(dinfo.type,"RECV")==0) {
				r_msg.status=1;
				rval=ros_msg_get_var_by_name(&msg,"clrfreqsearch",&clrfreqsearch);
				rval=ros_msg_get_var_by_name(&msg,"radar",&radar);
				rval=ros_msg_get_var_by_name(&msg,"channel",&channel);
			}
	  		else r_msg.status=0;
			break;

		      case PRE_CLRSEARCH:
		        sprintf(command_string,"PRE_CLRSEARCH");
			/* PRE_CLRFREQ: The ROS may issue this command to all drivers, 
			*  prior to doing a clear frequency search 
 			*/  
			if (verbose > 1 ) printf("Driver: PRE_CLRFREQ\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"DIO")==0) {
				r_msg.status=1;
			}
	  		else r_msg.status=0;
			break;

		      case POST_CLRSEARCH:
		        sprintf(command_string,"POST_CLRSEARCH");
			/* POST_CLRFREQ: The ROS may issue this command to all drivers, 
			*  prior to doing a clear frequency search 
 			*/  
			if (verbose > 1 ) printf("Driver: POST_CLRFREQ\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"DIO")==0) {
				r_msg.status=1;
			}
	  		else r_msg.status=0;
			break;

		      case CLRSEARCH:
		        sprintf(command_string,"CLRSEARCH");
			/* CLRSEARCH: The ROS may issue this command to all drivers, 
			*  to do a clear frequency search 
 			*/  
			if (verbose > 1 ) printf("Driver: CLRFREQ\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"RECV")==0) {
				r_msg.status=1;
  				ros_msg_add_var(&r_msg,&clrfreqsearch,sizeof(struct CLRFreqPRM),"clrfreqsearch","struct CLRFreqRPM");
				temp32=clrfreqsearch.freq_end_khz-clrfreqsearch.freq_start_khz;

				pwr= (double*) malloc(sizeof(double) *temp32);

				for(i=0;i<temp32;i++) {
				  pwr[i]=rand();
				  if(verbose > 3 ) printf("%8d : %8d :: %8.3lf\n",i,clrfreqsearch.freq_start_khz+i,pwr[i]);
				}
				ros_msg_add_var(&r_msg,&temp32,sizeof(int32),"N","int32");
				ros_msg_add_var(&r_msg,pwr,sizeof(double)*temp32,"pwr_per_khz","array of doubles");
			}
	  		else r_msg.status=0;
			free(pwr);
			pwr=NULL;
			break;

		      case AUX_COMMAND:
		        sprintf(command_string,"AUX_COMMAND");
			/* AUX_COMMAND: Site hardware specific commands which are not critical for operation, but
 			*  controlprograms may optionally access to if they are site aware.  
 			*/  
			if (verbose > 1 ) printf("Driver: AUX_COMMAND\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/

                        msg.status=1;
                        if(msg.status==1) {
			  /* process aux command dictionary here */
			  process_aux_msg(msg,&r_msg,verbose);
			}
			break;



/* Commands that should only be servced  by a single driver. 
 * Site ini file should be configured to instruct the ROS as to which driver
 * services each of the following commands */
		      case GET_TRTIMES:
		        sprintf(command_string,"GET_TRTIMES");
			/* GET_TRTIMES: After a trigger or external trigger command has been issued. The ROS 
			*  may issue this command to a driver as a way to retrieve information about the
			*  actual TR time windows that were used. 
			*  The timing driver is the only driver that should respond to this command
 			*/  
			if (verbose > 1 ) printf("Driver: GET_TRTIMES\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"TIMING")==0) r_msg.status=1;
	  		else r_msg.status=0;

			if(r_msg.status==1) {
			        if (verbose > 1 ) printf("Driver: tr_length %d\n",transmit_times.length);	
				ros_msg_add_var(&r_msg,&transmit_times.length,sizeof(int32),"num_tr_windows","int32");
				if(transmit_times.length > 0) {
				  ros_msg_add_var(&r_msg,transmit_times.start_usec,sizeof(uint32)*transmit_times.length,"tr_windows_start_usec","array");
				  ros_msg_add_var(&r_msg,transmit_times.duration_usec,sizeof(uint32)*transmit_times.length,"tr_windows_duration_usec","array");
				}
			}
			break;

		      case GET_DATA_STATUS:
		        sprintf(command_string,"GET_DATA_STATUS");
			/* GET_DATA_STATUS: After a trigger or external trigger command has been issued. The ROS 
			*  may issue this command to a driver as a way to determine if there was an error 
			*  collecting sample data.  Typically if there is no error GET_DATA will be sent as 
			*  the next command.  If there is an error (status < 0 ) GET_DATA may not be performed. 
			*  The receiver driver is the only driver that should respond to this command
 			*/  
			if (verbose > 1 ) printf("Driver: GET_DATA_STATUS\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
			data_status=1;
          		if(strcmp(dinfo.type,"RECV")==0) {
				r_msg.status=1;
				ros_msg_get_var_by_name(&msg,"radar",&radar);
				ros_msg_get_var_by_name(&msg,"channel",&channel);
				ros_msg_get_var_by_name(&msg,"bufnum",&bufnum);
				data_status=1;
				ros_msg_add_var(&r_msg,&data_status,sizeof(int32),"data_status","int32");
			}
	  		else r_msg.status=0;
			break;

		      case GET_DATA:
		        sprintf(command_string,"GET_DATA");
			/* GET_DATA: After a trigger or external trigger command has been issued. The ROS 
			*  may issue this command to a driver as a way to retrieve sample data. 
			*  The receiver driver is the only driver that should respond to this command
 			*/  
			if (verbose > 1 ) printf("Driver: GET_DATA\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"RECV")==0) {
				r_msg.status=1;
				ros_msg_get_var_by_name(&msg,"radar",&radar);
				ros_msg_get_var_by_name(&msg,"channel",&channel);
                                data.use_shared_memory=0;
				data.shared_memory_offset=0;
				data.bufnum=0;
				data.samples=100;
				data.status=1;
				ros_msg_add_var(&r_msg,&data,sizeof(struct DataPRM),"dataprm","struct DataPRM");
                                if(data.use_shared_memory==0) {
                                  if(main_data==NULL) {
				    main_data=malloc(sizeof(uint32)*data.samples);	
                                  }
                                  if(back_data==NULL) {
				    back_data=malloc(sizeof(uint32)*data.samples);	
                                  }
                                  collect_data(main_data,&data);   
                                  collect_data(back_data,&data);   
				  ros_msg_add_var(&r_msg,main_data,sizeof(uint32)*data.samples,"main_data","array");
				  ros_msg_add_var(&r_msg,back_data,sizeof(uint32)*data.samples,"back_data","array");
                                }
			}
	  		else r_msg.status=0;
			break;

		      case GET_TRIGGER_OFFSET:
		        sprintf(command_string,"GET_TRIGGER_OFFSET");
			/* GET_TRIGGER_OFFSET: Hardware like digital receivers and dds use internal digital filter logic
			*  as part of their operation, These digital filters have time delays which should be accounted 
			*  for in the timing of when the cards are triggered relative to the master trigger.
			*  This function allows for the ROS to request the offset values.
			*  Currently The receiver and dds drivers are the only driver that must respond to this command 
			*  with the MSI styled hardware. 
 			*/  
			if (verbose > 1 ) printf("Driver: GET_TRIGGER_OFFSET\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"RECV")==0) {
			        temp32=70;   //positive 70 microsecond offset relative to t0 
				r_msg.status=1;
				ros_msg_get_var_by_name(&msg,"radar",&radar);
				ros_msg_get_var_by_name(&msg,"channel",&channel);
				ros_msg_add_var(&r_msg,&data_status,sizeof(int32),"rx_trigger_offset_usec","int32");
			}
          		if(strcmp(dinfo.type,"DDS")==0) {
			        temp32=-50;   //negative 50 microsecond offset relative to t0 
				r_msg.status=1;
				ros_msg_get_var_by_name(&msg,"radar",&radar);
				ros_msg_get_var_by_name(&msg,"channel",&channel);
				ros_msg_add_var(&r_msg,&data_status,sizeof(int32),"dds_trigger_offset_usec","int32");
			}
	  		else r_msg.status=0;
			break;
		      case SET_TRIGGER_OFFSET:
		        sprintf(command_string,"SET_TRIGGER_OFFSET");
			/* SET_TRIGGER_OFFSET: Hardware like digital receivers and dds use internal digital filter logic
			*  as part of their operation. These digital filters have time delays which should be accounted
			*  for in the timing of when the cards are triggered relative to the master trigger.
			*  This function allows for the ROS to send trigger offset values.
			*  Currently The timing driver is the only driver that must respond to this command in the 
			*  MSI styled hardware.
 			*/  
			if (verbose > 1 ) printf("Driver: SET_TRIGGER_OFFSET\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"TIMING")==0) {
				temp32=0;
				ros_msg_get_var_by_name(&msg,"rx_trigger_offset_usec",&temp32);
				temp32=0;
				ros_msg_get_var_by_name(&msg,"dds_trigger_offset_usec",&temp32);
				temp32=0;
				ros_msg_get_var_by_name(&msg,"radar",&temp32);
				temp32=0;
				ros_msg_get_var_by_name(&msg,"channel",&temp32);
			}
	  		else r_msg.status=0;
			break;
		      case GET_EVENT_TIME:
		        sprintf(command_string,"GET_EVENT_TIME");
			/* GET_EVENT_TIME: The ROS may issue this command to a driver. 
			*  Only one driver should respond to this command 
			*  This should be turned into an AUX command 
 			*/  
			if (verbose > 1 ) printf("Driver: GET_EVENT_TIME\n");	
			/* Inform the ROS that this driver does not handle this command by sending 
 			* msg back with msg.status=0.
 			*/
          		if(strcmp(dinfo.type,"GPS")==0) {
				r_msg.status=1;
			        if (verbose > 1 ) printf("GET_EVENT_TIME: %d\n",msg.status);	
				ros_msg_add_var(&r_msg,&gps_event,sizeof(int32),"gps_event","int32");
				ros_msg_add_var(&r_msg,&gpssecond,sizeof(int32),"gps_second","int32");
				ros_msg_add_var(&r_msg,&gpsnsecond,sizeof(int32),"gps_nsecond","int32");
			}
	  		else r_msg.status=0;
			break;

			/* Required default case for unserviced commands */
		      default:
		        sprintf(command_string,"UNKNOWN");
			/* NOOPs: ROS commands that are not understood by the driver should send a msg.status=0  
			* Some drivers will not need to process all of the named commands listed above. 
			* For those driver the default case can be used and the ROS will deal with it accordingly.
 			*/  
			if (verbose > -10) printf("Driver: BAD CODE: %c : %d\n",datacode,(unsigned int)datacode);
                        r_msg.status=-1;
			break;
		    }
                    gettimeofday(&t1,NULL);
                    iniparser_set(diagnostic_INI,command_string, NULL,NULL);
                    sprintf(entry_string,"%s:%s",command_string,"elapsed_secs");
                    iniparser_set(diagnostic_INI,entry_string, elapsed_string(t0,t1),NULL);
                    sprintf(entry_string,"%s:%s",command_string,"time_stamp");
		    time_string=ctime(&t0.tv_sec);
                    strlength=strlen(time_string)-1;
                    strncpy(value,time_string,strlength);
		    value[strlength]='\0';
                    iniparser_set(diagnostic_INI,entry_string, value,NULL);
                    sprintf(entry_string,"%s:%s",command_string,"err_val");
                    sprintf(value,"%d",err_val);
                    iniparser_set(diagnostic_INI,entry_string, value,NULL);
                    sprintf(entry_string,"%s:%s",command_string,"err_string");
                    sprintf(value,"%s",err_string);
                    iniparser_set(diagnostic_INI,entry_string, value,NULL);
                    rval=ros_msg_send(msgsock, &r_msg);
		  }	
                  ros_msg_free_buffer(&r_msg);
                  ros_msg_free_buffer(&msg);
		} //while rval loop 
		if (verbose > 0 ) fprintf(stderr,"Closing socket\n");
		close(msgsock);
	};

        return 1;
}
