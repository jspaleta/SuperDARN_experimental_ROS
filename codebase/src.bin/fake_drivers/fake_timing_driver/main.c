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
#include "timing_defs.h"
#include "_regs_PLX9080.h"
#include "iniparser.h"

#define MAX_TSG 16
#define	MAX_TIME_SEQ_LEN 1048576
#define MAX_PULSES 100

dictionary *Site_INI;
int sock,msgsock;
int verbose=10;
int configured=1;
int		writingFIFO=0, dma_count=0, under_flag=0,empty_flag=0,IRQ, intid;
int		max_seq_count, xfercount, totransfer;
uintptr_t	mmap_io_ptr_dio;
unsigned char	*BASE0_dio, *io_BASE1_dio, *BASE1_phys_dio;
unsigned int	virtual_addr[1], physical_addr[1];
struct sigevent interruptevent;
int tr_event=0, scope_event=0;
pthread_t int_thread;
void graceful_cleanup(int signum)
{
  int temp;
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
        int old_pulse_index[MAX_RADARS][MAX_CHANNELS];
        int ready_index[MAX_RADARS][MAX_CHANNELS];
	unsigned int	*master_buf;
        int old_seq_id=-10;
        int new_seq_id=-1;
        struct TRTimes bad_transmit_times, bad_transmit_temp;
        unsigned int bad_transmit_counter=0;


	// socket and message passing variables
	int	data;
	char	datacode;
	int	rval;
        fd_set rfds,efds;
	// counter and temporary variables
	int	i,j,k,r,c,buf,index,offset_pad;
	int	dds_offset,rx_offset;
        int     scope_start,dds_trigger,rx_trigger;
	int 	temp;
	int	tempint;
	char	tempchar;
	int	status,dead_flag,step;
	int	tempcode;
        struct timeval t0,t1,t2,t3,t4,t5,t6;
        unsigned long elapsed;

	// function specific message variables
        int     numclients=0;
        struct  DriverMsg msg;
	// timing related variables
        struct timeval tv;
	struct	timespec	start, stop, sleep, now;
	float	ftime;
	int	clockresolution;
	time_t	tod;
	// pci, io, and memory variables
	unsigned int	BASE0[7], BASE1[7];
	unsigned int	*mmap_io_ptr;
	int		pci_handle;
	int		pci_device=0;

// PCI-7300A variables
	int		pci_handle_dio, IRQ_dio, mmap_io_dio;
        
//	int		 pseq[7]={0, 3, 4, 6}, scope_sync[16384], TR[16384], TX[16384], TX_array[16384], 
//                         trigger[16384], FIFOlevel[16384];
//	int		 tau=2400, tperiod=1, tlength=300, time_array[10], intt=200, loopcount=0, fifocnt=0;
        int delay_count;
        unsigned long counter;

        signal(SIGINT, graceful_cleanup);
        signal(SIGTERM, graceful_cleanup);

	Site_INI=NULL;
	temp=_open_ini_file();
        if(temp < 0 ) {
                fprintf(stderr,"Error opening Site ini file, exiting driver\n");
                exit(temp);
        }
        dds_offset=iniparser_getint(Site_INI,"timing:dds_trigger_offset",DDS_OFFSET);
        rx_offset=iniparser_getint(Site_INI,"timing:rx_trigger_offset",RX_OFFSET);
        if (verbose > -1 ) fprintf(stderr,"DDS Offset: %d RX Offset: %d\n",dds_offset,rx_offset);

        max_seq_count=0;
	if (verbose > 1) printf("Zeroing arrays\n");

	for (r=0;r<MAX_RADARS;r++){
	  for (c=0;c<MAX_CHANNELS;c++){
	    if (verbose > 1) printf("%d %d\n",r,c);
	    for (i=0;i<MAX_SEQS;i++) pulseseqs[r][c][i]=NULL;
            ready_index[r][c]=-1; 
            old_pulse_index[r][c]=-1; 
            seq_buf[r][c]=malloc(4*MAX_TIME_SEQ_LEN);
           
          } 
        }
        master_buf=malloc(4*MAX_TIME_SEQ_LEN);
        bad_transmit_times.length=0;
        bad_transmit_times.start_usec=malloc(sizeof(unsigned int)*MAX_PULSES);
        bad_transmit_times.duration_usec=malloc(sizeof(unsigned int)*MAX_PULSES);
       
	clock_gettime(CLOCK_REALTIME, &stop);
	if (temp == 1)	fprintf(stderr, "DMA buffers created sucessfully!\n");
	fprintf(stderr, " EXECUTION TIME: %d nsec \n", stop.tv_nsec-start.tv_nsec);

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
                  tv.tv_sec = 5;
                  tv.tv_usec = 0;
		  if (verbose > 1) printf("%d Entering Select\n",msgsock);
                  rval = select(msgsock+1, &rfds, NULL, &efds, &tv);
		  if (verbose > 1) printf("%d Leaving Select %d\n",msgsock,rval);
                  /* Don’t rely on the value of tv now! */
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
		    switch( datacode ){
		      case TIMING_REGISTER_SEQ:
		        if (verbose > 0) printf("\nRegister new timing sequence for timing card\n");	
		        msg.status=0;
                        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        r=client.radar-1; 
                        c=client.channel-1; 
			if (verbose > 1) printf("Radar: %d, Channel: %d Beamnum: %d Status %d\n",
			  client.radar,client.channel,client.tbeam,msg.status);	
		        rval=recv_data(msgsock,&index,sizeof(index));
		        if (verbose > 1) printf("Requested index: %d\n",index);	
		        if (verbose > 1) printf("Attempting Free on pulseseq :p\n",pulseseqs[r][c][index]);	
                        if (pulseseqs[r][c][index]!=NULL) {
                          if (pulseseqs[r][c][index]->rep!=NULL)  free(pulseseqs[r][c][index]->rep);
                          if (pulseseqs[r][c][index]->code!=NULL) free(pulseseqs[r][c][index]->code);
                          free(pulseseqs[r][c][index]);
                        }
		        if (verbose > 1) printf("Done Free - Attempting Malloc\n");	
                        pulseseqs[r][c][index]=malloc(sizeof(struct TSGbuf));
		        if (verbose > 1) printf("Finished malloc\n");	
                        rval=recv_data(msgsock,pulseseqs[r][c][index], sizeof(struct TSGbuf)); // requested pulseseq
                        pulseseqs[r][c][index]->rep=
                          malloc(sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        pulseseqs[r][c][index]->code=
                          malloc(sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->rep, 
                          sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->code, 
                          sizeof(unsigned char)*pulseseqs[r][c][index]->len);
			if (verbose > 1) printf("Pulseseq length: %d\n",pulseseqs[r][c][index]->len);	
                        old_seq_id=-10;
                        old_pulse_index[r][c]=-1;
                        new_seq_id=-1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      case TIMING_CtrlProg_END:
		        if (verbose > 0) printf("\nA client is done\n");	
                        msg.status=0;
                        old_seq_id=-10;
                        new_seq_id=-1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      case TIMING_CtrlProg_READY:
		        if (verbose > 0) printf("\nAsking to set up timing info for client that is ready %d\n",numclients);	
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
                        if (index!=old_pulse_index[r][c]) {
                        //if (1==1) {
			  if (verbose > -1) fprintf(stderr,"Need to unpack pulseseq\n");	
			  if (verbose > -1) fprintf(stderr,"Pulseseq length: %d\n",pulseseqs[r][c][index]->len);	
			// unpack the timing sequence
			  seq_count[r][c]=0;
                          step=(int)((double)pulseseqs[r][c][index]->step/(double)STATE_TIME+0.5);
                            
                        //If DDS or RX Offset is negative pad the seq_buf iwith the maximum negative offset
                          offset_pad=(int)((double)MIN(dds_offset,rx_offset)/((double)STATE_TIME+0.5))-2;
			  if (verbose > -1) printf("offset pad: %d\n",offset_pad);	
                          for(i=0;i>offset_pad;i--) {
                            seq_buf[r][c][seq_count[r][c]]=0;
                            seq_count[r][c]++;
                          }
			  for(i=0;i<pulseseqs[r][c][index]->len;i++){
			    tempcode=_decodestate(r,c,(pulseseqs[r][c][index]->code)[i]);	
			    for( j=0;j<step*(pulseseqs[r][c][index]->rep)[i];j++){
			      seq_buf[r][c][seq_count[r][c]]=tempcode;
			      seq_count[r][c]++;
			    }
			  }
                        }
	                if (verbose > 1) printf("Timing Card seq length: %d state step: %lf time: %lf\n",
                                                seq_count[r][c],STATE_TIME*1E-6,STATE_TIME*1E-6*seq_count[r][c]);

                        if (numclients >= maxclients) msg.status=-2;
		        if (verbose > 1) printf("\nclient ready done\n");	
                        numclients=numclients % maxclients;
                        old_pulse_index[r][c]=index;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break; 

		      case TIMING_PRETRIGGER:
                        gettimeofday(&t0,NULL);
			if(verbose > 1 ) printf("Setup Timing Card for next trigger\n");	
                          msg.status=0;
		          if (verbose > 1) printf("Max Seq length: %d Num clients: %d\n",max_seq_count,numclients);	
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

                          if (new_seq_id!=old_seq_id) { 
                            if (verbose > -1) printf("Calculating Master sequence %d %d\n",old_seq_id,new_seq_id);
                        //if (1==1) { 
                            max_seq_count=0;
                            for (i=0;i<numclients;i++) {
                              r=clients[i].radar-1;
                              c=clients[i].channel-1;
                              if (seq_count[r][c]>=max_seq_count) max_seq_count=seq_count[r][c];
		              if (verbose > 1) printf("Max Seq length %d\n",max_seq_count);	
                              counter=0;
			      if (verbose > 1) printf("Merging Client Seq %d into Master Seq %d %d length:%d\n",
                                                    i,r,c,seq_count[r][c]);	
                              for (j=0;j<seq_count[r][c];j++) {
			        if (verbose > 1) printf("Seq index: %d\n",j);
                                if (i==0) {
			          if (verbose > 1) printf("First client: %d\n",j);
                                  master_buf[j]=seq_buf[r][c][j];
                                  counter++;
                                }
                                else  {
			  	  master_buf[j]|=seq_buf[r][c][j];
			          if (verbose > 1) printf("Non-First client: %d\n",j);
				}
                              } 
                              if (verbose > 1 ) printf("Total Tr %d\n",counter);
			    }
			    // add the FIFO level bits
                            bad_transmit_times.length=0;
                            tr_event=0; 
                            scope_event=0; 
                            scope_start=-1;
                            dds_trigger=0;
                            rx_trigger=0;
	                    if (verbose > 1) printf("Fifo stamping Master Seq using FIFOLVL %d\n",FIFOLVL);	

			    for(i=0;i<max_seq_count;i++){
                              //if( ( i!=0 ) && ( i <= (max_seq_count-FIFOLVL) ) ) {
                                if( ((i%FIFOLVL)==0) ) {
				  for(j=0;j<FIFOWIDTH;j++){
                            		master_buf[i+j]|=0x84;
				  }
                                }
                              //}
                              if ((master_buf[i] & 0x02)==0x02) {
                                if (tr_event==0) { 
                                  if (verbose > 1 ) printf("Master TR sample start: %d %x\n",i,master_buf[i]); 
                                  bad_transmit_times.length++;
                                  if(bad_transmit_times.length > 0){ 
                                    if(bad_transmit_times.length < MAX_PULSES) { 
                                      (bad_transmit_times.start_usec)[bad_transmit_times.length-1]=i;
                                      (bad_transmit_times.duration_usec)[bad_transmit_times.length-1]=1;
                                    } else {
                                      printf("Too many transmit pulses\n");
                                    } 
                                  }
                                } else {
                                    (bad_transmit_times.duration_usec)[bad_transmit_times.length-1]++;
                                }
                                tr_event=1;
                              } else {
                                if(tr_event==1) 
                                  if (verbose > 1 ) printf("Master TR sample end: %d %x\n",i,master_buf[i]); 
                                tr_event=0;
                              }
                              if ((master_buf[i] & 0x01)==0x01) {
                                if (scope_event==0) { 
                                  if (verbose > 1 ) printf("Scope Sync sample start: %d %x\n",i,master_buf[i]); 
                                  scope_start=i;
                                }
                                scope_event=1;
                              } else {
                                if (scope_event==1) 
                                  if (verbose > 1 ) printf("Scope Sync sample end: %d %x\n",i,master_buf[i]); 
                                scope_event=0;
                              }
                            }
                            if (scope_start>-1) { 
                              dds_trigger=scope_start+(int)((double)dds_offset/((double)STATE_TIME+0.5));
                              rx_trigger=scope_start+(int)((double)rx_offset/((double)STATE_TIME+0.5));
                              if (verbose > 1 ) {
                                printf("---> Scope Sync in Master %ld at %d\n",max_seq_count,scope_start); 
                                printf("---> DDS Trigger in Master %ld at %d\n",max_seq_count,dds_trigger); 
                                printf("---> Rx Trigger in Master %ld at %d\n",max_seq_count,rx_trigger); 
                              } 
                            } else {
                              if (verbose > 1 ) printf("XXX> Scope Sync not in Master %ld\n",max_seq_count); 
                              dds_trigger=0;
                              rx_trigger=0;
                            }
                            if((dds_trigger>=0) && (dds_trigger<max_seq_count)) {
                              master_buf[dds_trigger]|=0x4000;                            
                            }
                            if((rx_trigger>=0) && (rx_trigger<max_seq_count)) {
                              master_buf[rx_trigger]|=0x8000;                            
                            }
                          }
                        

	                  if (verbose > 1) printf("seq length: %d state step: %lf time: %lf\n",
                                                max_seq_count,STATE_TIME*1E-6,(STATE_TIME*1E-6*max_seq_count));

			  if (verbose > 1) printf("Max Seq Count:%d\n",max_seq_count);	
                          if (verbose > 1) printf("END FIFO Stamps\n");

                        if (new_seq_id < 0 ) {
                          old_seq_id=-10;
                        }  else {
                          old_seq_id=new_seq_id;
                        }
                        new_seq_id=-1;

                        send_data(msgsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
                        send_data(msgsock, bad_transmit_times.start_usec, 
                                  sizeof(unsigned int)*bad_transmit_times.length);
                        send_data(msgsock, bad_transmit_times.duration_usec, 
                                  sizeof(unsigned int)*bad_transmit_times.length);

			msg.status=0;
                        if (verbose > 1)  printf("Ending Pretrigger Setup\n");
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        gettimeofday(&t6,NULL);
                        elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
                        elapsed+=(t6.tv_usec-t0.tv_usec);
                        if (verbose > 1) {
                          printf("Timing Pretrigger Elapsed Microseconds: %ld\n",elapsed);
                        }
                        break; 

		      case TIMING_TRIGGER:
			if (verbose > 1 ) printf("Setup Timing Card trigger\n");	
                        msg.status=0;
			if (verbose > 1) printf("Read msg struct from tcp socket!\n");	
                        gettimeofday(&t1,NULL);
			if (verbose > 1 ) printf(" Trigger Time: %d %d\n",t1.tv_sec,t1.tv_usec);	
			if (verbose > 1 ) printf("End Timing Card trigger\n");	
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;

                      case TIMING_GPS_TRIGGER:
                        if (verbose > 1 ) printf("Setup Timing Card GPS trigger\n");
                        msg.status=0;
                        if (verbose > 1) printf("Read msg struct from tcp socket!\n");

                        if (verbose > 1 ) printf("End Timing Card GPS trigger\n");
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      case TIMING_WAIT:
			if (verbose > 1 ) printf("Timing Card: Wait\n");	
                        msg.status=0;
			if (verbose > 1) printf("Read msg struct from tcp socket!\n");	
                        dead_flag=0;
                        if (verbose > 1)  printf("Ending Wait \n");
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			break;
		      case TIMING_POSTTRIGGER:
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
