//top- clock-out
//     clock-in
//     trigger
//     chan4
//     chan3
//     chan2
//bot- chan1
 

#include <errno.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <netdb.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "utils.h"
#include "beam_phase.h"
#include "dds_defs.h"
#include "ics660b.h"


int sock,msgsock;
int verbose=0;
uint32 ifmode=IF_DISABLED;
struct  RXFESettings rf_settings;
struct  RXFESettings if_settings;
double d=ANTENNA_SEPARATION;
unsigned int     transmitters[16][2] = {
                 /* pci index:0-?, chip:1-4 */
                        {0,1},
                        {0,2},
                        {0,3},
                        {0,4},
                        {1,1},
                        {1,2},
                        {1,3},
                        {1,4},
                        {2,1},
                        {2,2},
                        {2,3},
                        {2,4},
                        {3,1},
                        {3,2},
                        {3,3},
                        {3,4},
                };
float  test_freqs[16]= {
			1E6, 
			2E6, 
			3E6, 
			4E6, 
                        5E6, 
                        6E6, 
                        7E6,
                        8E6, 
                        9E6, 
                        10E6,
                        11E6, 
                        12E6, 
                        13E6,
                        14E6, 
                        15E6, 
                        16E6,
                };
float dp=0.41887902;
float  test_phase[16];
 
void graceful_cleanup(int signum)
{
  close(msgsock);
  close(sock);
  exit(0);
}


int main(){
    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES

	// socket and message passing variables
	char	datacode;
	int	rval;
        fd_set rfds,efds;
        int status;

	// counter and temporary variables
	int	i,ant,j,k,r,c,cc,count;
        int     configured,buf,ratio,index,numclients=0;
	int	tempcode;
        double microseconds;
        // ics660 variables
        int pci_ind,pci_min;
        int pci_master=DDS_MASTER_INDEX;
        FILE *ics660[4];
        char *device;
        int enable = ENABLE;

        struct  DriverMsg msg;
	// timing related variables
        struct  timeval tv;
	struct	timespec	start, stop, sleep, now;
	int	clockresolution;

        int  maxclients=MAX_RADARS*MAX_CHANNELS;
        int  max_seq_count;
        int  seq_count[MAX_RADARS][MAX_CHANNELS];
        int  old_pulse_index[MAX_RADARS][MAX_CHANNELS];
        int  ready_index[MAX_RADARS][MAX_CHANNELS];
        int  *seq_buf[MAX_RADARS][MAX_CHANNELS];
        int  active[MAX_RADARS][DDS_MAX_CHANNELS];
        struct  ControlPRM  clients[maxclients],client;
        struct  TSGbuf *pulseseqs[MAX_RADARS][MAX_CHANNELS][MAX_SEQS];
        struct  TSGprm *tsgprm;
        struct timeval t0,t1,t2,t3;
        unsigned long elapsed;

//BEAM vars
        int channel,chip,b_ind;
        int T_rise=10;
        double delta,phase,b0,freq_in;
        double wvnum,bmang; 
        float C=3.e8;
        double pi;
        double state_time = STATE_TIME;
//TIMESEQ vars
        int t_seq[200];
        int state1,state2;
        int inc;
        int seqlen=0; 
        int old_seq_id=-10;
        int new_seq_id=-1;


//	unsigned int	*master_buf;
        
//	int		 pseq[7]={0, 3, 4, 6}, scope_sync[16384], TR[16384], TX[16384], TX_array[16384], 
//                         trigger[16384], FIFOlevel[16384];
//	int		 tau=2400, tperiod=1, tlength=300, time_array[10], intt=200, loopcount=0, fifocnt=0;

//        signal(SIGINT, graceful_cleanup);
        ifmode=IF_DISABLED;
        for(ant=0;ant<16;ant++) test_phase[ant]=ant*dp;
        printf("Clock Freq: %lf\n",CLOCK_FREQ);
        pi=3.1415926;
        pci_min=0;
        max_seq_count=0;
	if (verbose > 1) printf("Zeroing arrays\n");
	for (r=0;r<MAX_RADARS;r++){
	  for (c=0;c<MAX_CHANNELS;c++){
	    if (verbose > 1) printf("%d %d\n",r,c);
	    for (i=0;i<MAX_SEQS;i++) pulseseqs[r][c][i]=NULL;
            old_pulse_index[r][c]=-1; 
            ready_index[r][c]=-1; 
            seq_buf[r][c]=malloc(4*MAX_TIME_SEQ_LEN);
          } 
        }
  if(IMAGING) {
    fprintf(stderr,"DDS IMAGING Mode\n");
    if (MAX_RADARS !=1 ) {
            fprintf(stderr, "imaging configuration only supports one radar\n");
            configured=0;
    }
    if (DDS_MAX_CARDS*4 < MAX_TRANSMITTERS) {
            fprintf(stderr, "Too few cards configured for imaging radar configuration\n");
            configured=0;
    }
  } else {
    fprintf(stderr,"DDS NON-IMAGING Mode\n");
    if (DDS_MAX_CARDS*2 < MAX_RADARS) {
            fprintf(stderr, "Too few cards configured for imaging radar configuration\n");
            configured=0;
    }

  }


//                master_buf=malloc(4*MAX_TIME_SEQ_LEN);



    // OPEN TCP SOCKET AND START ACCEPTING CONNECTIONS 
	sock=tcpsocket(DDS_HOST_PORT);
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
		      case DDS_REGISTER_SEQ:
		        if (verbose > -1) printf("\nRegister new timing sequence for timing card\n");	
		        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        r=client.radar-1; 
                        c=client.channel-1; 
			if (verbose > -1) printf("  Radar: %d, Channel: %d Beamnum: %d Status %d\n",
			  client.radar,client.channel,client.tbeam,msg.status);	
		        rval=recv_data(msgsock,&index,sizeof(index));
                        if (pulseseqs[r][c][index]!=NULL) {
		          if (verbose > -1) printf("  Pulse index %d exists %p\n",index,pulseseqs[r][c][index]);
                          if (pulseseqs[r][c][index]->rep!=NULL) free(pulseseqs[r][c][index]->rep);
                          pulseseqs[r][c][index]->rep=NULL;
                          if (pulseseqs[r][c][index]->code!=NULL) free(pulseseqs[r][c][index]->code);
                          pulseseqs[r][c][index]->code=NULL;
                          free(pulseseqs[r][c][index]);
                          pulseseqs[r][c][index]=NULL;
		          if (verbose > -1) printf("  Freed Pulse index %d %p\n",index,pulseseqs[r][c][index]);
                        }
                        pulseseqs[r][c][index]=malloc(sizeof(struct TSGbuf));
                        rval=recv_data(msgsock,pulseseqs[r][c][index], sizeof(struct TSGbuf)); // requested pulseseq
                        pulseseqs[r][c][index]->prm=malloc(sizeof(struct TSGprm));
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->prm, sizeof(struct TSGprm)); // requested pulseseq
                        pulseseqs[r][c][index]->rep=
                          malloc(sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        pulseseqs[r][c][index]->code=
                          malloc(sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->rep, 
                          sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->code, 
                          sizeof(unsigned char)*pulseseqs[r][c][index]->len);
			if (verbose > -1) {
		          printf("  New Pulse index %d : %p\n",index,pulseseqs[r][c][index]);
                          printf("    Pulseseq length: %d\n",pulseseqs[r][c][index]->len);	
                          tsgprm=pulseseqs[r][c][index]->prm;
                          printf("    Stdelay: %d\n",tsgprm->stdelay);	
                        }
                        old_pulse_index[r][c]=-1; 
                        old_seq_id=-10;
                        new_seq_id=-1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;

		      case DDS_CtrlProg_END:
                        if (verbose > 1) printf("DDS driver: Closing a control program\n");
                        old_seq_id=-10;
                        new_seq_id=-1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;

		      case DDS_RXFE_SETTINGS:
                        if (verbose > -1) printf("DDS driver: Configuring for IF Mode\n");
                        rval=recv_data(msgsock,&ifmode,sizeof(ifmode)); 
                        if (verbose > -1) printf("DDS driver: IF Mode %d \n",ifmode);
                        rval=recv_data(msgsock,&rf_settings,sizeof(struct RXFESettings)); 
                        rval=recv_data(msgsock,&if_settings,sizeof(struct RXFESettings)); 
                        if (ifmode && IMAGING ) fprintf(stderr,"WARNING: RF Mode can not be enabled with IMAGING\n");
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                       
                        break;
		      case DDS_CtrlProg_READY:
		        if (verbose > 1) printf("Asking to set up dds timing info for client that is ready\n");	
		        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        r=client.radar-1; 
                        c=client.channel-1; 
                        index=client.current_pulseseq_index; 
                        if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
                          clients[ready_index[r][c]]=client;
                        } else {
                          clients[numclients]=client;
                          ready_index[r][c]=numclients;
                          numclients=(numclients+1);
                        }
			if (verbose > 1) printf("Radar: %d, Channel: %d Beamnum: %d Numclients %d\n",
			  client.radar,client.channel,client.tbeam,numclients);	
                        index=client.current_pulseseq_index; 
                        if (index!=old_pulse_index[r][c]) {
			  if (verbose > -1) printf("Need to unpack pulseseq\n");	
			  if (verbose > -1) printf("Pulseseq length: %d\n",pulseseqs[r][c][index]->len);	
			// unpack the timing sequence
			  seq_count[r][c]=0;
                          microseconds=0.0; 
                          tempcode=0;
	                  if (verbose > 1) printf("microstep: %d state: %lf ratio: %d\n",
                                                pulseseqs[r][c][index]->step,STATE_TIME,ratio);
                          ratio=(int)((pulseseqs[r][c][index]->step*1E-6)/STATE_TIME+.49999999);
			  for(i=0;i<pulseseqs[r][c][index]->len;i++){
			    tempcode=_decodestate(r,c,(pulseseqs[r][c][index]->code)[i]);	
//JDS TODO chop off first STATE_DELAY microseconds
			    for( j=0;j<ratio*(pulseseqs[r][c][index]->rep)[i];j++){
                              microseconds+=STATE_TIME/1E-6;
                              if(microseconds > STATE_DELAY ) {
			        seq_buf[r][c][seq_count[r][c]]=tempcode;
			        seq_count[r][c]++;
                              }
			    }
			  }

                        }
	                  if (verbose > 1) printf("seq length: %d state step: %lf time: %lf\n",
                                                seq_count[r][c],STATE_TIME,STATE_TIME*seq_count[r][c]);
                        if (numclients >= maxclients) msg.status=-2;
		        if (verbose > 1) printf("client %d ready done %d %d \n",numclients,client.radar,client.channel);	
                        numclients=numclients % maxclients;
                        old_pulse_index[r][c]=index;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break; 

		      case DDS_PRETRIGGER:
	                one_shot_b(ics660[pci_master]);
                        gettimeofday(&t0,NULL);
			if(verbose > 0 ) {
                          printf("Setup DDS Card for PRE-trigger\n");	
                        }
		        if (verbose > 0) printf("Numclients %d\n",numclients);	
                        new_seq_id=-1;
                        max_seq_count=0;
	                for( i=0; i<numclients; i++) {
                          r=clients[i].radar-1; 
                          c=clients[i].channel-1; 
                          if (seq_count[r][c]>=max_seq_count) max_seq_count=seq_count[r][c];
		          if (verbose > 0) printf("Max Seq length %d\n",max_seq_count);	
                          new_seq_id+=r*1000 +
                             c*100 +
                             clients[i].current_pulseseq_index+1;
		          if (verbose > 1) printf("%d %d %d\n",i,new_seq_id,clients[i].current_pulseseq_index);	
                        }
                        if (verbose > 1) printf("Time Seq ID: New: %d Old:%d\n",new_seq_id,old_seq_id);	
	  /* send the seqence to timing_seqence function */
                        if (IMAGING==0) {
                          /* SET freq and filter for MSI */
                          if (new_seq_id!=old_seq_id) { 
                            if (verbose > -1) printf("Setting the TX timing sequence %d %d\n",old_seq_id,new_seq_id);	
                            gettimeofday(&t2,NULL);
                            msi_timing_sequence(numclients,max_seq_count,clients,&seq_count,seq_buf,active,ics660);
                            //printf("DDS:: %d %d :: msi_timing : \n",t2.tv_sec,t2.tv_usec);
                            if (verbose > 0) {
                              gettimeofday(&t3,NULL);
                              elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
                              elapsed+=(t3.tv_usec-t2.tv_usec);
                              printf("  DDS MSI Timing Elapsed Microseconds: %ld\n",elapsed);
                          }
                            /* Required one shot */
                          } else {
                            /* Use old timing sequence */                          
                          } 
                          gettimeofday(&t2,NULL);
	                  pci_ind=0;

                          for (r=1;r<=MAX_RADARS;r++){
                            for (cc=1;cc<=DDS_MAX_CHANNELS;cc++){
                                load_frequency(ics660[pci_ind], r, cc, 0.0);
                                load_phase(ics660[pci_ind],r,cc,0.0);
                            }
                          }
	                  for( i=0; i<numclients; i++) {
	                    freq_in= (double)clients[i].tfreq * 1000.; // in Hz
                            T_rise=clients[i].trise;
	                    r= clients[i].radar;
	                    c= clients[i].channel;
                            if (verbose>2) printf(" Loading %d freq: %lf chip: %d channel: %d\n",i,freq_in,r,c);
                            for(cc=1;cc<=DDS_MAX_CHANNELS;cc++) {
                              if(active[r-1][cc-1]==(c-1)) {
//                                printf("Active channel r: %d c: %d cc: %d active: %d\n",
//                                    r,c,cc,active[r-1][cc-1]);
                                gettimeofday(&t3,NULL);
                                load_frequency(ics660[pci_ind], r, cc, freq_in);
                                //printf("DDS:: %d %d :: %d %d :: Load freq: %lf\n",t3.tv_sec,t3.tv_usec,r,cc,freq_in);
                                load_phase(ics660[pci_ind],r,cc,0.0);
                                load_filter_taps(ics660[pci_ind],r,cc,T_rise,state_time);
                              }
                            }
                            if((ifmode==1) && (IMAGING==0)) {                   
                              r=clients[i].radar+2;

                              freq_in= ((double)(IF_FREQ-clients[i].tfreq)) * 1000./2.0; // in Hz
                              //freq_in = 29.5*1E6;
                              printf("IF Out Freq:  %d %lf\n",clients[i].tfreq,freq_in);
                              load_frequency(ics660[pci_ind], r, c, freq_in);
                              load_filter_taps(ics660[pci_ind],r,c,T_rise,state_time);
                              load_phase(ics660[pci_ind],r,c,0.0);
                            }
	                  }
	                  one_shot_b(ics660[pci_master]);
                          gettimeofday(&t3,NULL);
                          elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
                          elapsed+=(t3.tv_usec-t2.tv_usec);
                          if (verbose > 0) printf("  DDS Set Filter and Freq  Elapsed Microseconds: %ld\n",elapsed);
                          if (verbose > 1) printf("Done setting the beam\n");	

                        } else {
                          /* SET freq, filter and BEAM CODE for IMAGING */
                          /* SET freq and filter for IMAGING */
                          if (new_seq_id!=old_seq_id) { 
//                          if (1) { 
                            if (verbose > -1) printf("Setting the TX timing sequence %d %d\n",old_seq_id,new_seq_id);	
                            gettimeofday(&t2,NULL);
                            msi_timing_sequence(numclients,max_seq_count,clients,&seq_count,seq_buf,active,ics660);
                            //printf("DDS:: %d %d :: msi_timing : \n",t2.tv_sec,t2.tv_usec);
                            if (verbose > 0) {
                              gettimeofday(&t3,NULL);
                              elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
                              elapsed+=(t3.tv_usec-t2.tv_usec);
                              printf("  DDS IMAGING Timing Elapsed Microseconds: %ld\n",elapsed);
                            }
                            /* Required one shot */
	                    //one_shot_b(ics660[pci_master]);
                          } else {
                            /* Use old timing sequence */                          
                          } 
                          
                          gettimeofday(&t2,NULL);
	                  pci_ind=0;
                          for (ant=0;ant<MAX_TRANSMITTERS;ant++){
                            pci_ind=transmitters[ant][0];
                            chip=transmitters[ant][1];
                            for (cc=1;cc<=DDS_MAX_CHANNELS;cc++){
                                load_frequency(ics660[pci_ind], chip, cc, 0.0);
	                        //one_shot_b(ics660[pci_master]);
                                //load_phase(ics660[pci_ind],chip,cc,0.0);
	                        //one_shot_b(ics660[pci_master]);
                            }
                          }
	                  for( i=0; i<numclients; i++) {
	                    freq_in= (double)clients[i].tfreq * 1000.; // in Hz
                            T_rise=clients[i].trise;
	                    r= clients[i].radar;
	                    c= clients[i].channel;
                            if (verbose>2) printf(" Loading %d freq: %lf chip: %d channel: %d\n",i,freq_in,r,c);
                            if ((clients[i].tbeam >=0) && (clients[i].tbeam < 16)) { 
                              delta=calculate_delta(freq_in,beamdirs_rad[clients[i].tbeam],d);
                            } else {
                              delta=0.0;
                            }
                            for(ant=0;ant<MAX_TRANSMITTERS;ant++) {
                              phase=delta*((double)ant-7.5);
                              pci_ind=transmitters[ant][0];
                              chip=transmitters[ant][1];
                              for(cc=1;cc<=DDS_MAX_CHANNELS;cc++) {
                                if(active[r-1][cc-1]==(c-1)) {
                                  gettimeofday(&t3,NULL);
                                  load_frequency(ics660[pci_ind], chip, cc, freq_in);
	                          //one_shot_b(ics660[pci_master]);
                                  load_phase(ics660[pci_ind],chip,cc,phase);
	                          //one_shot_b(ics660[pci_master]);
                                  load_filter_taps(ics660[pci_ind],chip,cc,T_rise,state_time);
	                          //one_shot_b(ics660[pci_master]);
                                }
                              }
                            }
	                  }
                          one_shot_b(ics660[pci_master]);
                          gettimeofday(&t3,NULL);
                          elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
                          elapsed+=(t3.tv_usec-t2.tv_usec);
                          if (verbose > 0) printf("  DDS Set Filter and Freq  Elapsed Microseconds: %ld\n",elapsed);
                          if (verbose > 1) printf("Done setting the beam\n");	
                        } 
                        msg.status=0;
                        numclients=0;
                        for (r=0;r<MAX_RADARS;r++){
                          for (c=0;c<MAX_CHANNELS;c++){
                            ready_index[r][c]=-1;
                          }
                        }
                        max_seq_count=0;
                        if (new_seq_id < 0 ) {
                          old_seq_id=-10;
                        }  else {
                          old_seq_id=new_seq_id;
                        }
                        new_seq_id=-1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        gettimeofday(&t1,NULL);
                        elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
                        elapsed+=(t1.tv_usec-t0.tv_usec);
                        if (verbose > 1) printf("  DDS Pretrigger Elapsed Microseconds: %ld\n",elapsed);
                        if (verbose > 1)  printf("Ending Pretrigger Setup\n");
                        break; 

/*		      case DDS_TRIGGER:
			if (verbose > 0 ) printf("Setup DDS Card trigger\n");	
			if (verbose > 1) printf("Read msg struct from tcp socket!\n");	
                        numclients=0;
                        max_seq_count=0;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			if (verbose > 0 ) printf("End Timing Card trigger\n");	
                        break;
*/
		      default:
			if (verbose > 0) fprintf(stderr,"BAD CODE: %c : %d\n",datacode,datacode);
			break;
		    }
		  }	
		} 
		if (verbose > 0 ) fprintf(stderr,"Closing socket\n");
		close(msgsock);
	};

        return 1;
}
