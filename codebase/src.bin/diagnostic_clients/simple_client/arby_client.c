/*
*       Sample Program : TCP stream sockets, Control Program process
*       Copyright License: MIT License 
*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <getopt.h>
#include "utils.h"
#include "rosmsg.h"
#include "control_program.h"
#include "tsg.h"

int s;
char *ros_ip=ROS_IP;
int ros_port=ROS_PORT;
int verbose=10;
FILE *fp;
void graceful_exit(int signum)
{
  struct ROSMsg smsg,rmsg;
        if (signum==13) errno=EPIPE;
        else {
          ros_msg_init(&smsg);
          ros_msg_init(&rmsg);
          ros_msg_set_command(&smsg,QUIT,"quit","NONE");
          ros_msg_send(s, &smsg);
          ros_msg_recv(s, &rmsg);
          ros_msg_free_buffer(&smsg);
          ros_msg_free_buffer(&rmsg);
          printf("Msg Status: %d\n",rmsg.status); 
        }
        fclose(fp);
        perror( "Stopping the control program process");
        exit(errno);
};


main( int argc, char *argv[])
{
  int nrang,frang,rsep,smsep,txpl,mpinc,mppul,nbaud,samples;
  int *pcode=NULL;
  int status,i,freq,j=0;
  int32 bufnum,radar,channel,index;
  int flag,counter;
  short I,Q;
  char command;
  int32 num_transmitters;
  struct ROSMsg *smsg=NULL,*rmsg=NULL;
  uint32 *main_data;
  uint32 *back_data;
  uint32 *agc;
  uint32 *lopwr;
  struct DataPRM dprm;
  struct ControlPRM parameters;
  struct tx_status txstatus;
  struct TRTimes bad_transmit_times;
  struct CLRFreqPRM clrfreq_parameters;
  struct SeqPRM tprm;
  struct TSGbuf *pulseseq=NULL;
  struct TSGprm prm;
  struct sockaddr_un sa;
  struct timeval t0,t1,t2,t3;
  unsigned long elapsed;
  int32 tfreq;
  float noise;
  int ptab[8] = {0,14,22,24,27,31,42,43};
  int bmnum; 

  unsigned int bufsize;
  char value[200];
  int32 bytes;
  char *dict_string=NULL;
  int32 temp=0,temp_data=44;
  char *secname_in_dict=NULL; 
  char secname_static[200]; 
  char entry[200]; 
  int arg;

  radar=1;  //Default radar 1  
  channel=1;  //Default channel 1
  /* Process arguments */
  static struct option long_options[] =
  {
               /* These options don't set a flag.
                * We distinguish them by their indices. */
               {"verbose",      no_argument,            0,      'v'},
               {"radar",       required_argument,      0,      'r'},
               {"channel",         required_argument,      0,      'c'},
               {"help",         required_argument,      0,      'h'},
               {0, 0, 0, 0}
  };
  while (1)
  {
           /* getopt_long stores the option index here. */
           int option_index = 0;

           arg = getopt_long (argc, argv, "vhr:c:",
                            long_options, &option_index);

           /* Detect the end of the options. */
           if (arg == -1)
             break;

           switch (arg)
           {
             case 'r':
               radar=atoi(optarg);
               printf ("option -r with value `%s' %d\n", optarg,radar);
              break;
             case 'c':
               channel=atoi(optarg);
               printf ("option -c with value `%s' %d\n", optarg,channel);
               break;

             case 'v':
                verbose++;
               break;

             case 'h':
             case '?':
               printf ("Usage:\n" );
               printf ("Generic ROS diagnostic controlprogram for prototyping\n" );
               printf ("-h / --help : show this message\n");
               printf ("-v / --verbose : increase verbosity by 1\n");
               printf ("-r radar / --radar radar  : Select radar number\n");
               printf ("radar values: 1 to MAX_RADARS supported by the site\n");
               printf ("-c channel / --channel channel  : Select channel number\n");
               printf ("channel values: 1 to MAX_CHANNELS supported by the site\n");
               exit(0);
               break;

             default:
               abort();
  	    }

  }


  smsg=malloc(sizeof(struct ROSMsg));
  rmsg=malloc(sizeof(struct ROSMsg));
  ros_msg_init(smsg);
  ros_msg_init(rmsg);
//Initialize structures

  for (i=0;i<MAX_TRANSMITTERS;i++) {
        txstatus.LOWPWR[i]=0,
        txstatus.AGC[i]=0,
        txstatus.status[i]=0;
  }
  fp=fopen("/tmp/gc314_test_data.txt", "w+");
  main_data=NULL;
  back_data=NULL;
  agc=NULL;
  lopwr=NULL;
  bad_transmit_times.start_usec=NULL;
  bad_transmit_times.length=0;
  bad_transmit_times.duration_usec=NULL;

  mppul=8;
  nrang=75;
  frang=180;
  rsep=45;
  smsep=300;
  txpl=300;
  mpinc=1500;
  nbaud=1;
  index=0;  
/*
*  Init Time Sequence Parameter structure
*/
  prm.nrang=nrang;         
  prm.frang=frang;         
  prm.rsep=rsep;          
  prm.smsep=smsep;
  prm.txpl=txpl; 
  prm.mpinc=mpinc;
  prm.mppul=mppul; 
  prm.mlag=0;
  prm.nbaud=nbaud;
  prm.rtoxmin=360;
  prm.stdelay=2;
  prm.gort=1;
  prm.code=pcode;
  prm.pat=ptab;
  index=0;
  
  bmnum=0;
/*
 * Init Operational Frequency Limit Variables
*/

/*
 * The SIGPIPE signal will be received if the peer has gone away
 * and an attempt is made to write data to the peer.  Ignoring
 * the signal causes the write operation to receive an EPIPE error.
 * Thus, the user is informed about what happened.
*/
  signal(SIGPIPE, graceful_exit);
  signal(SIGINT, graceful_exit);
  setbuf(stdout, 0);
  setbuf(stderr, 0);
/* Connect to ROS server process */

/*
 * Create the control program tcp socket
 */
  if ((s=opentcpsock(ros_ip, ros_port)) == -1) {
          perror("control program - ros tcp connection failed");
          exit(0);
  }

/*
 * send and receive the Radar request structure.
 */
 if(verbose>1) printf("Sending the Register Chan Command %c\n",REGISTER_RADAR_CHAN);
   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,REGISTER_RADAR_CHAN,"register_radar_chan","NONE");
   ros_msg_add_var(smsg,&radar,sizeof(int32),"radar","int32");
   ros_msg_add_var(smsg,&channel,sizeof(int32),"channel","int32");
   ros_msg_send(s, smsg); //Send the Command Message
   ros_msg_recv(s, rmsg); //resv the handshake back
   if(verbose>1) printf("Radar Chan Transfer Status: %d\n",rmsg->status); 
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);
/*
 * Create Pulse Sequence mimic SiteTimeSeq function in site library.
 * Re-create TSGMake using modified TSGBuf structure
 * Look at whether TSGAdd TSGCheck DIOVerifyID are needed
 */
 if(verbose>1) printf("Entering MakeTimeSeq\n");
 pulseseq=TSGMake(&prm,index,&flag);
 if (pulseseq!=NULL) {
   if(verbose>1) printf("Pulseseq len: %d\n",pulseseq->len);
   if(verbose>1) printf("Sending the Register Seq Command %d\n",REGISTER_SEQ);
   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,REGISTER_SEQ,"register_seq","NONE");
   tprm.len=pulseseq->len;
   tprm.step=pulseseq->step;
   tprm.samples=0;
   tprm.smdelay=0;
   index=pulseseq->index;
   ros_msg_add_var(smsg,&tprm,sizeof(struct SeqPRM),"tprm","SeqPRM");
   ros_msg_add_var(smsg,&index,sizeof(int32),"index","int32");
   ros_msg_add_var(smsg,pulseseq->rep,sizeof(unsigned char)*pulseseq->len,"rep","unsigned char array");
   ros_msg_add_var(smsg,pulseseq->code,sizeof(unsigned char)*pulseseq->len,"code","unsigned char array");
   ros_msg_send(s, smsg);
   ros_msg_recv(s, rmsg);
     if(verbose>1) printf("PulseSeq Transfer Status: %d\n",rmsg->status); 
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);
 }
/*
 * Request a default ControlPRM parameter structure to work with 
 */

 if(verbose>1) printf("Get Default Parameters %d\n",GET_PARAMETERS);
   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,GET_PARAMETERS,"get_parameters","NONE");
   ros_msg_send(s, smsg);
   ros_msg_recv(s, rmsg);
   ros_msg_get_var_by_name(rmsg,"parameters",&parameters);
      if(verbose>1) printf("Get Parameters Command Status: %d\n",rmsg->status); 
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);
 while(1) {
/* Rotate beam direction*/
   bmnum=(bmnum +1) % 16;
/*
 * Clear Frequency Search
 */
   clrfreq_parameters.freq_start_khz=10000; 
   clrfreq_parameters.freq_end_khz=11000;  
   clrfreq_parameters.nave=20;  
   clrfreq_parameters.rbeam=bmnum;  
   clrfreq_parameters.rx_bandwidth_khz=3.3; 

   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,REQUEST_CLEAR_FREQ_SEARCH,"request_clear_freq_search","NONE");
   ros_msg_add_var(smsg,&clrfreq_parameters,sizeof(struct CLRFreqPRM),"clrfreq_parameters","CLRFreqPRM");
   ros_msg_send(s, smsg);
   ros_msg_recv(s, rmsg);
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);

   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,REQUEST_ASSIGNED_FREQ,"request_assigned_freq","NONE");
   ros_msg_send(s, smsg);
   ros_msg_recv(s, rmsg);
   ros_msg_get_var_by_name(rmsg,"assigned_freq_khz",&tfreq);
   ros_msg_get_var_by_name(rmsg,"assigned_noise_pwr",&noise);
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);


   if(verbose>1) printf("Data Acquisition Loop\n");
   gettimeofday(&t0,NULL);
   if(verbose>1) printf("Sending the Keepalive Command\n");
   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,PING,"ping","NONE");
   ros_msg_send(s, smsg);
   ros_msg_recv(s, rmsg);
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);
   if(verbose>1) printf("Msg Status: %d\n",rmsg->status); 

/*
 * Set-up the operational program hardware parameters 
 */
   samples=500;
   parameters.tbeam=bmnum;   
   parameters.tfreq=12000;   
   parameters.rfreq=12000;   
   parameters.trise=10;   
   parameters.baseband_samplerate=3333; //in Hz
   parameters.filter_bandwidth=parameters.baseband_samplerate; //in Hz
   parameters.match_filter=1;
   parameters.number_of_samples=samples; // TODO: Calculate this from pulseseq

/*
 * Send the requested operational program hardware parameters 
 */

   if(verbose>1) printf("Send Set Parameters Command %d\n",SET_PARAMETERS);
   if(verbose>1) {
     printf("  radar: %d\n",parameters.radar);
     printf("  channel: %d\n",parameters.channel);
     printf("  tfreq: %d\n",parameters.tfreq);
     printf("  current transmit beam: %d\n",parameters.tbeam);
     printf("  current pulse index: %d %d %d\n",parameters.pulseseq_index[0],parameters.pulseseq_index[1],parameters.pulseseq_index[2]);
   }
   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,SET_PARAMETERS,"set_parameters","NONE");
   ros_msg_add_var(smsg,&parameters,sizeof(struct CLRFreqPRM),"parameters","ControlPRM");
   ros_msg_send(s, smsg);
   ros_msg_recv(s, rmsg);
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);


   if(verbose>1) printf("Sending the Set Ready Command %d\n",CtrlProg_READY);
   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,CtrlProg_READY,"set_ready","ALL");
   ros_msg_send(s, smsg);
   ros_msg_recv(s, rmsg);
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);

   gettimeofday(&t2,NULL);
   elapsed=(t2.tv_sec-t0.tv_sec)*1E6;
   elapsed+=(t2.tv_usec-t0.tv_usec);
   if(verbose>0) printf("  Ready Elapsed Microseconds: %ld\n",elapsed);

   if(verbose>1) printf("Sending the get data Command %d\n",GET_DATA);
   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,GET_DATA,"GET_DATA","ALL");
   bufnum=0;
   ros_msg_add_var(smsg,&bufnum,sizeof(int32),"data_buffer_number","int32");
   ros_msg_send(s, smsg);

   if(main_data!=NULL) {
        free(main_data);
        main_data=NULL;
   } 
   if(back_data!=NULL) { 
        free(back_data);
        back_data=NULL;
   }
   if(verbose>1) printf("wait for data structure\n");
   ros_msg_recv(s, rmsg);

   ros_msg_get_var_by_name(rmsg,"dprm",&dprm);
   if(verbose>1) printf("  samples: %d\n",dprm.samples);
   if(verbose>1) printf("  status: %d\n",dprm.status);
   if(dprm.status>=0) {
        main_data=malloc(sizeof(uint32)*dprm.samples);
        back_data=malloc(sizeof(uint32)*dprm.samples);
        ros_msg_get_var_by_name(rmsg,"main_data",main_data);
        ros_msg_get_var_by_name(rmsg,"back_data",back_data);
        if(bad_transmit_times.start_usec!=NULL) free(bad_transmit_times.start_usec);
        if(bad_transmit_times.duration_usec!=NULL) free(bad_transmit_times.duration_usec);
        ros_msg_get_var_by_name(rmsg,"num_tr_windows",&bad_transmit_times.length);
        if(verbose>1) printf("Number of Bad TR regions: %d\n",bad_transmit_times.length);
        bad_transmit_times.start_usec=malloc(sizeof(uint32)*bad_transmit_times.length);
        bad_transmit_times.duration_usec=malloc(sizeof(uint32)*bad_transmit_times.length);
        ros_msg_get_var_by_name(rmsg,"tr_window_start_usec",bad_transmit_times.start_usec);
        ros_msg_get_var_by_name(rmsg,"tr_window_duration_usec",bad_transmit_times.duration_usec);
        for (i=0;i<bad_transmit_times.length;i++) if(verbose>1) printf("  Start:  %d (usec) Duration:  %d (usec)\n",
                                                     bad_transmit_times.start_usec[i],bad_transmit_times.duration_usec[i]);
        if (verbose > 1) printf("Main Data Peek\n");
        fprintf(fp,":::Iteration::: %d\n",j);
        for (i=0;i<dprm.samples;i++) {
          I=(main_data[i] & 0xffff0000) >> 16;
          Q=main_data[i] & 0x0000ffff;
          if (verbose > 1) printf("        Data index: %d I: %d Q: %d \n",i,I,Q);
          fprintf(fp,"%d, %d , %d\n",i,I,Q);
        }
        j++;
   }
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);
      gettimeofday(&t3,NULL);
      elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
      elapsed+=(t3.tv_usec-t2.tv_usec);
      if(verbose>0) printf("  Wait Elapsed Microseconds: %ld\n",elapsed);

/* Verify the Program Parameters by requesting values from ROS: 
*  This is a needed step as the ROS may force parameters to different values than requested 
*   if multiple operating programs are running. (This relates to priority parameter)
*/

    ros_msg_set_command(smsg,AUX_COMMAND,"GET_TX_STATUS","DIO");
    smsg->status=1;
    printf("AUX  STATUS Radar %d\n",radar);
    ros_msg_add_var(smsg,&radar,sizeof(int32),"radar","int32");
    temp=0;
    ros_msg_get_var_by_name(smsg,"radar",&temp);
    printf("AUX  STATUS Send %d\n",temp);
    ros_msg_send(s, smsg);
    ros_msg_free_buffer(smsg);
    ros_msg_recv(s, rmsg);
    ros_msg_dump_var_info(rmsg); 
    ros_msg_get_var_by_name(rmsg,"txstatus",&txstatus); 
    ros_msg_free_buffer(rmsg);
    printf("AUX  STATUS DONE\n");
//    for (i=0;i<MAX_TRANSMITTERS;i++) {
//      printf("%d : %d %d %d\n",i,
//        txstatus.LOWPWR[i],
//        txstatus.AGC[i],
//        txstatus.status[i]);
//    }

 if(verbose>1) printf("Get Parameters %d\n",GET_PARAMETERS);
   ros_msg_init(smsg);
   ros_msg_init(rmsg);
   ros_msg_set_command(smsg,GET_PARAMETERS,"get_parameters","NONE");
   ros_msg_send(s, smsg);
   ros_msg_recv(s, rmsg);
   ros_msg_get_var_by_name(rmsg,"parameters",&parameters);
      if(verbose>1) printf("Get Parameters Command Status: %d\n",rmsg->status); 
   ros_msg_free_buffer(smsg);
   ros_msg_free_buffer(rmsg);
   if(verbose>1) {
     printf("  radar: %d\n",parameters.radar);
     printf("  channel: %d\n",parameters.channel);
     printf("  tfreq: %d\n",parameters.tfreq);
     printf("  current transmit beam: %d\n",parameters.tbeam);
     printf("  current pulse index: %d %d %d\n",parameters.pulseseq_index[0],parameters.pulseseq_index[1],parameters.pulseseq_index[2]);
   }

   gettimeofday(&t1,NULL);
   elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
   elapsed+=(t1.tv_usec-t0.tv_usec);
   if(verbose>0) printf("  Data Aquisition Elapsed Microseconds: %ld\n",elapsed);
#ifdef __QNX__
   //delay(200);
   //sleep(1);
#else
   sleep(1);
#endif
   gettimeofday(&t2,NULL);
   elapsed=(t2.tv_sec-t0.tv_sec)*1E6;
   elapsed+=(t2.tv_usec-t0.tv_usec);
   if(verbose>0) printf("              Pulse Elapsed Microseconds: %ld\n",elapsed);
 }
 if (pulseseq!=NULL) TSGFree(pulseseq);
 close(s); 
 exit(0);
}
