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
#include "utils.h"
#include "control_program.h"
#include "tsg.h"
#include "iniparser.h"
int s;
char *ros_ip=ROS_IP;
int ros_port=ROS_PORT;
int verbose=10;
FILE *fp;
void graceful_exit(int signum)
{
  struct ROSMsg msg;

        if (signum==13) errno=EPIPE;
        else {
          msg.type=QUIT;
          send_data(s, &msg, sizeof(struct ROSMsg));
          recv_data(s, &msg, sizeof(struct ROSMsg));
          printf("Msg Status: %d\n",msg.status); 
        }
        fclose(fp);
        perror( "Stopping the control program process");
        exit(errno);
};


main( int argc, char *argv[])
{
  int nrang,frang,rsep,smsep,txpl,mpinc,mppul,nbaud,samples;
  int *pcode=NULL;
  int status,index,i,r,c,freq,j=0;
  int flag,counter;
  short I,Q;
  char command;
  int32 num_transmitters;
  struct ROSMsg smsg,rmsg;
  uint32 *main;
  uint32 *back;
  uint32 *agc;
  uint32 *lopwr;
  struct DataPRM dprm;
  struct ControlPRM parameters;
  struct tx_status txstatus;
  struct TRTimes bad_transmit_times;
  struct CLRFreqPRM clrfreq_parameters;
  struct TSGbuf *pulseseq=NULL;
  struct TSGprm prm;
  struct sockaddr_un sa;
  struct timeval t0,t1,t2,t3;
  unsigned long elapsed;
  int32 tfreq;
  float noise;
  int ptab[8] = {0,14,22,24,27,31,42,43};
  int bmnum; 

  dictionary *aux_dict=NULL;
  void *temp_buf=NULL; // malloced buffer needs to be freed 
  void *dict_buf=NULL; // pointer into dictionary do not free
  unsigned int bufsize;
  char value[200];
  int32 bytes;
  char *dict_string=NULL;
  int32 temp_data=44;
  char *secname_in_dict=NULL; 
  char secname_static[200]; 
  char entry[200]; 
  int32 nsecs;

//Initialize structures

  for (i=0;i<MAX_TRANSMITTERS;i++) {
        txstatus.LOWPWR[i]=0,
        txstatus.AGC[i]=0,
        txstatus.status[i]=0;
  }
  fp=fopen("/tmp/gc314_test_data.txt", "w+");
  main=NULL;
  back=NULL;
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
 if(verbose>1) printf("Sending the Register Chan Command %c\n",SET_RADAR_CHAN);
  smsg.type=SET_RADAR_CHAN;
    send_data(s, &smsg, sizeof(struct ROSMsg)); //Send the Command Message
    r=1;  //Ask for radar 1  
    c=1;  //Ask for channel 1
    send_data(s, &r, sizeof(r)); //Send the radar request
    send_data(s, &c, sizeof(c)); //Send the channel request
    recv_data(s, &rmsg, sizeof(struct ROSMsg)); //resv the handshake back
    if(verbose>1) printf("Radar Chan Transfer Status: %d\n",rmsg.status); 

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
   smsg.type=REGISTER_SEQ;
     send_data(s, &smsg, sizeof(struct ROSMsg));
     send_data(s, pulseseq, sizeof(struct SeqPRM));
     send_data(s, pulseseq->rep, sizeof(unsigned char)*pulseseq->len);
     send_data(s, pulseseq->code, sizeof(unsigned char)*pulseseq->len);
     recv_data(s, &rmsg, sizeof(struct ROSMsg));
     if(verbose>1) printf("PulseSeq Transfer Status: %d\n",rmsg.status); 
 }
/*
 * Request a default ControlPRM parameter structure to work with 
 */

 if(verbose>1) printf("Get Default Parameters %d\n",GET_PARAMETERS);
    smsg.type=GET_PARAMETERS;
      send_data(s, &smsg, sizeof(struct ROSMsg));
      recv_data(s, &parameters, sizeof(struct ControlPRM));
      recv_data(s, &rmsg, sizeof(struct ROSMsg));
      if(verbose>1) printf("Get Parameters Command Status: %d\n",rmsg.status); 
 while(1) {
/* Rotate beam direction*/
   bmnum=(bmnum +1) % 16;
/*
 * Clear Frequency Search
 */
  clrfreq_parameters.start=10000; 
  clrfreq_parameters.end=11000;  
  clrfreq_parameters.nave=20;  
  clrfreq_parameters.rbeam=bmnum;  
  clrfreq_parameters.filter_bandwidth=250;  

  smsg.type=REQUEST_CLEAR_FREQ_SEARCH;
  send_data(s, &smsg, sizeof(struct ROSMsg));
  send_data(s, &clrfreq_parameters, sizeof(struct CLRFreqPRM));
  recv_data(s, &rmsg, sizeof(struct ROSMsg));


  smsg.type=REQUEST_ASSIGNED_FREQ;
  send_data(s, &smsg, sizeof(struct ROSMsg));
  recv_data(s,&tfreq, sizeof(int32)); 
  recv_data(s,&noise, sizeof(float));  
  recv_data(s,&rmsg, sizeof(struct ROSMsg)); 

   if(verbose>1) printf("Data Acquisition Loop\n");
   gettimeofday(&t0,NULL);
   if(verbose>1) printf("Sending the Keepalive Command\n");
    smsg.type=PING; 
      send_data(s, &smsg, sizeof(struct ROSMsg));
      recv_data(s, &rmsg, sizeof(struct ROSMsg));
      if(verbose>1) printf("Msg Status: %d\n",rmsg.status); 

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
     printf("  current pulse index: %d\n",parameters.current_pulseseq_index);
   }
    smsg.type=SET_PARAMETERS;
      send_data(s, &smsg, sizeof(struct ROSMsg));
      send_data(s, &parameters, sizeof(struct ControlPRM));
      recv_data(s, &rmsg, sizeof(struct ROSMsg));

   if(verbose>1) printf("Sending the Set Ready Command %d\n",SET_READY_FLAG);
    smsg.type=SET_READY_FLAG;
      send_data(s, &smsg, sizeof(struct ROSMsg));
      if(verbose>1) printf("wait for return message\n");
      recv_data(s, &rmsg, sizeof(struct ROSMsg));
   gettimeofday(&t2,NULL);
   elapsed=(t2.tv_sec-t0.tv_sec)*1E6;
   elapsed+=(t2.tv_usec-t0.tv_usec);
   if(verbose>0) printf("  Ready Elapsed Microseconds: %ld\n",elapsed);
   if(verbose>1) printf("Sending the get data Command %d\n",GET_DATA);
    smsg.type=GET_DATA;
      send_data(s, &smsg, sizeof(struct ROSMsg));
      if(main!=NULL) {
        free(main);
        main=NULL;
      } 
      if(back!=NULL) { 
        free(back);
        back=NULL;
      }
      if(verbose>1) printf("wait for data structure\n");
      recv_data(s, &dprm, sizeof(struct DataPRM));
      if(verbose>1) printf("  samples: %d\n",dprm.samples);
      if(verbose>1) printf("  status: %d\n",dprm.status);
      if(dprm.status>=0) {
        main=malloc(sizeof(uint32)*dprm.samples);
        back=malloc(sizeof(uint32)*dprm.samples);
        recv_data(s, main, sizeof(uint32)*dprm.samples);
        recv_data(s, back, sizeof(uint32)*dprm.samples);
        if(bad_transmit_times.start_usec!=NULL) free(bad_transmit_times.start_usec);
        if(bad_transmit_times.duration_usec!=NULL) free(bad_transmit_times.duration_usec);
        recv_data(s, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
        if(verbose>1) printf("Number of Bad TR regions: %d\n",bad_transmit_times.length);
        bad_transmit_times.start_usec=malloc(sizeof(uint32)*bad_transmit_times.length);
        bad_transmit_times.duration_usec=malloc(sizeof(uint32)*bad_transmit_times.length);
        recv_data(s, bad_transmit_times.start_usec, sizeof(uint32)*bad_transmit_times.length);
        recv_data(s, bad_transmit_times.duration_usec, sizeof(uint32)*bad_transmit_times.length);
        for (i=0;i<bad_transmit_times.length;i++) if(verbose>1) printf("  Start:  %d (usec) Duration:  %d (usec)\n",
                                                     bad_transmit_times.start_usec[i],bad_transmit_times.duration_usec[i]);
        //recv_data(s, &num_transmitters, sizeof(int32));
        //recv_data(s, txstatus.AGC, sizeof(int32)*num_transmitters);
        //recv_data(s, txstatus.LOWPWR, sizeof(int32)*num_transmitters);
        recv_data(s, &rmsg, sizeof(struct ROSMsg));
        if (verbose > 1) printf("Main Data Peek\n");
        fprintf(fp,":::Iteration::: %d\n",j);
        for (i=0;i<dprm.samples;i++) {
          I=(main[i] & 0xffff0000) >> 16;
          Q=main[i] & 0x0000ffff;
          if (verbose > 1) printf("        Data index: %d I: %d Q: %d \n",i,I,Q);
          fprintf(fp,"%d, %d , %d\n",i,I,Q);
        }
        j++;
      } else {
        recv_data(s, &rmsg, sizeof(struct ROSMsg));
      } 
      gettimeofday(&t3,NULL);
      elapsed=(t3.tv_sec-t2.tv_sec)*1E6;
      elapsed+=(t3.tv_usec-t2.tv_usec);
      if(verbose>0) printf("  Wait Elapsed Microseconds: %ld\n",elapsed);
/* Verify the Program Parameters by requesting values from ROS: 
*  This is a needed step as the ROS may force parameters to different values than requested 
*   if multiple operating programs are running. (This relates to priority parameter)
*/

    aux_dict=dictionary_new(0);
    iniparser_set(aux_dict,"COMMAND",NULL);
    iniparser_set(aux_dict,"command","GET_TX_STATUS");
    iniparser_set(aux_dict,"DIO",NULL);
    sprintf(value,"%d",r);
    iniparser_set(aux_dict,"DIO:radar",value);
    iniparser_set(aux_dict,"data",NULL);
    sprintf(value,"%d",sizeof(int32));
    iniparser_set(aux_dict,"data:bytes",value);
    dictionary_setbuf(aux_dict,"data",&temp_data,sizeof(int32));
    nsecs=iniparser_getnsec(aux_dict);
    for(i=0 ; i<nsecs;i++) {
      secname_in_dict=iniparser_getsecname(aux_dict,i);
      dict_buf=dictionary_getbuf(aux_dict,secname_in_dict,&bufsize);
      printf("%s: %d\n",secname_in_dict,bufsize);
    }

    dict_string=iniparser_to_string(aux_dict);
    bytes=strlen(dict_string)+1;
    smsg.type=AUX_COMMAND;
    smsg.status=1;
    send_data(s, &smsg, sizeof(struct ROSMsg));
    recv_data(s, &rmsg, sizeof(struct ROSMsg));
    if(rmsg.status==1) {
      printf("AUX Command is valid\n");
      send_data(s, &bytes, sizeof(int32));
      send_data(s, dict_string, bytes*sizeof(char));
      /* Prepare to send arb. data buf */
/*
      if(iniparser_find_entry(aux_dict,"data")==1) {
        dict_buf=dictionary_getbuf(aux_dict,"data",&bufsize);
        bytes=bufsize;
        send_data(s,dict_buf,bytes);
      }
*/
      nsecs=iniparser_getnsec(aux_dict);
      send_data(s, &nsecs, sizeof(int32));
      for(i=0 ; i<nsecs;i++) {
        secname_in_dict=iniparser_getsecname(aux_dict,i);
        dict_buf=dictionary_getbuf(aux_dict,secname_in_dict,&bufsize);
        bytes=strlen(secname_in_dict)+1;
        send_data(s,&bytes,sizeof(int32));
        send_data(s,secname_in_dict,bytes);
        bytes=bufsize;
        send_data(s,dict_buf,bytes);
      }

      if(aux_dict!=NULL) iniparser_freedict(aux_dict);
      aux_dict=NULL;
      printf("AUX Command send %p\n",aux_dict);
      recv_data(s, &bytes, sizeof(int32));
      if(dict_string!=NULL) free(dict_string);
      dict_string=malloc(sizeof(char)*(bytes+10));
      recv_data(s, dict_string, bytes*sizeof(char));
      if(aux_dict!=NULL) iniparser_freedict(aux_dict);
      aux_dict=NULL;
      aux_dict=iniparser_load_from_string(aux_dict,dict_string);
      free(dict_string);
      /* Prepare to recv arb. buf data buf and place it into dict*/
/*
      if(iniparser_find_entry(aux_dict,"data")==1) {
        bytes=iniparser_getint(aux_dict,"data:bytes",0);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=malloc(bytes);
        recv_data(s,temp_buf,bytes);
        dictionary_setbuf(aux_dict,"data",temp_buf,bytes);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=NULL;
      }
*/
      nsecs=0;
      recv_data(s,&nsecs,sizeof(int32));
      printf("DIO AUX Command nsecs %d\n",nsecs);
      for(i=0;i<nsecs;i++) {
        recv_data(s,&bytes,sizeof(int32));
        recv_data(s,secname_static,bytes);
        sprintf(entry,"%s:bytes",secname_static);
        bytes=iniparser_getint(aux_dict,entry,0);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=malloc(bytes);
        recv_data(s,temp_buf,bytes);
        dictionary_setbuf(aux_dict,secname_static,temp_buf,bytes);
        if(temp_buf!=NULL) free(temp_buf);
        temp_buf=NULL;
      }
      recv_data(s, &rmsg, sizeof(struct ROSMsg));
    }

    dict_buf=dictionary_getbuf(aux_dict,"dio",&bufsize);
    printf("Data Bufsize %d tx_status size: %d\n",bufsize,sizeof(struct tx_status));
    printf("TX Status for radar %d\n",r);
    memmove(&txstatus,dict_buf,bufsize);
    for (i=0;i<MAX_TRANSMITTERS;i++) {
      printf("%d : %d %d %d\n",i,
        txstatus.LOWPWR[i],
        txstatus.AGC[i],
        txstatus.status[i]);
    }

    if(aux_dict!=NULL) iniparser_freedict(aux_dict);
    aux_dict=NULL;


   if(verbose>1) printf("Send Get Parameters Command %d\n",GET_PARAMETERS);
    smsg.type=GET_PARAMETERS;
      send_data(s, &smsg, sizeof(struct ROSMsg));
      recv_data(s, &parameters, sizeof(struct ControlPRM));
      recv_data(s, &rmsg, sizeof(struct ROSMsg));
   if(verbose>1) {
     printf("  radar: %d\n",parameters.radar);
     printf("  channel: %d\n",parameters.channel);
     printf("  tfreq: %d\n",parameters.tfreq);
     printf("  current transmit beam: %d\n",parameters.tbeam);
     printf("  current pulse index: %d\n",parameters.current_pulseseq_index);
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
