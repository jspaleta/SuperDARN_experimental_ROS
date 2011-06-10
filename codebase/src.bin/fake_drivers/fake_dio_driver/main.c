#include <sys/types.h> 
#include <signal.h>
#include <math.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "priority.h"
#include "include/plx_functions.h"
#include "include/plx_defines.h"
#include "utils.h"
#include "rtypes.h"
#define ANTENNAS 20
#define IMAGING 0

int sock,msgsock;
int verbose=10;
int configured=1;
int *final_phasecodes[MAX_RADARS][ANTENNAS];
int *final_attencodes[MAX_RADARS][ANTENNAS];
double *std_angles[MAX_RADARS],*angles[MAX_RADARS], *freqs[MAX_RADARS];
int num_freqs[MAX_RADARS],num_std_angles[MAX_RADARS],num_angles[MAX_RADARS],num_antennas[MAX_RADARS],num_beam_addresses[MAX_RADARS];
int std_angle_index_offset[MAX_RADARS],angle_index_offset[MAX_RADARS];



void graceful_cleanup(int signum)
{
  close(msgsock);
  close(sock);
  exit(0);
}

int main(){

    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES
	// socket and message passing variables
        unsigned char if_addr,rf_addr;
        uint32 ifmode=IF_DISABLED;
        struct RXFESettings rf_settings;
        struct RXFESettings if_settings;
	unsigned int IOBASE=NULL;
	int	rval;
        char    datacode;
        fd_set rfds,efds;

	// function specific message variables
        struct DriverMsg msg;
        int     maxclients=MAX_RADARS*MAX_CHANNELS;
        struct  ControlPRM  clients[maxclients],client ;
        struct tx_status txstatus;

	// counter and temporary variables
	int	a,b,i,r,c,buf,tx;
        int     numclients=0;
        int     best_client;
        int  ready_index[MAX_RADARS][MAX_CHANNELS];
	// pci, io, and memory variables
	// timing related variables
        struct timeval tv;
        FILE *beamtablefile;
        char filename[80];
        //char dir[20]=BEAMTABLE_DIR;
        char radar_name[20]=RADAR_NAME;
//        int ANTENNAS;

        signal(SIGINT, graceful_cleanup);
        for (r=0;r<MAX_RADARS;r++){
          for (c=0;c<MAX_CHANNELS;c++){
            ready_index[r][c]=-1;
          }
        }
    // setup default rxfe settings.
       rf_settings.amp1=1;  
       rf_settings.amp2=0;  
       rf_settings.amp3=0;
       rf_settings.att1=0;
       rf_settings.att2=0;
       rf_settings.att3=0;
       rf_settings.att4=0;
       rf_settings.ifmode=0;
       rf_addr=build_RXFE_EEPROM_address(rf_settings);
       if_settings.amp1=1;  
       if_settings.amp2=1;  
       if_settings.amp3=1;
       if_settings.att1=0;
       if_settings.att2=0;
       if_settings.att3=0;
       if_settings.att4=0;
       if_settings.ifmode=1;
       if_addr=build_RXFE_EEPROM_address(if_settings);  
    // open beam table files
      for(r=0;r<MAX_RADARS;r++) {
        beamtablefile=NULL; 
        freqs[r]=NULL;
        std_angles[r]=NULL;
        angles[r]=NULL;
        num_freqs[r]=0;
        num_std_angles[r]=0;
        num_angles[r]=0;
        num_beam_addresses[r]=0;
        sprintf(filename,"%s/site.%s/beamcode_lookup_table_%s_%d.dat",SITE_DIR,SITE_NAME,radar_name,r+1);
        beamtablefile=fopen(filename,"r+");
        printf("%p %s\n",beamtablefile,filename);
        if(beamtablefile!=NULL) {
          fread(&num_freqs[r],sizeof(int),1,beamtablefile);
          fread(&num_std_angles[r],sizeof(int),1,beamtablefile);
          fread(&num_angles[r],sizeof(int),1,beamtablefile);
          fread(&num_beam_addresses[r],sizeof(int),1,beamtablefile);
          fread(&num_antennas[r],sizeof(int),1,beamtablefile);
          fread(&std_angle_index_offset[r],sizeof(int),1,beamtablefile);
          fread(&angle_index_offset[r],sizeof(int),1,beamtablefile);
          freqs[r]=calloc(num_freqs[r],sizeof(double));
          rval=fread(freqs[r],sizeof(double),num_freqs[r],beamtablefile);
          std_angles[r]=calloc(num_angles[r],sizeof(double));
          rval=fread(std_angles[r],sizeof(double),num_std_angles[r],beamtablefile);
          angles[r]=calloc(num_angles[r],sizeof(double));
          rval=fread(angles[r],sizeof(double),num_angles[r],beamtablefile);
          for(a=0;a<ANTENNAS;a++){
            final_phasecodes[r][a]=calloc(num_beam_addresses[r],sizeof(int));
            final_attencodes[r][a]=calloc(num_beam_addresses[r],sizeof(int));
            for(b=0;b<num_beam_addresses[r];b++){
              final_phasecodes[r][a][b]=-1;
              final_attencodes[r][a][b]=-1;
            }
          }
          for(a=0;a<ANTENNAS;a++){
            printf("a: %d\n",a);
            rval=fread(final_phasecodes[r][a],sizeof(int),BEAMCODES,beamtablefile);
          }
          for(a=0;a<ANTENNAS;a++){
            rval=fread(final_attencodes[r][a],sizeof(int),BEAMCODES,beamtablefile);
          }
          fclose(beamtablefile);
          beamtablefile=NULL;
        } else {
          fprintf(stderr,"Error reading beam lookup table file\n");
        }
        printf("Done with file\n");
     }    
    // OPEN TCP SOCKET AND START ACCEPTING CONNECTIONS 
	sock=tcpsocket(DIO_HOST_PORT);
	listen(sock, 5);
        printf("Entering Main Loop\n");
	while(1){
                rval=1;
		msgsock=accept(sock, 0, 0);
		if (verbose > 0) printf("accepting socket!!!!!\n");
		if( (msgsock==-1) ){
			perror("accept FAILED!");
			return EXIT_FAILURE;
		}
		else while (rval>=0){
                  FD_ZERO(&rfds);
                  FD_SET(msgsock, &rfds); //Add msgsock to the read watch
                  FD_ZERO(&efds);
                  FD_SET(msgsock, &efds);  //Add msgsock to the exception watch
                  /* Wait up to five seconds. */
                  tv.tv_sec = 5;
                  tv.tv_usec = 0;
		  if (verbose > 0) printf("%d Entering Select\n",msgsock);
                  rval = select(msgsock+1, &rfds, NULL, &efds, &tv);
		  if (verbose > 0) printf("%d Leaving Select %d\n",msgsock,rval);
                  /* Don’t rely on the value of tv now! */
                  if (FD_ISSET(msgsock,&efds)){
                    if (verbose > 0) printf("Exception on msgsock %d ...closing\n",msgsock);
                    break;
                  }
                  if (rval == -1) perror("select()");
                  rval=recv(msgsock, &buf, sizeof(int), MSG_PEEK); 
                  if (verbose>0) printf("%d PEEK Recv Msg %d\n",msgsock,rval);
		  if (rval==0) {
                    if (verbose > 0) printf("Remote Msgsock %d client disconnected ...closing\n",msgsock);
                    break;
                  } 
		  if (rval<0) {
                    if (verbose > 0) printf("Msgsock %d Error ...closing\n",msgsock);
                    break;
                  } 
                  if ( FD_ISSET(msgsock,&rfds) && rval>0 ) {
                    if (verbose>0) printf("Data is ready to be read\n");
		    if (verbose > 0) printf("%d Recv Msg\n",msgsock);
                    rval=recv_data(msgsock,&msg,sizeof(struct DriverMsg));
                    datacode=msg.type;
		    if (verbose > 0) printf("\nmsg code is %c\n", datacode);
				switch( datacode ){

					case DIO_CtrlProg_READY:
						if (verbose > 1) printf("\nAsking to set up dio info for client that is ready\n");	
						if (verbose > 1) printf("Read msg struct from tcp socket!\n");	
						rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
//                                                rval=recv_data(msgsock, &pulseseq, sizeof(struct TSGbuf));
                                                r=client.radar-1;
                                                c=client.channel-1;
                                                if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
                                                  clients[ready_index[r][c]]=client;
                                                } else {
                                                  clients[numclients]=client;
                                                  ready_index[r][c]=numclients;
                                                  numclients++;
                                                }
                                                numclients=numclients % maxclients;
						msg.status=1;
                                                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                                                break; 
					case DIO_GET_TX_STATUS:
						rval=recv_data(msgsock,&r,sizeof(r));
						if (verbose > 0) printf("\nDIO tx status for radar: %d\n",r);	
                                                if(configured) {  
                                                  msg.status=_get_status(IOBASE,r,&txstatus); 
                                                } else {
                                                }
						if (verbose > 0) {
                                                  for(tx=0;tx<16;tx++) {
                                                    if (verbose > 0 ) printf("  Transmitter: %d AGC: %d LOWPWR: %d\n",tx,txstatus.AGC[tx],txstatus.LOWPWR[tx]);	
                                                  }
                                                }
                                                rval=send_data(msgsock, &txstatus, sizeof(struct tx_status));
                                                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                                                break; 
					case DIO_PRETRIGGER:
						if (verbose > 1) printf("DIO Pretrigger\n");	
                                                if(configured) {  
                                                  if (IMAGING==0) {
                                                    if (TX_BEAM_PRIORITY==1) {
                                                  /*  priority beam direction*/
                                                      for (r=1;r<=MAX_RADARS;r++) {
                                                        best_client=maxclients+1; 
                                                        for (i=0;i<numclients;i++) {
                                                          if (clients[i].radar==r) { 
                                                            if (best_client >=maxclients) {
                                                              best_client=i;
                                                            } else {  
                                                              if (clients[i].priority<clients[best_client].priority) {
                                                                best_client=i;
                                                              }
                                                            }
                                                          }
                                                        }
                                                        if (best_client <maxclients) {
                                                          client=clients[best_client];
                                                          _select_card(IOBASE,&client); 
                                                          msg.status=_select_beam(IOBASE,&client); 
                                                        }
                                                      }
                                                    } else {
                                                  /*  Per radar channel phasing*/
                                                      for (i=0;i<numclients;i++) {
                                                        client=clients[best_client];
                                                        _select_card(IOBASE,&client); 
                                                        msg.status=_select_beam(IOBASE,&client); 
                                                      }
                                                    }
                                                  } else {
                                                  /* IMAGING DIO PRE-trigger steps*/
                                                  }
                                                }
                                                for (r=0;r<MAX_RADARS;r++){
                                                  for (c=0;c<MAX_CHANNELS;c++){
                                                    ready_index[r][c]=-1;
                                                  }
                                                }
                                                numclients=0;
						msg.status=1;
                                                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
						if (verbose > 1) printf("\nDIO Pretrigger\n");	
                                                break; 
					case DIO_CLRFREQ:
						if (verbose > 1) printf("DIO clrfreq setup\n");	
						rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                                                if(configured) {
                                                  _select_card(IOBASE,&client); 
                                                  msg.status=_select_beam(IOBASE,&client); 
                                                  // Set up RXFE for each radar
                                                  if(ifmode) { 
                                                    set_RXFE_EEPROM_address(IOBASE, rf_addr);
                                                  }
                                                }
						msg.status=1;
                                                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
						if (verbose > 1) printf("DIO clrfreq end\n");	
                                                break;
                                        case DIO_RXFE_RESET:
						if (verbose > 1) printf("\nDIO reset rxfe\n");	
                                                // Set up RXFE for each radar
                                                if(configured) {
                                                  if(ifmode) { 
                                                    set_RXFE_EEPROM_address(IOBASE, if_addr);
                                                  } else {
                                                    set_RXFE_EEPROM_address(IOBASE, rf_addr);
                                                  } 
                                                }
						msg.status=1;
                                                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                                                break;
                                        case DIO_RXFE_SETTINGS:
                                                if (verbose > -1) printf("DIO driver: Re-configuring RXFE Settings\n");
                                                rval=recv_data(msgsock,&ifmode,sizeof(ifmode)); 
                                                if (verbose > -1) printf("DIO driver: IF Mode %d \n",ifmode);
                                                rval=recv_data(msgsock,&rf_settings,sizeof(struct RXFESettings)); 
                                                rval=recv_data(msgsock,&if_settings,sizeof(struct RXFESettings));
                                                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                                                if(configured) {
                                                  rf_addr=build_RXFE_EEPROM_address(rf_settings);
                                                  if_addr=build_RXFE_EEPROM_address(if_settings);
                                                  if(ifmode) {
                                                    if (verbose > -1) printf("IF ENABLED\n"); 
                                                    set_RXFE_EEPROM_address(IOBASE,if_addr); 
                                                  } else {
                                                    if (verbose > -1) printf("RF ENABLED\n"); 
                                                    set_RXFE_EEPROM_address(IOBASE,rf_addr); 
                                                  }
                                                }
                                                break;
					default:
						if (verbose > 0) fprintf(stderr,"BAD CODE: %c : %d\n",datacode,datacode);
                                                msg.status=-1;
                                                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
						break;

				}

			}
		} 
		close(msgsock);
		if (verbose > 0) fprintf(stderr,"Closing socket\n");
	}


        return 1;
}
