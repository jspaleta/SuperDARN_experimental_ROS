#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include "global_server_variables.h"

int ROS_Assign_Radar(struct ClientProgram *client1,struct timeval *client_timeout,int radar,int channel) 
{
     int s,fd,id; 
     unsigned long client_address;
     struct ClientProgram *client;
     struct sockaddr_un sa;
     struct ArbMsg msg;
     msg.command=RADAR_INIT;
     char client_name[80];
     struct timeval tv;
     tv.tv_sec = 30;  /* 30 Secs Timeout */

/*
 * Change the default recv timeout to requested timeout
*/ 
/*     if ((setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,(struct timeval *)&tv,sizeof(struct timeval)))== -1) {
               perror("client - setsockopt failed");
               exit(0);
     }*/
/*
 * Change the default buffer size to improve throughput for
 * large data transfers
*/ 
/*     if ((setsockopt(s, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize)))== -1) {
               perror("client - setsockopt failed");
               exit(0);
     }*/
     fprintf(stderr,"Sending Client timeout %ld %ld\n",client_timeout->tv_sec,client_timeout->tv_usec);
     send_data(s, client_timeout, sizeof(struct timeval));
     fprintf(stderr,"Sending radar %d\n",radar);
     send_data(s, &radar, sizeof(radar));
     fprintf(stderr,"Sending channel %d\n",channel);
     send_data(s, &channel, sizeof(channel));
     fprintf(stderr,"Recieving id\n");
     recv_data(s, &id, sizeof(id));
     fprintf(stderr,"assigned id: %d\n",id);
     if (id < 0) {
     fprintf(stderr,"Bad id: %d\n",id);
       return -1;
     }
     recv_data(s, &client_address, sizeof(client_address));
     fprintf(stderr,"Recieved Client Address is %lu\n",client_address);
     sprintf(client_name,"%s%lu",CLIENT_PREFIX,id);
     fprintf(stderr,"Client Name is %s\n",client_name);
     fflush(stderr); 
     fd= shm_open(client_name, O_RDWR|O_CREAT, 0777);
     if (ftruncate(fd, sizeof(struct ClientProgram)) == -1) {
          printf("client: bad truncate \n");
          exit(0);
     }

     client=mmap(client_address,sizeof(struct ClientProgram),PROT_READ,MAP_FIXED|MAP_SHARED,fd,NULL);
     fprintf(stderr,"Client fd is %d\n",fd);
     close(fd);     
     fprintf(stderr,"Mapped Client Address is %lu\n",client);
//     printf("  Client Address %lu %lu\n",client_address,*client_address);
     printf("  Client ID %lu\n",client->id);
     printf("  Radar Number: %d\n",client->radar);
     printf("  Chan Number: %d\n",client->channel);
     printf("  Timeout secs: %ld\n",client->timeout.tv_sec);
     printf("  Timeout usecs: %ld\n",client->timeout.tv_usec);
     printf("  Access secs: %ld\n",client->last_access.tv_sec);
     printf("  Access usecs: %ld\n",client->last_access.tv_usec);

     return s;
}

int ROS_Unassign_Radar(int s,void** client) 
{
     struct ClientProgram *client_address;
     struct ArbMsg msg;
     msg.command=RADAR_CLOSE;
     char client_name[80];
/*
 * send and receive the Radar request structure.
 */
     client_address=*client;
     sprintf(client_name,"%s%lu",CLIENT_PREFIX,client_address->id);
     munmap(client_address,sizeof(struct ClientProgram));
     shm_unlink(client_name);
     send_data(s, &msg, sizeof(struct ArbMsg));
     client_address=*client;
     fprintf(stderr,"Closing Client Address %lu\n",client_address);
     send_data(s, &client_address, sizeof(client_address));
//     recv_data(s, &client_address, sizeof(client_address));
     
     return 0;
}
