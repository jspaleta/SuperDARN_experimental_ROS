#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "global_server_variables.h"

//#define MSG_NOSIGNAL 0 



int opentcpsock(char *hostip, int port){
	//DECLARE VARIABLES FOR IP CONNECTIONS
	int		sock,temp;
	struct	sockaddr_in	server;
	struct	hostent		*hp, *gethostbyname();
	int	option;
        socklen_t optionlen;
	//int		do_scan_rx[4];
	//struct 	protent*	protocol_info;

	//hostip=HOST;
	//port=DEFAULT_PORT;

	//SET UP IP CONNECTION
	sock=socket(AF_INET, SOCK_STREAM, 0);
	if( (sock < 0) ) {
		perror("opening stream socket");
		exit(1);
	}
	server.sin_family=AF_INET;
	hp=gethostbyname(hostip);
	if( hp == 0 ){
		fprintf(stderr, "unknown host");
		exit(2);
	}
	memcpy(&server.sin_addr, hp->h_addr, hp->h_length);
	server.sin_port=htons(port);
	temp=connect(sock, (struct sockaddr *)&server, sizeof(server));
	if( temp < 0){
		perror("connecting stream socket");
		return -1;
	}


	//protocol_info=getprotobyname("tcp");
	//printf("protocol name = %s\n",protocol_info->p_name);
        option = 1;
        temp = setsockopt( sock, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option) );

	option=TCP_NODELAY;
	optionlen=4;
	temp=setsockopt(sock,6,TCP_NODELAY,&option,optionlen);
	temp=getsockopt(sock,6,TCP_NODELAY,&option,&optionlen);
//	printf("temp=%d  optionlen=%d option=%d\n",temp,optionlen,option);
	optionlen=4;
	option=32768;
	temp=setsockopt(sock,SOL_SOCKET,SO_SNDBUF,&option,optionlen);
	temp=getsockopt(sock,SOL_SOCKET,SO_SNDBUF,&option,&optionlen);
//	printf("temp=%d  optionlen=%d option=%d\n",temp,optionlen,option);
	optionlen=4;
	option=32768;
	temp=setsockopt(sock,SOL_SOCKET,SO_RCVBUF,&option,optionlen);
	temp=getsockopt(sock,SOL_SOCKET,SO_RCVBUF,&option,&optionlen);
//	printf("temp=%d  optionlen=%d option=%d\n",temp,optionlen,option);

    return sock;
}

int send_data(int fd,void  *buf,size_t buflen)
{
     int cc=0,total=0;
     while (buflen > 0) {
          cc = send(fd, buf, buflen, MSG_NOSIGNAL);
          if (cc == -1) {
               return cc;
          }
          if (cc == 0) {
            return -1;
          }

          buf += cc;
          total += cc;
          buflen -= cc;
     }
     return total;
}

int recv_data(int fd,void *buf,size_t buflen)
{
     int cc=0,total=0;
     while (buflen > 0) {
          cc = recv(fd, buf, buflen, MSG_NOSIGNAL);
          if (cc == -1) {
               return cc;
          }
          if (cc == 0) {
            return -1;
          }
          buf += cc;
          total += cc;
          buflen -= cc;
     }
     return total;

}

char* ltoa(int value, char* str, int radix) {
static char dig[] =
"0123456789"
"abcdefghijklmnopqrstuvwxyz";
int n = 0, neg = 0;
unsigned int v;
char* p, *q;
char c;
 
if (radix == 10 && value < 0) {
value = -value;
neg = 1;
}
v = value;
do {
str[n++] = dig[v%radix];
v /= radix;
} while (v);
if (neg)
str[n++] = '-';
str[n] = '\0';
for (p = str, q = p + n/2; p != q; ++p, --q)
c = *p, *p = *q, *q = c;
return str;
}


int driver_msg_int(struct DriverMsg *msg,char command, char *name,int status){
  msg->command_type=command;
  strcpy(msg->command_name,name);
  msg->bytes=0;
  if((void *)msg->buffer!=NULL) free((void *)msg->buffer);
  msg->buffer=0;
  msg->status=status;
  return 0;
};

int driver_msg_clear(struct DriverMsg *msg){
  msg->command_type=0;
  strcpy(msg->command_name,"");
  msg->bytes=0;
  if((void *)msg->buffer!=NULL) free((void *)msg->buffer);
  msg->buffer=0;
  msg->status=0;
  return 0;
};

int driver_msg_write(struct DriverMsg *msg,void *arg,unsigned int bytes){
 msg->buffer=(uint64) realloc((void *)msg->buffer,msg->bytes+bytes);
 memmove((void *)msg->buffer+msg->bytes,arg,bytes); 
 msg->bytes+=bytes;
 return 0; 
};

int driver_msg_read(struct DriverMsg *msg,void *arg,unsigned int bytes){
  void *address;
  address=(void *)msg->buffer+msg->bytes-bytes;
  if(address>=(void *)msg->buffer) {
  } else {
    address=(void *)msg->buffer;
    bytes=msg->bytes;
  }
  memmove(arg,address,bytes);
  msg->bytes-=bytes;
  return 0;
};

int driver_msg_send(int socket,struct DriverMsg *msg) {
  return 0;
}

int driver_msg_recv(int socket,struct DriverMsg *msg) {
  return 0;
}


