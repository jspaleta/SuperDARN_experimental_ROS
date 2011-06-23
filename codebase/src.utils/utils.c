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


int  driver_msg_init(struct DriverMsg *msg){
  msg->command_type=0;
  strncpy(msg->command_name,"",MAX_MSG_STRING);
  strncpy(msg->driver,"",MAX_MSG_STRING);
  msg->is_reply=0;
  msg->bytes=0;
  msg->num_vars=0;
  msg->status=0;
  msg->buffer=(uint64)NULL;
  msg->vars=(uint64)NULL;
  return 0;
};

int driver_msg_set_command(struct DriverMsg *msg,char type, char *name,char *driver){
  msg->command_type=type;
  strncpy(msg->command_name,name,MAX_MSG_STRING);
  strncpy(msg->driver,driver,MAX_MSG_STRING);
  return 1;
}

int driver_msg_free_buffer(struct DriverMsg *msg){
  if(msg!=NULL) {
    msg->command_type=0;
    strncpy(msg->command_name,"",MAX_MSG_STRING);
    strncpy(msg->driver,"",MAX_MSG_STRING);
    msg->bytes=0;
    if((void *)msg->buffer!=NULL) free((void *)msg->buffer);
    if((struct msgvar *)msg->vars!=NULL) free((struct msgvar *)msg->vars);
    msg->buffer=0;
    msg->vars=0;
    msg->num_vars=0;
    msg->status=0;
  }
  return 0;
};

int driver_msg_add_var(struct DriverMsg *msg,void *arg,unsigned int bytes,char *var_name,char *type){
 struct msgvar var;
 struct msgvar *vars;
 int32 temp=0; 
 var.offset=msg->bytes;
 var.bytes=bytes;
 strncpy(var.name,var_name,100);
 printf("Add: #vars: %d vars: %lu\n",msg->num_vars,msg->vars);  
 printf("Add: buffer: %lu bytes: %d\n",msg->buffer,msg->bytes);  
 msg->vars=(uint64) realloc((struct msgvar *)msg->vars,(msg->num_vars+1)*sizeof(struct msgvar));
 vars=(struct msgvar*)msg->vars;
 memmove(&vars[msg->num_vars],&var,sizeof(struct msgvar)); 
 msg->buffer=(uint64) realloc((void *)msg->buffer,msg->bytes+bytes);
 memmove((void *)msg->buffer+var.offset,arg,var.bytes); 
 msg->bytes+=bytes;
 msg->num_vars++;
 printf("Add: #vars: %d vars: %lu\n",msg->num_vars,msg->vars);  
 memmove(&temp,(void *)msg->buffer,sizeof(int32));
 printf("Add: buffer: %lu bytes: %d :: %d\n",msg->buffer,msg->bytes, temp);  

 if(var.offset+var.bytes!=msg->bytes){
   fprintf(stderr,"Error in driver_msg_add_var\n");
   return -1;
 }

 return 0; 
};

int driver_msg_dump_var_info(struct DriverMsg *msg){
  struct msgvar *var;
  struct msgvar *vars;
  int var_index,bytes;
  void *address;
  vars=(struct msgvar *)msg->vars;
  for(var_index=0;var_index<msg->num_vars;var_index++) {
    printf("Var: %d ",var_index);
    var=&vars[var_index];
    printf("name: %s ",var->name);
    printf("offset: %d ",var->offset);
    printf("bytes: %d\n",var->bytes);
  }
  return 0;
};

int driver_msg_get_var_by_index(struct DriverMsg *msg,int var_index,void *buf){
  struct msgvar *var;
  struct msgvar *vars;
  int bytes;
  void *address;
  vars=(struct msgvar *)msg->vars;
  if(var_index >= msg->num_vars) {
   fprintf(stderr,"Error in driver_msg_get_var_by_index\n");
   return -1;
  }
  var=&vars[var_index];

  bytes=var->bytes;
  address=(void *)msg->buffer+var->offset;

  if((address<(void *)msg->buffer) || (address+bytes > (void *)msg->buffer+msg->bytes)) {
    fprintf(stderr,"Error in driver_msg_get_var_by_index\n");
    return -1;
    address=(void *)msg->buffer;
    bytes=msg->bytes;
  }
  memmove(buf,address,bytes);
  return 0;
};

int driver_msg_get_var_by_name(struct DriverMsg *msg,char *var_name,void *buf){
  struct msgvar *var;
  struct msgvar *vars;
  int var_index,bytes;
  void *address;
  vars=(struct msgvar *)msg->vars;
  for(var_index=0;var_index<msg->num_vars;var_index++) {
    if(strcmp(vars[var_index].name,var_name)==0) break;
  }
  if(var_index >= msg->num_vars) {
   fprintf(stderr,"Error var %s not in msg\n",var_name);
   return -1;
  }
  var=&vars[var_index];
  bytes=var->bytes;
  address=(void *)msg->buffer+var->offset;
  if(  ((uint64)address<(uint64)msg->buffer) || (((uint64)address+(uint64)bytes) > ((uint64)msg->buffer+(uint64)msg->bytes))) {
    fprintf(stderr,"Error in driver_msg_get_var_by_name\n");
    return -1;
  }
  memmove(buf,address,bytes);
  return 0;
};

int driver_msg_send(int socket,struct DriverMsg *msg) {
  int retval,error=0;
  retval=send_data(socket,msg,sizeof(struct DriverMsg));

  if(retval!=sizeof(struct DriverMsg)) error++; 

  retval=send_data(socket,(void *)msg->buffer,msg->bytes);
  if(retval!=msg->bytes) error++; 

  retval=send_data(socket,(void *)msg->vars,msg->num_vars*sizeof(struct msgvar));
  if(retval!=msg->num_vars*sizeof(struct msgvar)) error++; 

  if(error>0) {
    printf("driver_msg_send Error\n");
    return -1;
  }
  return 0;
}

int driver_msg_recv(int socket,struct DriverMsg *msg) {
  int error=0,retval;
  driver_msg_init(msg);
  retval=recv_data(socket,msg,sizeof(struct DriverMsg));
  msg->buffer=(uint64)NULL;
  msg->vars=(uint64)NULL;

  if(retval!=sizeof(struct DriverMsg)) {
    printf("msg_recv:  msg error : %d\n",retval);
    error++;
  }

  msg->buffer=(uint64)malloc(msg->bytes);
  retval=recv_data(socket,(void *)msg->buffer,msg->bytes);
  if(retval!=msg->bytes) {
    printf("msg_recv: buffer err: %d\n",retval);
    error++;
  }
  msg->vars=(uint64)malloc(msg->num_vars*sizeof(struct msgvar));
  retval=recv_data(socket,(void *)msg->vars,msg->num_vars*sizeof(struct msgvar));
  if(retval!=msg->num_vars*sizeof(struct msgvar)) {
    printf("msg_recv: var err: %d\n",retval);
    error++;
  }
  if(error>0) return -1;
  return 0;

}


