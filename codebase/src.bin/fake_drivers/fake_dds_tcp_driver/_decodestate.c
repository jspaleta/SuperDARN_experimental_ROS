#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#define X_bit 0x04
#define P_bit 0x10

int _decodestate(int r,int c,char state){

	int output, code;
	code=0;
	if(( state & X_bit)==X_bit ){
	  if(( state & P_bit)==P_bit) code = -1;
	  else code = 1;
	} else code = 0;
	return code;
}
