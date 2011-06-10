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

int _decodestate(int r,int c,char state){

	int output, code;
	code=0;
// Hi (19-32) Bits mapped to radar channels
	if( (state & 0x80) == 0x80 ) code|=(0x01<<(7+(r+1)*(c+1)));  	//scope sync bit
	if( (state & 0x02) == 0x02 ) code|=(0x01<<(7+2*(r+1)*(c+1)));	  	//TR bit
	if( (state & 0x10) == 0x10 ) code|=(0x01<<(7+3*(r+1)*(c+1))); 	//phase bit
// Lo 8 Bits not mapped to radar channel
	if( (state & 0x80) == 0x80 ) code|=0x01;  			//scope sync bit
	if( (state & 0x02) == 0x02 ) {
          code|=0x02;	  		//TR bit
        }
//	if( (state & 0x10) == 0x10 ) code|=0x08; 			//phase bit
//	if( (state & 0x04) == 0x04 ) code|=0x4;  			//Tx bit
//	if( (state & 0x08) == 0x08 ) code|=0x10;  			//attenuation bit
//	if( (state & 0x01) == 0x01 ) code|=0x20;  			//sample bit
// Bits 14-15 for hardware triggers
//	if( (state & 0x80) == 0x80 ) code|=0x4000;  		// Port_B14 (Pin 52) DDS  trigger bit
//	if( (state & 0x80) == 0x80 ) code|=0x8000;  		// Port_B15 (Pin 51) RX  trigger bit
	
	return code;
}
