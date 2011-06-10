#include <sys/types.h>
#ifdef __QNX__
  #include <hw/inout.h>
  #include <sys/socket.h>
  #include <sys/neutrino.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
#endif
#include <math.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "include/plx_functions.h"
#include "include/plx_defines.h"
#include "utils.h"

extern int verbose;

int _select_card(unsigned int base, struct ControlPRM *client){

        /* This code selects a card to address.  This can be used for
           writing data to the EEPROM, or to verify the output of the
           EEPROM. There are 20 cards in the phasing matrix, addresses
           0-19.  A card is addressed when this address corresponds to
           the switches on the phasing card.  Card address 31 is reserved for
           programming purposes.
        */
#ifdef __QNX__  
        int temp,radar,address;
        unsigned int portA,portB,portC,cntl; 
        struct  timespec nsleep;
        radar=client->radar;
        address=client->tbeam;
        //address=31;
        nsleep.tv_sec=0;
        nsleep.tv_nsec=5000;
        switch(radar) {
          case 1:
            portA=PA_GRP_0;
            portB=PB_GRP_0;
            portC=PC_GRP_0;
            break;
          case 2:
            if (DEVICE_ID==0x0c78) {
              portA=PA_GRP_2;
              portB=PB_GRP_2;
              portC=PC_GRP_2;
            } else {
              portA=PA_GRP_0;
              portB=PB_GRP_0;
              portC=PC_GRP_0;
            }
            break;
          default:
            portA=PA_GRP_0;
            portB=PB_GRP_0;
            portC=PC_GRP_0;
            break;
        }


    // check if card address is reasonable
        if ( (address>31) | (address<0) ){
                fprintf(stderr,"INVALID CARD ADDRESS - must be between 0 and 32\n");
                return -1;
        }
    // shift address left 1 bit (write enable is the lowest bit)
        address=address << 1;
    // mask out bits not used for addressing the cards
        address=address & 0x3e;
    // check for other bits in CH0, PortC that may be on
        temp=in8(base+portC);
        temp=temp & 0xc1;
    // add other bit of PortC to the address bits
        address=address+temp;
    // output the address and original other bits to PortC
        out8(base+portC,address);
        nanosleep(&nsleep,NULL);
    // verify the output
        temp=in8(base+portC);
        if (temp==address) return 0;
        else{
                fprintf(stderr,"CARD SELECT OUTPUT ERROR - requested code not sent\n");
                fprintf(stderr," code=%d\n", temp);
                return -1;
        }
#else
  return 0;
#endif  
}

