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
#include "global_server_variables.h"
#include "include/plx_functions.h"
#include "include/plx_defines.h"
#include "utils.h"
#include "site.h"

extern int verbose;

int _select_tx(unsigned int base, int radar,int address){

	return 0;

}

int _get_status(unsigned int base,int radar,struct tx_status *txstatus )
{
  int tx_address;
  for (tx_address=0;tx_address<MAX_TRANSMITTERS;tx_address++) {

      txstatus->status[tx_address]=0xf;
      txstatus->AGC[tx_address]=1;       
      txstatus->LOWPWR[tx_address]=1;
  }
  return 0;
}



