#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#ifdef __QNX__
  #include <hw/pci.h>
  #include <hw/inout.h>
  #include <sys/neutrino.h>
  #include <sys/mman.h>
#endif
#include "plx_defines.h"
#include "control_program.h"

int build_RXFE_EEPROM_address(struct RXFESettings settings){
  return 0;
}

int set_RXFE_EEPROM_address(unsigned int  base, unsigned int  address){
  return 0;
}
int read_RXFE_EEPROM_address(int  base, int cardnum, unsigned int  address){
        return 0;

}

int write_RXFE_EEPROM_address(int  base, int cardnum, unsigned int  address, int Adata){
	return 0;

}

