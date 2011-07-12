#include "iniparser.h"
#include "driver_structures.h"
#include "site_defaults.h"


/* This function should be used to configure the appropriate driver_info from the ini file and system defaults*/
/* including opening up any hardware devices for reading */
/* a return status of 1 means the hardware has been configured correctly */
/* return status of 0 means the hardware was not detected  and to continue operations in a fake driver mode*/
/* return status < 0 means hardware was detected by there is a detectable hardware fault */
/* This function should be called just after the ini file has been read in by the main driver process */
/* the t_driver data type should contain everything of interest for a specific driver */

/* This function is anticipated to be site specific. The members of the t_info union will also be site specific */

int _configure_driver(dictionary *ini, t_driver *dinfo) {
          if(strcmp(dinfo->type,"TIMING")==0) {
                dinfo->tcp.port_number=iniparser_getint(ini,"timing:tcp_port",TIMING_HOST_PORT);
                dinfo->info.timing.pci_index=iniparser_getint(ini,"timing:pci_index",-1);
                dinfo->info.timing.state_time_usec=iniparser_getint(ini,"timing:state_time_usec",5);
                dinfo->max_seq_length=iniparser_getint(ini,"timing:max_seq_length",MAX_SEQ_LENGTH);
                dinfo->max_pulses=iniparser_getint(ini,"timing:max_pulses",MAX_PULSES);
                dinfo->max_seqs=iniparser_getint(ini,"timing:max_seq",MAX_SEQS);
          }
          if(strcmp(dinfo->type,"DDS")==0) {
                dinfo->tcp.port_number=iniparser_getint(ini,"dds:tcp_port",DDS_HOST_PORT);
                dinfo->info.dds.pci_index=iniparser_getint(ini,"dds:pci_index",-1);
          }
          if(strcmp(dinfo->type,"DIO")==0) {
                dinfo->tcp.port_number=iniparser_getint(ini,"dio:tcp_port",DIO_HOST_PORT);
                dinfo->info.dio.pci_index=iniparser_getint(ini,"dio:pci_index",-1);
          }
          if(strcmp(dinfo->type,"GPS")==0) {
                dinfo->tcp.port_number=iniparser_getint(ini,"gps:tcp_port",GPS_HOST_PORT);
                dinfo->info.gps.pci_index=iniparser_getint(ini,"gps:pci_index",-1);
          }
          if(strcmp(dinfo->type,"RECV")==0) {
                dinfo->tcp.port_number=iniparser_getint(ini,"recv:tcp_port",RECV_HOST_PORT);
                dinfo->info.recv.pci_index=iniparser_getint(ini,"recv:pci_index",-1);
          }
	  return 0;
}
