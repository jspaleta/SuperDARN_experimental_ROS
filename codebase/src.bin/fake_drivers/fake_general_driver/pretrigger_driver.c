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

int pretrigger_driver(t_driver dinfo) {
	  return 0;
}
