#include "global_server_variables.h"
#include "site.h"
#include "iniparser.h"

extern int verbose;
int _open_ini_file(dictionary **arg,char *ini_name) {
     dictionary *Site_INI=*arg;


     if(Site_INI!=NULL) {
       iniparser_freedict(Site_INI);
       Site_INI=NULL;
     }
     if(verbose >0) fprintf(stderr, "parsing ini file: %s\n", ini_name);
     Site_INI=iniparser_load(ini_name);
     if (Site_INI==NULL) {
	fprintf(stderr, "cannot parse file: %s\n", ini_name);
	return -1;
     }
     *arg=Site_INI;
     return 0;
}

int _dump_ini_section(FILE *f,dictionary *ini,char *driver_section_name) {
	iniparser_dump_secname(ini,driver_section_name,f);
	return 0;
}
