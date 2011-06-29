#include "global_server_variables.h"
#include "site.h"
#include "iniparser.h"


int _open_ini_file(dictionary **arg) {
     dictionary *Site_INI=*arg;
     char ini_name[80]="";


     if(Site_INI!=NULL) {
       iniparser_freedict(Site_INI);
       Site_INI=NULL;
     }
     sprintf(ini_name,"%s/site.ini",SITE_DIR);
     fprintf(stderr, "parsing ini file: %s\n", ini_name);
     Site_INI=iniparser_load(ini_name);
     if (Site_INI==NULL) {
	fprintf(stderr, "cannot parse file: %s\n", ini_name);
	return -1;
     }
     *arg=Site_INI;
     return 0;
}

int _dump_ini_section(dictionary *ini,char *driver_section_name) {

}
