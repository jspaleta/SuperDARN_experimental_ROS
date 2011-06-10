#include "global_server_variables.h"
#include "site.h"
#include "iniparser.h"

extern dictionary *Site_INI;

int _open_ini_file() {
     char ini_name[80]="";
     char entry_name[80]="";
     int exists_flag;
     int temp_int;


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
     return 0;
}

