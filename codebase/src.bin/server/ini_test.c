/*"global_server_variables.h"*/

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include "global_server_variables.h"
#include "control_program.h"
#include "iniparser.h"

pthread_mutex_t settings_lock;
int verbose=0;
dictionary  *ini;

int main()
{
     char ini_name[80]="/root/test.ini";
     char entry_name[80]="";
     int exists_flag;
     int radars;
     int r;

     ini= iniparser_load(ini_name);
     printf("Testing INI reading\n");
//        if (ini==NULL) {
//                fprintf(stderr, "cannot parse file: %s\n", ini_name);
//                return -1 ;
//        }
     exists_flag=iniparser_find_entry(ini,"site_settings:number_of_radars");
     if(exists_flag) {
       radars=iniparser_getint(ini,"site_settings:number_of_radars",0);
       printf(" Number of radars:%d\n",radars);        
     } else {
       printf("Entry does not exist: %s\n","site_settings:number_of_radars");
     }
     for(r=1;r<=radars;r++) {
       sprintf(entry_name,"radar_%d",r);
       exists_flag=iniparser_find_entry(ini,entry_name);
       if(exists_flag) {
         printf(" radar %d section exists\n",r);        
         sprintf(entry_name,"radar_%d:stid",r);
         printf("   Station ID: %d\n",iniparser_getint(ini,entry_name,-1));
       }
     }
    //    iniparser_dump(ini, stderr);
     iniparser_freedict(ini);
     return 0;
}
