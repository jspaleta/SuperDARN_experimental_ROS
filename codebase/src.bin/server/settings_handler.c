#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "control_program.h"
#include "global_server_variables.h"
#include "dio_handler.h"
#include "utils.h"
#include "iniparser.h"

extern int verbose;
extern pthread_mutex_t settings_lock;
extern dictionary *Site_INI;
extern int trigger_type;
extern int32 gpsrate;

void *settings_parse_ini_file(struct SiteSettings *ros_settings) {
     int temp_int;

     pthread_mutex_unlock(&settings_lock);
     ros_settings->ifmode=iniparser_getboolean(Site_INI,"site_settings:ifmode",IF_DISABLED);
     sprintf(ros_settings->name,"%s",iniparser_getstring(Site_INI,"site_settings:name",SITE_NAME));
     temp_int=iniparser_getint(Site_INI,"site_settings:trigger_type",0);
     switch(temp_int) {
       case 0:
       case 1:
       case 2:
         trigger_type=temp_int;
         gpsrate=iniparser_getint(Site_INI,"gps:trigger_rate",1);
         break;
       default:
         trigger_type=0;
         break;
     }
     pthread_mutex_unlock(&settings_lock);
     pthread_exit(NULL);

}

void *settings_rxfe_update_rf(struct RXFESettings *rxfe_rf_settings)
{

  pthread_mutex_lock(&settings_lock);

     if( verbose > 1 ) fprintf(stderr,"RXFE RF Mode Settings from INI File ::\n");

     rxfe_rf_settings->ifmode=0;
     if( verbose > 1 ) fprintf(stderr,"RXFE :: IF: %d\n",rxfe_rf_settings->ifmode);                              

     rxfe_rf_settings->amp1=iniparser_getboolean(Site_INI,"rxfe_rf:enable_amp1",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: amp1: %d\n",rxfe_rf_settings->amp1);                              
     rxfe_rf_settings->amp2=iniparser_getboolean(Site_INI,"rxfe_rf:enable_amp2",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: amp2: %d\n",rxfe_rf_settings->amp2);                              
     rxfe_rf_settings->amp3=iniparser_getboolean(Site_INI,"rxfe_rf:enable_amp3",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: amp3: %d\n",rxfe_rf_settings->amp3);                              
  
     rxfe_rf_settings->att1=iniparser_getboolean(Site_INI,"rxfe_rf:enable_att1",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: att1: %d\n",rxfe_rf_settings->att1);                              
     rxfe_rf_settings->att2=iniparser_getboolean(Site_INI,"rxfe_rf:enable_att2",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: att2: %d\n",rxfe_rf_settings->att2);                              
     rxfe_rf_settings->att3=iniparser_getboolean(Site_INI,"rxfe_rf:enable_att3",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: att3: %d\n",rxfe_rf_settings->att3);                              
     rxfe_rf_settings->att4=iniparser_getboolean(Site_INI,"rxfe_rf:enable_att4",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: att4: %d\n",rxfe_rf_settings->att4);                              


  pthread_mutex_unlock(&settings_lock);
  pthread_exit(NULL);
}

void *settings_rxfe_update_if(struct RXFESettings *rxfe_if_settings)
{

  pthread_mutex_lock(&settings_lock);

     if( verbose > 1 ) fprintf(stderr,"RXFE IF Mode Settings from INI File ::\n");

     rxfe_if_settings->ifmode=1;
     if( verbose > 1 ) fprintf(stderr,"RXFE :: IF: %d\n",rxfe_if_settings->ifmode);                              

     rxfe_if_settings->amp1=iniparser_getboolean(Site_INI,"rxfe_if:enable_amp1",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: amp1: %d\n",rxfe_if_settings->amp1);                              
     rxfe_if_settings->amp2=iniparser_getboolean(Site_INI,"rxfe_if:enable_amp2",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: amp2: %d\n",rxfe_if_settings->amp2);                              
     rxfe_if_settings->amp3=iniparser_getboolean(Site_INI,"rxfe_if:enable_amp3",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: amp3: %d\n",rxfe_if_settings->amp3);                              

     rxfe_if_settings->att1=iniparser_getboolean(Site_INI,"rxfe_if:enable_att1",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: att1: %d\n",rxfe_if_settings->att1);                              
     rxfe_if_settings->att2=iniparser_getboolean(Site_INI,"rxfe_if:enable_att2",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: att2: %d\n",rxfe_if_settings->att2);                              
     rxfe_if_settings->att3=iniparser_getboolean(Site_INI,"rxfe_if:enable_att3",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: att3: %d\n",rxfe_if_settings->att3);                              
     rxfe_if_settings->att4=iniparser_getboolean(Site_INI,"rxfe_if:enable_att4",0);
     if( verbose > 1 ) fprintf(stderr,"RXFE :: att4: %d\n",rxfe_if_settings->att4);                              


  pthread_mutex_unlock(&settings_lock);
  pthread_exit(NULL);
}



