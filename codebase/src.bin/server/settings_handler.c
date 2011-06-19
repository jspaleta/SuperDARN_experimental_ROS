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
     char entry_name[80]="";
     int exists_flag;
     int temp_int;

     pthread_mutex_unlock(&settings_lock);
/*
     if(Site_INI!=NULL) {
       iniparser_freedict(Site_INI);
       Site_INI=NULL;
     }
     sprintf(ini_name,"%s/site.ini",SITE_DIR);
     fprintf(stderr, "parsing file: %s\n", ini_name);
     Site_INI=iniparser_load(ini_name);
     if (Site_INI==NULL) {
       fprintf(stderr, "cannot parse file: %s\n", ini_name);
       pthread_mutex_unlock(&settings_lock);
       pthread_exit(NULL);
     }
*/
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
     printf("Trigger Type: %d\n",trigger_type);
     pthread_mutex_unlock(&settings_lock);
     pthread_exit(NULL);

}

void *settings_rxfe_update_rf(struct RXFESettings *rxfe_rf_settings)
{

  pthread_mutex_lock(&settings_lock);

     fprintf(stdout,"RXFE RF Mode Settings from INI File ::\n");

     rxfe_rf_settings->ifmode=0;
            fprintf(stdout,"RXFE :: IF: %d\n",rxfe_rf_settings->ifmode);                              

     rxfe_rf_settings->amp1=iniparser_getboolean(Site_INI,"rxfe_rf:enable_amp1",0);
            fprintf(stdout,"RXFE :: amp1: %d\n",rxfe_rf_settings->amp1);                              
     rxfe_rf_settings->amp2=iniparser_getboolean(Site_INI,"rxfe_rf:enable_amp2",0);
            fprintf(stdout,"RXFE :: amp2: %d\n",rxfe_rf_settings->amp2);                              
     rxfe_rf_settings->amp3=iniparser_getboolean(Site_INI,"rxfe_rf:enable_amp3",0);
            fprintf(stdout,"RXFE :: amp3: %d\n",rxfe_rf_settings->amp3);                              
  
     rxfe_rf_settings->att1=iniparser_getboolean(Site_INI,"rxfe_rf:enable_att1",0);
            fprintf(stdout,"RXFE :: att1: %d\n",rxfe_rf_settings->att1);                              
     rxfe_rf_settings->att2=iniparser_getboolean(Site_INI,"rxfe_rf:enable_att2",0);
            fprintf(stdout,"RXFE :: att2: %d\n",rxfe_rf_settings->att2);                              
     rxfe_rf_settings->att3=iniparser_getboolean(Site_INI,"rxfe_rf:enable_att3",0);
            fprintf(stdout,"RXFE :: att3: %d\n",rxfe_rf_settings->att3);                              
     rxfe_rf_settings->att4=iniparser_getboolean(Site_INI,"rxfe_rf:enable_att4",0);
            fprintf(stdout,"RXFE :: att4: %d\n",rxfe_rf_settings->att4);                              


  pthread_mutex_unlock(&settings_lock);
  pthread_exit(NULL);
}

void *settings_rxfe_update_if(struct RXFESettings *rxfe_if_settings)
{

  pthread_mutex_lock(&settings_lock);

     fprintf(stdout,"RXFE IF Mode Settings from INI File ::\n");

     rxfe_if_settings->ifmode=1;
            fprintf(stdout,"RXFE :: IF: %d\n",rxfe_if_settings->ifmode);                              

     rxfe_if_settings->amp1=iniparser_getboolean(Site_INI,"rxfe_if:enable_amp1",0);
            fprintf(stdout,"RXFE :: amp1: %d\n",rxfe_if_settings->amp1);                              
     rxfe_if_settings->amp2=iniparser_getboolean(Site_INI,"rxfe_if:enable_amp2",0);
            fprintf(stdout,"RXFE :: amp2: %d\n",rxfe_if_settings->amp2);                              
     rxfe_if_settings->amp3=iniparser_getboolean(Site_INI,"rxfe_if:enable_amp3",0);
            fprintf(stdout,"RXFE :: amp3: %d\n",rxfe_if_settings->amp3);                              
  
     rxfe_if_settings->att1=iniparser_getboolean(Site_INI,"rxfe_if:enable_att1",0);
            fprintf(stdout,"RXFE :: att1: %d\n",rxfe_if_settings->att1);                              
     rxfe_if_settings->att2=iniparser_getboolean(Site_INI,"rxfe_if:enable_att2",0);
            fprintf(stdout,"RXFE :: att2: %d\n",rxfe_if_settings->att2);                              
     rxfe_if_settings->att3=iniparser_getboolean(Site_INI,"rxfe_if:enable_att3",0);
            fprintf(stdout,"RXFE :: att3: %d\n",rxfe_if_settings->att3);                              
     rxfe_if_settings->att4=iniparser_getboolean(Site_INI,"rxfe_if:enable_att4",0);
            fprintf(stdout,"RXFE :: att4: %d\n",rxfe_if_settings->att4);                              


  pthread_mutex_unlock(&settings_lock);
  pthread_exit(NULL);
}



/*
void *settings_rxfe_update_if(struct RXFESettings *rxfe_if_settings)
{
  FILE  *fp;
  char rxfe_file[300],hmm[300];
  char *s,*line,*field;
  pthread_mutex_lock(&settings_lock);
  if (rxfe_if_settings!=NULL) {
    sprintf(rxfe_file,"%s/rxfe_if.dat",SITE_DIR);
    fp=fopen(rxfe_file,"r+");
    if(fp==NULL) {
      fprintf(stdout,"RXFE Settings File: FAILED :: %s\n",rxfe_file);
    } else {
      fprintf(stdout,"RXFE Settings File: OPENED :: %s\n",rxfe_file);
      s=fgets(hmm,300,fp);
      while (s!=NULL) {
        field=strtok_r(s," ",&line);
        if (strlen(field) > 0) {
          if(field[0]=='#') {
            fprintf(stdout,"found a comment :%s\n",s);                                                           
          } else {                                                                                               
            if ((field[0]>='1') && (field[0] <='9')) rxfe_if_settings->ifmode=atoi(field);            
            fprintf(stdout,"RXFE :: IF: %d\n",rxfe_if_settings->ifmode);                              
            field=strtok_r(NULL," ",&line);                                                                     
            if ((field[0]>='1') && (field[0] <='9')) rxfe_if_settings->amp1=atoi(field);              
            fprintf(stdout,"RXFE :: amp1: %d\n",rxfe_if_settings->amp1);                              
            field=strtok_r(NULL," ",&line);                                                                     
            if ((field[0]>='1') && (field[0] <='9')) rxfe_if_settings->amp2=atoi(field);              
            fprintf(stdout,"RXFE :: amp2: %d\n",rxfe_if_settings->amp2);                              
            field=strtok_r(NULL," ",&line);                                                                     
            if ((field[0]>='1') && (field[0] <='9')) rxfe_if_settings->amp3=atoi(field);              
            fprintf(stdout,"RXFE :: amp3: %d\n",rxfe_if_settings->amp3);                              
            field=strtok_r(NULL," ",&line);                                                                     
            if ((field[0]>='1') && (field[0] <='9')) rxfe_if_settings->att1=atoi(field);              
            fprintf(stdout,"RXFE :: att1: %d\n",rxfe_if_settings->att1);                              
            field=strtok_r(NULL," ",&line);                                                                     
            if ((field[0]>='1') && (field[0] <='9')) rxfe_if_settings->att2=atoi(field);              
            fprintf(stdout,"RXFE :: att2: %d\n",rxfe_if_settings->att2);                              
            field=strtok_r(NULL," ",&line);                                                                     
            if ((field[0]>='1') && (field[0] <='9')) rxfe_if_settings->att3=atoi(field);              
            fprintf(stdout,"RXFE :: att3: %d\n",rxfe_if_settings->att3);                              
            field=strtok_r(NULL," ",&line);                                                                     
            if ((field[0]>='1') && (field[0] <='9')) rxfe_if_settings->att4=atoi(field);              
            fprintf(stdout,"RXFE :: att4: %d\n",rxfe_if_settings->att4);                              
          }                                                                                                      
        }                                                                                                       
        s=fgets(hmm,300,fp);                                                                                 
      }                                                                                                         
      fclose(fp);      
    }
  }
  pthread_mutex_unlock(&settings_lock);
  pthread_exit(NULL);
}

void *settings_ros_update(struct SiteSettings *ros_settings)
{
  FILE  *fp;
  char ros_file[300],hmm[300];
  char *s,*line,*field;
  pthread_mutex_lock(&settings_lock);
  if (ros_settings!=NULL) {
    sprintf(ros_file,"%s/ros_settings.dat",SITE_DIR);
    fp=fopen(ros_file,"r+");
    if(fp==NULL) {
      fprintf(stdout,"ROS Settings File: FAILED :: %s\n",ros_file);
    } else {
      fprintf(stdout,"ROS Settings File: OPENED :: %s\n",ros_file);
      s=fgets(hmm,300,fp);
      while (s!=NULL) {
        field=strtok_r(s," ",&line);
        if (strlen(field) > 0) {
          if(field[0]=='#') {
            fprintf(stdout,"found a comment :%s\n",s);                                                           
          } else {                                                                                               
            if ((field[0]>='1') && (field[0] <='9')) ros_settings->ifmode=atoi(field);            
            fprintf(stdout,"ROS :: IF: %d\n",ros_settings->ifmode);                              
            field=strtok_r(NULL," ",&line);                                                                     
          }                                                                                                      
        }                                                                                                       
        s=fgets(hmm,300,fp);                                                                                 
      }                                                                                                         
      fclose(fp);      
    }
  }
  pthread_mutex_unlock(&settings_lock);
  pthread_exit(NULL);
}
*/
