#include "control_program.h"
#include "global_server_variables.h"
extern int verbose;
extern dictionary *Site_INI;

struct ControlProgram* _find_priority_cp(int radar,struct Thread_List_Item *thread_list) {
    struct ControlProgram *cprog=NULL,*priority_cprog=NULL;
    int priority=-1;
	
    while(thread_list!=NULL){
       cprog=thread_list->data;
       if (cprog!=NULL) {
         if (cprog->state!=NULL) {
           if (cprog->state->active==1) {
             if ((cprog->parameters->radar==radar) || (radar==0)) {
                if(cprog->parameters->priority>priority){
		  priority=cprog->parameters->priority;
		  priority_cprog=cprog;
                }
	     }
           }
         }
       }
       thread_list=thread_list->prev;
    }
    if (verbose > 0) {
      printf("Priority cprog: %p for radar: %d\n",priority_cprog,radar);
      if (priority_cprog!=NULL) {
        if (priority_cprog->state!=NULL) {
          printf("  radar: %d  channel %d\n",priority_cprog->parameters->radar,priority_cprog->parameters->channel);
        }
      }
    }
    return priority_cprog;
}


void _enforce_priority(int radar,struct ControlProgram *priority_cprog,struct Thread_List_Item *thread_list) {
    struct ControlProgram *cprog=NULL;
    if(priority_cprog!=NULL) {
      if(priority_cprog->parameters!=NULL) {
        while(thread_list!=NULL){
          cprog=thread_list->data;
          if (cprog!=NULL && (cprog!=priority_cprog)) {
            if (cprog->parameters!=NULL) {
              if (cprog->state!=NULL) {
                if (cprog->state->active==1) {
                  if (cprog->parameters!=NULL) {
                    if ((cprog->parameters->radar==radar) || (radar==0)) {
                      printf("Enforcing Control Program Priority Policy\n");
		      if(iniparser_getboolean(Site_INI,"priority:enforce_tfreq",0)){
		        cprog->parameters->tfreq=priority_cprog->parameters->tfreq;	
                      }
		      if(iniparser_getboolean(Site_INI,"priority:enforce_rfreq",0)){
		        cprog->parameters->rfreq=priority_cprog->parameters->rfreq;	
                      }
		      if(iniparser_getboolean(Site_INI,"priority:enforce_tbeam",0)){
		        cprog->parameters->tbeam=priority_cprog->parameters->tbeam;	
		        cprog->parameters->tbeamcode=priority_cprog->parameters->tbeamcode;	
		        cprog->parameters->tbeamazm=priority_cprog->parameters->tbeamazm;	
		        cprog->parameters->tbeamwidth=priority_cprog->parameters->tbeamwidth;	
                      }
		      if(iniparser_getboolean(Site_INI,"priority:enforce_rbeam",0)){
		        cprog->parameters->rbeam=priority_cprog->parameters->rbeam;	
		        cprog->parameters->rbeamcode=priority_cprog->parameters->rbeamcode;	
		        cprog->parameters->rbeamazm=priority_cprog->parameters->rbeamazm;	
		        cprog->parameters->rbeamwidth=priority_cprog->parameters->rbeamwidth;	
                      }
		      if(iniparser_getboolean(Site_INI,"priority:enforce_seq_index",0)){
		        cprog->parameters->current_pulseseq_index=priority_cprog->parameters->current_pulseseq_index;	
                      }
		      if(iniparser_getboolean(Site_INI,"priority:enforce_trise",0)){
		        cprog->parameters->trise=priority_cprog->parameters->trise;	
                      }
		      if(iniparser_getboolean(Site_INI,"priority:enforce_samplerate",0)){
		        cprog->parameters->baseband_samplerate=priority_cprog->parameters->baseband_samplerate;	
                      }
		      if(iniparser_getboolean(Site_INI,"priority:enforce_filter_bandwidth",0)){
		        cprog->parameters->filter_bandwidth=priority_cprog->parameters->filter_bandwidth;	
                      }
		      if(iniparser_getboolean(Site_INI,"priority:enforce_match_filter",0)){
		        cprog->parameters->match_filter=priority_cprog->parameters->match_filter;	
                      }
                    }
                  }
	        }
              }
            }
          }
          thread_list=thread_list->prev;
        }	
      }	
    }
    return ;
}
