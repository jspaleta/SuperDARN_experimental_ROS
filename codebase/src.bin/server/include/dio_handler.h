#include "global_server_variables.h"
#include "iniparser.h"

#define ANTENNA_BEAM 'b'
#define ANTENNA_AUTO 'a'
#define ANTENNA_FIX 'f'


struct FreqTable *FreqLoadTable(FILE *fp);
void *dio_site_settings(void *arg);
void *DIO_ready_controlprogram(struct ControlProgram *arg);
void *DIO_ready_clrsearch(struct ControlProgram *arg);
void *DIO_pre_clrsearch(struct ControlProgram *arg);
void *DIO_post_clrsearch(struct ControlProgram *arg);

void *DIO_pretrigger(void *arg);
//void dio_default_config(struct dio_hdw *hdw);
//int load_config(FILE *fp,struct dio_hdw *hdw);
void *DIO_pre_clrfreq(struct ControlProgram *arg);
void *DIO_post_clrfreq(struct ControlProgram *arg);
void *DIO_aux_command(dictionary **dict_p);
void *DIO_aux_msg(struct ROSMsg *msg);
