#include "global_server_variables.h"

#define ANTENNA_BEAM 'b'
#define ANTENNA_AUTO 'a'
#define ANTENNA_FIX 'f'


struct FreqTable *FreqLoadTable(FILE *fp);
void *dio_rxfe_settings(void *arg);
void *DIO_ready_controlprogram(struct ControlProgram *arg);
void *DIO_pretrigger(void *arg);
void *DIO_transmitter_status(int radar);
//void dio_default_config(struct dio_hdw *hdw);
//int load_config(FILE *fp,struct dio_hdw *hdw);
void *DIO_clrfreq(struct ControlProgram *arg);
void *DIO_rxfe_reset(void *arg);

