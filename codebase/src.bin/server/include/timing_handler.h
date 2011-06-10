#include "global_server_variables.h"

//void *timing_handler(void *arg);
void *timing_pretrigger(void *arg);
void *timing_posttrigger(void *arg);
void *timing_ready_controlprogram(struct ControlProgram *arg);
void *timing_end_controlprogram(void *arg);
void *timing_wait(void *arg);
void *timing_trigger(int type);
void *timing_transmitter_status(struct tx_status *arg);
void *timing_register_seq(struct ControlProgram *arg);

