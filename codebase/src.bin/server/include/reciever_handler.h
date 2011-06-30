#include "global_server_variables.h"

void *receiver_site_settings(void *arg);
void *receiver_end_controlprogram(struct ControlProgram *arg);

void *receiver_ready_controlprogram(struct ControlProgram *arg);

void *receiver_ready_clrsearch(struct ControlProgram *arg);
void *receiver_pre_clrsearch(struct ControlProgram *arg);
void *receiver_post_clrsearch(struct ControlProgram *arg);
void *receiver_clrsearch(struct ControlProgram *arg);

void *receiver_pretrigger(void *arg);

void *receiver_posttrigger(void *arg);

void *receiver_controlprogram_get_data(struct ControlProgram *arg);

void *receiver_assign_frequency(struct ControlProgram *arg);


