#include <sys/time.h>
#include "global_server_variables.h"
void *control_handler(struct ControlProgram *control_program);
struct ControlProgram *control_init();
struct ControlProgram* find_registered_controlprogram_by_radar_channel(int radar,int channel);

