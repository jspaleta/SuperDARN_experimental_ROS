#include "control_program.h"
#include "global_server_variables.h"

void _enforce_priority(int radar,struct ControlProgram *priority_cprog,struct Thread_List_Item *thread_list);

struct ControlProgram* _find_priority_cp(int radar,struct Thread_List_Item *thread_list);

