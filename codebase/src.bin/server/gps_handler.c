#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "rosmsg.h"
#include "control_program.h"
#include "global_server_variables.h"
#include "gps_handler.h"
#include "utils.h"
#include "iniparser.h"

extern int gpssock;
extern int verbose;
extern pthread_mutex_t gps_comm_lock;

void *GPS_aux_msg(struct ROSMsg *msg_p)
{
  struct ROSMsg s_msg,r_msg;
  driver_msg_init(&s_msg);
  driver_msg_init(&r_msg);
  pthread_mutex_lock(&gps_comm_lock);
  memmove(&s_msg,msg_p,sizeof(struct ROSMsg));
  driver_msg_send(gpssock, &s_msg);
  driver_msg_recv(gpssock, &r_msg);
  memmove(msg_p,&r_msg,sizeof(struct ROSMsg));
  pthread_mutex_unlock(&gps_comm_lock);
  pthread_exit(NULL);
};

