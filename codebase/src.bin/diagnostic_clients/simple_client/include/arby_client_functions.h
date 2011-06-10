void graceful_exit(int signum);
int ROS_Assign_Radar(struct ClientProgram *client,struct timeval *client_timeout,int radar,int channel); 
int ROS_Unassign_Radar(int s,void** client); 
 

