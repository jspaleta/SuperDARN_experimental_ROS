#!/bin/sh
# This is an example script to run from cron that provides a heart beat and restart
#  mechanims, if your OS does not support restart natively in its init system

LOGFILE="/tmp/ros_monitor.log"
STOP_CMD="/root/new-merged/stop.ros"
START_CMD="/root/new-merged/start.ros"


CURRENT_TIME=`date -t`
CURRENT_TIMESTR=`date -u`
TIME_0=`expr $CURRENT_TIME - 120`
RUNNING="/tmp/ros_running"
ACTIVE_SERVER_FILE="/tmp/server_activity_time"
ACTIVE_RECV_FILE="/tmp/recv_activity_time"
LAST_SERVER_CMD_FILE="/tmp/server_cmd_time"
LAST_RECV_CMD_FILE="/tmp/recv_cmd_time"
LAST_RESTART=0
SERVER_COUNT=`ps -A|grep -c arby_server`
RECV_COUNT=`ps -A|grep -c receiver_tcp_driver`

RESTART=0


if [ -f $RUNNING ]
then
  echo "$CURRENT_TIMESTR : Server is Running" 
  if [ $SERVER_COUNT -ne 1 ]
  then
    if [ $SERVER_COUNT -eq 0 ]
    then
      echo "$CURRENT_TIMESTR : No Server Process Detected" >> $LOGFILE
      echo "$CURRENT_TIMESTR : No Server Process Detected" 
    else
      echo "$CURRENT_TIMESTR : Multiple Servers Detected" >> $LOGFILE
      echo "$CURRENT_TIMESTR : Multiple Servers Detected" 
    fi
    RESTART=1
  fi

  if [ $RECV_COUNT -ne 1 ]
  then
    if [ $RECV_COUNT -eq 0 ]
    then
      echo "$CURRENT_TIMESTR : No RECV Process Detected"  >> $LOGFILE
      echo "$CURRENT_TIMESTR : No RECV Process Detected"  
    else
      echo "$CURRENT_TIMESTR : Multiple RECV Detected"    >> $LOGFILE
      echo "$CURRENT_TIMESTR : Multiple RECV Detected"    
    fi
    RESTART=1
  fi
  

  if [ -f $ACTIVE_SERVER_FILE ]
  then
    ACTIVE_SERVER_TIME=`cat $ACTIVE_SERVER_FILE`
    if [ $ACTIVE_SERVER_TIME -lt $TIME_0 ]
    then
      echo "$CURRENT_TIMESTR : Server appears to be hung"  >> $LOGFILE
      echo "$CURRENT_TIMESTR : Server appears to be hung" 
      RESTART=1
    fi
  fi

  if [ -f $ACTIVE_RECV_FILE ]
  then
    ACTIVE_RECV_TIME=`cat $ACTIVE_RECV_FILE`
    if [ $ACTIVE_RECV_TIME -lt $TIME_0 ]
    then
      echo "$CURRENT_TIMESTR : RECV appears to be hung"  >> $LOGFILE
      echo "$CURRENT_TIMESTR : RECV appears to be hung"  
      RESTART=1
    fi
  fi

  if [ -f $LAST_SERVER_CMD_FILE ]
  then
    LAST_SERVER_CMD=`cat $LAST_SERVER_CMD_FILE`
    if [ $LAST_SERVER_CMD -lt $TIME_0 ]
    then
      echo "$CURRENT_TIMESTR : Server appears to be hung no msg processed"  >> $LOGFILE
      echo "$CURRENT_TIMESTR : Server appears to be hung no msg processed"  
      RESTART=1
    fi
  fi

  if [ -f $LAST_RECV_CMD_FILE ]
  then
     LAST_RECV_CMD=`cat $LAST_RECV_CMD_FILE`
    if [ $LAST_RECV_CMD -lt $TIME_0 ]
    then
      echo "$CURRENT_TIMESTR : RECV appears to be hung no msg processed"  >> $LOGFILE
      echo "$CURRENT_TIMESTR : RECV appears to be hung no msg processed"  
      RESTART=1
    fi
  fi  

  if [ $RESTART -eq 1 ]
  then
    echo "$CURRENT_TIMESTR : Restarting ROS"  >> $LOGFILE
    echo "$CURRENT_TIMESTR : Restarting ROS"  
#    mkdir  /tmp/jef_${TIME_0}
#    cp -f /tmp/*.log /tmp/jef_${TIME_0}/
#    cp -f /tmp/*lock /tmp/jef_${TIME_0}/
    $STOP_CMD
    sleep 10
    $START_CMD
    echo "$CURRENT_TIMESTR : Restart Complete"  >> $LOGFILE
    echo "$CURRENT_TIMESTR : Restart Complete"  
  fi
fi
