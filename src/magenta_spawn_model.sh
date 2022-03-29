#!/bin/bash			

### source the workspace
source ../devel/setup.bash

### Get parameters and init
declare -i j
declare -i kill_num
magenta_prefix=$(rosparam get /magenta/prefix)
magenta_num=$(rosparam get /magenta/num)
kill_num=0                                  

### spawn magenta robots
for ((i=1; i<=magenta_num; ++i))
do
    # export AGENT=$j
    rosrun world_model      world_model_node   ${magenta_prefix}${i}    __name:=${magenta_prefix}_world_model${i} &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
    
    rosrun nubot_control    nubot_control_node ${magenta_prefix}${i}   __name:=${magenta_prefix}_nubot_control${i} &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
    sleep 0.5
done 

######### Don't to use RTDB for convenience. Use "rostopic pub" to publish game control
########  info instead.

### run rtdt comm
#rosrun world_model comm &
#PIDS[kill_num]=$!
#let "kill_num=kill_num+1"

### run coachinfo_publisher
# sleep 5
# rosrun  simulation_interface coach_robot_comm_RTDB  ${magenta_prefix}${j} __name:=${magenta_prefix}_coach_robot_comm_RTDB &
# PIDS[kill_num]=$!
# let "kill_num=kill_num+1"

### run strategy_pub_node
rosrun simulation_interface strategy_pub_node ${magenta_prefix} __name:=${magenta_prefix}_strategy_pub &
PIDS[kill_num]=$!
let "kill_num=kill_num+1"

### Optional: run joystick drivers to control football movement
# rosrun joy joy_node &
# PIDS[kill_num]=$!

### kill thoes background processes
trap 'kill ${PIDS[*]}' SIGINT
wait

