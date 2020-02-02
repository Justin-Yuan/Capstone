#!/bin/bash
# pipeline for running contest1 
# reference: https://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
# reference: https://unix.stackexchange.com/questions/471931/how-to-create-a-bunch-of-background-processes-in-terminator-terminals-later-kill

# enable job control  
set -m

# options 
EXECUTABLE="contest1"
SAVE_MAP=false
MAP_NAME="map"
LOAD_SIM="roslaunch mie443_contest1 turtlebot_world.launch"
LOAD_REAL="roslaunch turtlebot_bringup minimal.launch"
LOAD_CMD=$LOAD_SIM

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -e|--executable)
    EXECUTABLE="$2"
    shift # past argument
    shift # past value
    ;;
    -r|--real)
    LOAD_CMD=$LOAD_REAL
    shift # past argument
    ;;
    -s|--save_map)
    SAVE_MAP=true
    shift # past bool argument
    ;;
    -m|--map_name)
    MAP_NAME="$2"
    shift # past argument
    shift # past value
    ;;
    --default)
    DEFAULT=YES
    shift # past argument
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters


# main service 
# terminator -T "roscore" -e "roscore" &
# roscore_pid=$!
# sleep 2 

# load map and bot 
# terminator -T "world" -e "roslaunch mie443_contest1 turtlebot_world.launch; bash" &
terminator -T "world" -e "$LOAD_CMD; bash" &
world_pid=$!
sleep 10 || exit

# slam 
terminator -T "slam" -e "roslaunch mie443_contest1 gmapping.launch; bash" &
gmapping_pid=$!
# sleep 2 

# rviz 
terminator -T "rviz" -e "roslaunch turtlebot_rviz_launchers view_navigation.launch; bsah" &  
rviz_pid=$!
# sleep 2 

# control 
terminator -T "control" -e "rosrun mie443_contest1 $EXECUTABLE" &
# terminator -T "control" -e "sleep 5" &
control_pid=$! 

# wait for shutdown signal 
echo Press ENTER to shutdown all processes
read 
echo Shutting down...

# save map
if [ "$SAVE_MAP" = true ] ; then
    echo Saving map... 
    rosrun map_server map_saver –f $MAP_NAME 
fi

# kill all spawned terminals 
kill $(jobs -p)

# wait $control_pid
# echo "control ended..."
# kill -9 $rviz_pid
# kill -9 $gmapping_pid
# kill -9 $world_pid
# kill -9 $roscore_pid
# echo "run ended..."
