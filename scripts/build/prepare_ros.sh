#!/bin/bash
# Do not fail if a command fails
set +e

# Ignore DART
touch deps/dart/CATKIN_IGNORE

# The symlink needs to be removed since ROS doesn't work with it
rm ./deps/limbo/exp/skireil

# Service message files need to be removed, otherwise ROS doesn't build them
rm ./include/skireil/ManageSkirosWorkerRequest.h
rm ./include/skireil/ManageSkirosWorkerResponse.h
rm ./include/skireil/NextActionRequest.h
rm ./include/skireil/NextActionResponse.h
rm ./include/skireil/SkirosNextActionRequest.h
rm ./include/skireil/SkirosNextActionResponse.h
rm ./include/skireil/ParamFloat.h
rm ./include/skireil/ParamInt.h
rm ./include/skireil/ParamString.h
rm ./include/skireil/ros_msgs/*

