#!/bin/bash

bridge=""
while [ -z "$bridge" ]; do
  bridge=`rosservice call /controller_manager/list_controllers | grep ihmc_valkyrie_control_java_bridge`
  sleep 1
  echo "waiting for ihmc_valkyrie_control_java_bridge"
done

echo "Found ihmc_valkyrie_control_java_bridge! Launching grasp_init."

roslaunch srcsim grasping_init.launch
