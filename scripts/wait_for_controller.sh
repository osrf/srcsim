#!/bin/bash

bridge=""
while [ -z "$bridge" ]; do
  bridge=`rosparam list | grep ihmc_valkyrie_control_java_bridge`
  sleep 1
  echo "waiting for ihmc_valkyrie_control_java_bridge"
done

echo "Found ihmc_valkyrie_control_java_bridge! Launching grasp_init."

sleep 5

roslaunch srcsim grasping_init.launch
