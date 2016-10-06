#!/bin/bash
echo "setup gazebo paths"
export tmp=/root/local
export PKG_CONFIG_PATH=${tmp}/lib/pkgconfig:${tmp}/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}
export PATH=${tmp}/bin:${PATH}
export LIBRARY_PATH=${tmp}/lib:${LIBRARY_PATH}
export LD_LIBRARY_PATH=${tmp}/lib:${tmp}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
source ${tmp}/share/gazebo/setup.sh

echo "wait 60s for system to load"
sleep 60

echo "lower harness"
rostopic pub -1 /valkyrie/harness/velocity std_msgs/Float32 '{data: -0.05}'

echo "stop lowering after 20 seconds (optional)"
sleep 20
rostopic pub -1 /valkyrie/harness/velocity std_msgs/Float32 '{data: 0}'

echo "switch to high level control in 3 seconds"
sleep 3
rostopic pub -1 /ihmc_ros/valkyrie/control/low_level_control_mode ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage '{requested_control_mode: 2, unique_id: -1}'

echo "pause then detach in 5 seconds"
sleep 5
gz world -p 1
sleep 1
rostopic pub -1 /valkyrie/harness/detach std_msgs/Bool true &
sleep 1
gz world -p 0

echo "done"
