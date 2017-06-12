#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import LinkState

def callback(msg):
  rospy.logerr("I am a cheater")

rospy.init_node("CheatTest")

pub = rospy.Publisher("/gazebo/set_link_state", LinkState)
sub = rospy.Subscriber("/gazebo/link_states", LinkStates, callback)

rospy.spin()
