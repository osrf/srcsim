#!/usr/bin/env python

import rospy

from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import LinkState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from dynamic_reconfigure.msg import Config
from dynamic_reconfigure.msg import ConfigDescription
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose


class CheatDetectNode(object):
    def __init__(self):
        rospy.init_node("GazeboHelper")
        cheat_pubs = {
                "/gazebo/link_states": LinkStates,
                "/gazebo/model_states": ModelStates,
                "/gazebo/parameter_descriptions": ConfigDescription,
                "/gazebo/parameter_updates": Config,
                "/srcsim/finals/force_checkpoint_completion": Empty,
                "/valkyrie/harness/attach": Pose,
        }

        cheat_subs = {
                "/gazebo/set_link_state": LinkState,
                "/valkyrie/harness/detach": Bool,
                "/valkyrie/harness/velocity": Float32,
                "/gazebo/set_model_state": ModelState,
        }

        self.pubs = list()
        self.subs = list()
        for name, message_type in cheat_pubs.items():
            pub_info = {"name": name, "pub": None, }
            pub_info["pub"] = rospy.Publisher(name, message_type, queue_size=1)
            self.pubs.append(pub_info)
        for name, message_type in cheat_subs.items():
            sub_info = {"name": name, "sub": None}
            sub_info["sub"] = rospy.Subscriber(name, message_type, self._callback)
            self.subs.append(sub_info)

    def _callback(self, msg):
        rospy.logerr("Cheat detected (received message): %s", msg)

    def wait_for_cheaters(self):
        while not rospy.is_shutdown():
            for pub in self.pubs:
                if pub["pub"].get_num_connections() != 0:
                    rospy.logerr("Cheat detected (something subscribed) %r", pub["pub"].impl.get_stats())
            for sub in self.subs:
                if sub["sub"].get_num_connections() != 0:
                    rospy.logerr("Cheat detected (something published) %r", sub["sub"].impl.get_stats())
            rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        node = CheatDetectNode()
        node.wait_for_cheaters()
    except rospy.ROSInterruptException:
        pass
