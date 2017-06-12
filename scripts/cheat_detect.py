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


class CheatDetectNode(object):
    def __init__(self):
        rospy.init_node("GazeboHelper")
        cheat_pubs = {
                "/gazebo/link_states": LinkStates,
                "/gazebo/model_states": ModelStates,
                "/gazebo/parameter_descriptions": ConfigDescription,
                "/gazebo/parameter_updates": Config,
        }

        cheat_subs = {
                "/gazebo/set_link_state": LinkState,
                "/valkyrie/harness/detach": Bool,
                "/valkyrie/harness/velocity": Float32,
                "/gazebo/set_model_state": ModelState,
        }

        self.pubs = list()
        self.subs = list()
        self.pub_counts = []
        self.sub_counts = []
        for name, message_type in cheat_pubs.items():
            pub_info = {"name": name, "pub": None, }
            pub_info["pub"] = rospy.Publisher(name, message_type, queue_size=1)
            self.pubs.append(pub_info)
            self.pub_counts.append(0)
        for name, message_type in cheat_subs.items():
            sub_info = {"name": name, "sub": None}
            sub_info["sub"] = rospy.Subscriber(name, message_type, self._callback)
            self.subs.append(sub_info)
            self.sub_counts.append(0)

    def _callback(self, msg):
        rospy.logerr("Cheat detected (received message): %s", msg)

    def wait_for_cheaters(self):
        while not rospy.is_shutdown():
            for idx, pub in enumerate(self.pubs):
                if pub["pub"].get_num_connections() != self.pub_counts[idx]:
                    rospy.logerr("Cheat detected (something subscribed) %r", pub["pub"].impl.get_stats())
                    self.pub_counts[idx] = pub["pub"].get_num_connections()
            for idx, sub in enumerate(self.subs):
                if sub["sub"].get_num_connections() != self.sub_counts[idx]:
                    rospy.logerr("Cheat detected (something published) %r", sub["sub"].impl.get_stats())
                    self.sub_counts[idx] = sub["sub"].get_num_connections()
            rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        node = CheatDetectNode()
        node.wait_for_cheaters()
    except rospy.ROSInterruptException:
        pass
