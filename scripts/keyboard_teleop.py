#!/usr/bin/env python

from collections import OrderedDict
import select
import sys
import termios
import tty

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import HeadTrajectoryRosMessage
from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

import rospy

from tf.transformations import quaternion_from_euler


class KeyboardTeleop(object):

    ARM_BINDINGS = OrderedDict([
        ('q', {'joint_index': 0, 'side': 'left', 'min': -2.85, 'max': 2.0}),  # leftShoulderPitch
        ('w', {'joint_index': 1, 'side': 'left', 'min': -1.519, 'max': 1.266,
               'invert': True}),  # leftShoulderRoll
        ('e', {'joint_index': 2, 'side': 'left', 'min': -3.1, 'max': 2.18}),  # leftShoulderYaw
        ('r', {'joint_index': 3, 'side': 'left', 'min': -2.174, 'max': 0.12,
               'invert': True}),  # leftElbowPitch
        ('a', {'joint_index': 4, 'side': 'left', 'min': -2.019, 'max': 3.14}),  # leftForearmYaw
        ('s', {'joint_index': 5, 'side': 'left', 'min': -0.62, 'max': 0.625,
               'invert': True}),  # leftWristRoll
        ('d', {'joint_index': 6, 'side': 'left', 'min': -0.36, 'max': 0.49,
               'invert': True}),  # leftWristPitch
        ('t', {'joint_index': 'reset', 'side': 'left'}),

        ('u', {'joint_index': 0, 'side': 'right', 'min': -2.85, 'max': 2.0}),  # rightShoulderPitch
        ('i', {'joint_index': 1, 'side': 'right', 'min': -1.266, 'max': 1.519}),  # rightShoulderRoll
        ('o', {'joint_index': 2, 'side': 'right', 'min': -3.1, 'max': 2.18}),  # rightShoulderYaw
        ('p', {'joint_index': 3, 'side': 'right', 'min': -0.12, 'max': 2.174}),  # rightElbowPitch
        ('j', {'joint_index': 4, 'side': 'right', 'min': -2.019, 'max': 3.14}),  # rightForearmYaw
        ('k', {'joint_index': 5, 'side': 'right', 'min': -0.625, 'max': 0.62}),  # rightWristRoll
        ('l', {'joint_index': 6, 'side': 'right', 'min': -0.49, 'max': 0.36}),  # rightWristPitch
        ('y', {'joint_index': 'reset', 'side': 'right'}),
    ])

    HEAD_BINDINGS = OrderedDict([
        ('x', {'joint_index': 0, 'min': -0.5, 'max': 0.5}),
        ('c', {'joint_index': 1, 'min': -0.5, 'max': 0.5}),
        ('v', {'joint_index': 2, 'min': -1.0, 'max': 1.0}),
        ('g', {'joint_index': 'reset'}),
    ])

    NECK_BINDINGS = OrderedDict([
        ('b', {'joint_index': 0, 'min': 0.0, 'max': 0.5}),  # lowerNeckPitch? 'min': 0, 'max': 1.162
        ('n', {'joint_index': 1, 'min': -1.0, 'max': 1.0}),  # neckYaw? 'min': -1.047, 'max': 1.047
        ('m', {'joint_index': 2, 'min': -0.5, 'max': 0.0}),  # upperNeckPitch? 'min': -0.872, 'max': 0
        ('h', {'joint_index': 'reset'}),
    ])

    def __init__(self):
        self.joint_values = {
            'left arm': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'right arm': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'head': [0.0, 0.0, 0.0],
            'neck': [0.0, 0.0, 0.0],
        }
        # ensure no characters are bound repeatedly
        bindings = [self.ARM_BINDINGS, self.HEAD_BINDINGS, self.NECK_BINDINGS]
        keys = set([])
        [keys.update(b.keys()) for b in bindings]
        assert len(keys) == sum([len(b) for b in bindings]), 'Duplicate binding'

    def run(self):
        try:
            self.init()
            self.print_usage()
            while not rospy.is_shutdown():
                ch = self.get_key()
                self.process_key(ch)
        finally:
            self.fini()

    def init(self):
        # save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        # create publisher for arm trajectories
        robot_name = rospy.get_param('/ihmc_ros/robot_name')
        self.arm_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/arm_trajectory'.format(robot_name),
            ArmTrajectoryRosMessage, queue_size=1)
        self.neck_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/neck_trajectory'.format(robot_name),
            NeckTrajectoryRosMessage, queue_size=1)
        self.head_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/head_trajectory'.format(robot_name),
            HeadTrajectoryRosMessage, queue_size=1)

        # make sure the simulation is running otherwise wait
        rate = rospy.Rate(10)  # 10hz
        publishers = [
            self.arm_publisher, self.neck_publisher, self.head_publisher]
        if any([p.get_num_connections() == 0 for p in publishers]):
            rospy.loginfo('waiting for subscriber...')
            while any([p.get_num_connections() == 0 for p in publishers]):
                rate.sleep()

    def fini(self):
        self.arm_publisher = None

        # restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def print_usage(self):
        def get_bound_char(bindings, x):
            return x.upper() if bindings[x].get('invert', False) else x

        msg_args = []

        # left arm - first row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[0:4]]))
        # left arm - second row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[4:7]]))
        # left arm - reset
        msg_args.append(' or '.join([
            self.ARM_BINDINGS.keys()[7], self.ARM_BINDINGS.keys()[7].upper()]))

        # right arm - first row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[8:12]]))
        # right arm - second row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[12:15]]))
        # right arm - reset
        msg_args.append(' or '.join([
            self.ARM_BINDINGS.keys()[15], self.ARM_BINDINGS.keys()[15].upper()]))

        # head - roll, pitch, yaw
        msg_args += [
            get_bound_char(self.HEAD_BINDINGS, x)
            for x in self.HEAD_BINDINGS.keys()[0:3]]
        # head - reset
        msg_args.append(' or '.join([
            self.HEAD_BINDINGS.keys()[3], self.HEAD_BINDINGS.keys()[3].upper()]))

        # neck
        msg_args.append(', '.join([
            get_bound_char(self.NECK_BINDINGS, x)
            for x in self.NECK_BINDINGS.keys()[0:3]]))
        # neck - reset
        msg_args.append(' or '.join([
            self.NECK_BINDINGS.keys()[3], self.NECK_BINDINGS.keys()[3].upper()]))

        msg = """
            Keyboard Teleop for Space Robotics Challenge 0.1.0
            Copyright (C) 2017 Open Source Robotics Foundation
            Released under the Apache 2 License
            --------------------------------------------------
            Left arm joints: {}
                             {}
                Reset: {}

            Right arm joints: {}
                              {}
                Reset: {}

            Head angles:
                Roll: {}
                Pitch: {}
                Yaw: {}
                Reset all head angles: {}

            Neck joints: {}
                Reset all neck joints: {}

            The shown characters turn the joints in positive direction.
            The opposite case turns the joints in negative direction.

            ?: Print this menu
            <TAB>: Print all current joint values
            <ESC>: Quit
            """.format(*msg_args)
        self.loginfo(msg)

    def get_key(self):
        """Get input from the terminal."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        return key

    def process_key(self, ch):
        """Process key event."""
        if ch == '?':
            self.print_usage()
            return

        if ord(ch) == 9:  # TAB key
            for label in sorted(self.joint_values.keys()):
                self.loginfo(
                    'Current joint values for the %s: %s' %
                    (label, self.joint_values[label]))
            return

        if ord(ch) == 27:  # ESCAPE key
            self.loginfo('Quitting')
            rospy.signal_shutdown('Shutdown')
            return

        if ch.lower() in self.ARM_BINDINGS:
            self.process_arm_command(self.ARM_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.HEAD_BINDINGS:
            self.process_head_command(self.HEAD_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.NECK_BINDINGS:
            self.process_neck_command(self.NECK_BINDINGS[ch.lower()], ch)
            return

    def process_arm_command(self, binding, ch):
        msg = ArmTrajectoryRosMessage()
        msg.unique_id = -1

        side = binding['side']
        if side == 'left':
            msg.robot_side = ArmTrajectoryRosMessage.LEFT
        elif side == 'right':
            msg.robot_side = ArmTrajectoryRosMessage.RIGHT
        else:
            assert False, "Unknown arm side '%s'" % side

        self._update_joint_values('%s arm' % side, binding, ch)
        self._append_trajectory_point_1d(
            msg, 1.0, self.joint_values['%s arm' % side])

        self.arm_publisher.publish(msg)

    def process_head_command(self, binding, ch):
        msg = HeadTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('head', binding, ch)
        self._append_trajectory_point_so3(
            msg, 1.0, self.joint_values['head'])
        self.head_publisher.publish(msg)

    def process_neck_command(self, binding, ch):
        msg = NeckTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('neck', binding, ch)
        self._append_trajectory_point_1d(
            msg, 1.0, self.joint_values['neck'])
        self.neck_publisher.publish(msg)

    def _update_joint_values(self, label, binding, ch):
        joint_index = binding['joint_index']
        if joint_index == 'reset':
            # reset all joints to zero
            self.loginfo('Reset %s' % label)
            self.joint_values[label] = [0.0] * len(self.joint_values[label])

        else:
            # update value within boundaries
            value = self.joint_values[label][joint_index]
            is_lower_case = ch == ch.lower()
            factor = 1.0 if is_lower_case else -1.0
            if binding.get('invert', False):
                factor *= -1.0
            STEP = 0.1
            value += STEP * factor
            value = min(max(value, binding['min']), binding['max'])
            self.joint_values[label][joint_index] = value

            self.loginfo(
                'Move %s joint #%d %s to %.1f: [%s]' %
                (label, joint_index, '++' if (is_lower_case != binding.get('invert', False)) else '--',
                 self.joint_values[label][joint_index],
                 ', '.join(
                     ['%.1f' % v for v in self.joint_values[label]])))

    def _append_trajectory_point_1d(self, msg, time, joint_values):
        if not msg.joint_trajectory_messages:
            msg.joint_trajectory_messages = [
                OneDoFJointTrajectoryRosMessage() for _ in joint_values]
        for i, joint_value in enumerate(joint_values):
            point = TrajectoryPoint1DRosMessage()
            point.time = time
            point.position = joint_value
            point.velocity = 0
            msg.joint_trajectory_messages[i].trajectory_points.append(point)

    def _append_trajectory_point_so3(self, msg, time, joint_values):
        roll, pitch, yaw = joint_values
        quat = quaternion_from_euler(roll, pitch, yaw)
        point = SO3TrajectoryPointRosMessage()
        point.time = time
        point.orientation = Quaternion()
        point.orientation.x = quat[0]
        point.orientation.y = quat[1]
        point.orientation.z = quat[2]
        point.orientation.w = quat[3]
        point.angular_velocity = Vector3()
        point.angular_velocity.x = 0
        point.angular_velocity.y = 0
        point.angular_velocity.z = 0
        msg.taskspace_trajectory_points.append(point)

    def loginfo(self, msg):
        """Log info message while terminal is in funky mode."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        rospy.loginfo(msg)
        tty.setraw(sys.stdin.fileno())


if __name__ == '__main__':
    rospy.init_node('keyboard_teleop')
    teleop = KeyboardTeleop()
    teleop.run()
