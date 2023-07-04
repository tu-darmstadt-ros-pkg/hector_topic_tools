#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function, division

import argparse

import rospy
import sensor_msgs.msg
import math


class DataCollector:
    def __init__(self):
        self.joints = self.parse_arguments().joints
        rospy.loginfo(f"Tracking joints: {self.joints}")
        self.max = dict()
        for joint in self.joints:
            self.max[joint] = 0.0

        self.sub = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.callback)

    @staticmethod
    def parse_arguments():
        parser = argparse.ArgumentParser(description='TODO')
        parser.add_argument('joints', help='Joints to track', nargs='+')

        try:
            import argcomplete
        except ImportError:
            argcomplete = None
        else:
            argcomplete.autocomplete(parser)
        args = parser.parse_args(rospy.myargv()[1:])
        return args

    def callback(self, joint_state_msg):
        for joint, maximum in self.max.items():
            try:
                joint_index = joint_state_msg.name.index(joint)
            except ValueError as e:
                continue

            effort = math.fabs(joint_state_msg.effort[joint_index])
            self.max[joint] = max(effort, maximum)

        self.print_maximum()

    def print_maximum(self):
        # output = ""
        # for key, value in self.max.items():
        #     output += key + ": " + str(value)
        rospy.loginfo(f"{self.max}")


if __name__ == "__main__":
    rospy.init_node('joint_effort_max', anonymous=True)
    collector = DataCollector()
    rospy.spin()
