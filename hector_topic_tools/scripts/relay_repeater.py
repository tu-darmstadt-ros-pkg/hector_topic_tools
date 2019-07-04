#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function, division

import argparse

import rospy
import rostopic


class RelayRepeater:
    def __init__(self):
        self.args = self.parse_arguments()

        input_class, input_topic, self.input_fn = rostopic.get_topic_class(self.args.input_topic,
                                                                           blocking=self.args.wait_for_start)
        if input_topic is None:
            rospy.logerr('ERROR: Invalid input topic: {}'.format(self.args.input_topic))
            exit(1)

        self.msg_list = []
        self.pub = rospy.Publisher(self.args.output_topic, input_class, queue_size=1000)
        self.sub = rospy.Subscriber(input_topic, input_class, self.callback)

        self.timer = rospy.Timer(rospy.Duration(self.args.publish_period), self.timer_callback)

    @staticmethod
    def parse_arguments():
        parser = argparse.ArgumentParser(description='Republish and repeat all messages from one topic to another')
        parser.add_argument('input_topic', help='Source topic of messages')
        parser.add_argument('output_topic', help='Target topic for messages')
        parser.add_argument("--publish_period", type=float, default=5.0, help="Desired publishing period of messages "
                                                                              "on the target topic in s.")
        parser.add_argument('--wait-for-start', action='store_true', help='Wait for input messages.')

        try:
            import argcomplete
        except ImportError:
            argcomplete = None
        else:
            argcomplete.autocomplete(parser)
        args = parser.parse_args(rospy.myargv()[1:])
        return args

    def callback(self, m):
        if self.input_fn is not None:
            m = self.input_fn(m)

        self.msg_list.append(m)
        rospy.loginfo("Republishing {} messages.".format(len(self.msg_list)))

        self.pub.publish(m)

    def timer_callback(self, _):
        for m in self.msg_list:
            self.pub.publish(m)


if __name__ == "__main__":
    rospy.init_node('relay_field', anonymous=True)
    repeater = RelayRepeater()
    rospy.spin()
