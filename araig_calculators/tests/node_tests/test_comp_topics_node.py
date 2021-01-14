#!/usr/bin/env python
import sys
import unittest
import time
import rostest
import rospy
import rostopic
from  araig_msgs.msg import BoolStamped


class TestCompParam(unittest.TestCase):

    def setUp(self):
        _pub_topic_1 = "/test/in_bool_1"
        _pub_topic_2 = "/test/in_bool_2"
        _sub_topic = "/test/out_bool"

        rospy.init_node('test_comp_param', anonymous=True)
        self.pub_1 = rospy.Publisher(_pub_topic_1, BoolStamped, latch=True, queue_size=10)
        self.pub_2 = rospy.Publisher(_pub_topic_2, BoolStamped, latch=True, queue_size=10)

        rospy.Subscriber(_sub_topic, BoolStamped, callback=self.callback_1, queue_size=10)

        while (not rospy.has_param("/calculators/comp_topics_node/tolerance")):
            time.sleep(0.1)
        
        self.tolerance = rospy.get_param("/calculators/comp_topics_node/tolerance")
        self.result = None

    def test_expected(self):
        pub_msg = BoolStamped()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.data = True
        self.pub_1.publish(pub_msg)

        pub_msg = BoolStamped()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.data = True
        self.pub_2.publish(pub_msg)

        while self.result is None:
            time.sleep(0.1)

        self.assertTrue(self.result, msg='Compare with Tolerance Failed')

    def callback_1(self, msg):
        self.result = msg.data

if __name__ == '__main__':
    pkg = 'araig_calculators'
    name = 'test_comp_topics'
    rostest.rosrun(pkg, name, TestCompParam)