#!/usr/bin/env python
import sys
import unittest
import time
import rostest
import rospy
from  araig_msgs.msg import BoolStamped
from std_msgs.msg import Float64

class TestCompParam(unittest.TestCase):

    def setUp(self):
        _pub_topic_1 = '/test/in_float'
        _sub_topic_1 = '/test/out_bool'

        rospy.init_node('test_comp_param', anonymous=True)
        self.pub_1 = rospy.Publisher(_pub_topic_1, Float64, latch=True, queue_size=10)

        rospy.Subscriber(_sub_topic_1, BoolStamped, callback=self.callback_1, queue_size=10)

        while (not rospy.has_param("/calculators/test_comp_param_node/param") and \
            not rospy.has_param("/calculators/test_comp_param_node/tolerance")):
            time.sleep(0.1)
        
        self.param = rospy.get_param("/calculators/test_comp_param_node/param")
        self.tolerance = rospy.get_param("/calculators/test_comp_param_node/tolerance")
        self.result = None

    def test_expected(self):
        pub_msg = Float64()
        pub_msg.data = self.param
        self.pub_1.publish(pub_msg)

        while self.result is None:
            time.sleep(0.1)
        self.assertTrue(self.result, msg='Compare without Tolerance Failed')

        pub_msg = Float64()
        pub_msg.data = self.param + self.tolerance - 0.001
        self.pub_1.publish(pub_msg)

        self.result = None
        while self.result is None:
            time.sleep(0.1)
        self.assertTrue(self.result, msg='Compare with Tolerance Failed')

    def callback_1(self, msg):
        self.result = msg.data

if __name__ == '__main__':
    pkg = 'araig_calculators'
    name = 'test_comp_param'
    rostest.rosrun(pkg, name, TestCompParam)