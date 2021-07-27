#!/usr/bin/env python
import sys
import threading
import unittest
import time
import rostest
import rospy
from  araig_msgs.msg import BoolStamped

class TestEdgeTrigger(unittest.TestCase):

    def setUp(self):
        _pub_topic_1 = '/out_bool'
        _sub_topic_high = '/in_high'
        _sub_topic_low = '/in_low'

        rospy.init_node('test_edge_detector', anonymous=True)
        self.pub_1 = rospy.Publisher(_pub_topic_1, BoolStamped, latch=True, queue_size=10)

        rospy.Subscriber(_sub_topic_high, BoolStamped, callback=self.callback_high, queue_size=10)
        rospy.Subscriber(_sub_topic_low, BoolStamped, callback=self.callback_low, queue_size=10)

        self.result_high = None
        self.result_low = None

        self.mutex = threading.Lock()

    def test_expected(self):
        pub_msg = BoolStamped()

        # Test1
        pub_msg.header.stamp = rospy.Time().now()
        pub_msg.data = True
        self.result_high = None
        self.result_low = None
        self.pub_1.publish(pub_msg)

        while not rospy.is_shutdown():
            with self.mutex:
                l_result_high = self.result_high
                l_result_low = self.result_low
            if l_result_high != None and l_result_low != None:
                break

        self.assertTrue(self.result_high, msg='Test1 rising edge - High failed')
        self.assertFalse(self.result_low, msg='Test1 rising edge - Low failed')

        # Test2
        pub_msg = BoolStamped()
        pub_msg.data = False
        self.result_high = None
        self.result_low = None
        self.pub_1.publish(pub_msg)

        while not rospy.is_shutdown():
            with self.mutex:
                l_result_high = self.result_high
                l_result_low = self.result_low
            if l_result_high != None and l_result_low != None:
                break

        self.assertFalse(self.result_high, msg='Test2 rising edge - High failed')
        self.assertTrue(self.result_low, msg='Test2 rising edge - Low failed')

        # Test3
        pub_msg.header.stamp = rospy.Time().now()
        pub_msg.data = True
        self.result_high = None
        self.result_low = None
        self.pub_1.publish(pub_msg)

        while not rospy.is_shutdown():
            with self.mutex:
                l_result_high = self.result_high
                l_result_low = self.result_low
            if l_result_high != None and l_result_low != None:
                break

        self.assertTrue(self.result_high, msg='Test3 rising edge - High failed')
        self.assertFalse(self.result_low, msg='Test3 rising edge - Low failed')

    def callback_high(self, msg):
        with self.mutex:
            self.result_high = msg.data

    def callback_low(self, msg):
        with self.mutex:
            self.result_low = msg.data

if __name__ == '__main__':
    pkg = 'araig_calculators'
    name = 'test_edge_detector'
    rostest.rosrun(pkg, name, TestEdgeTrigger)