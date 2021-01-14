#!/usr/bin/env python

import sys
import unittest
import time
import rostest
import rospy
from geometry_msgs.msg import PoseStamped
from  araig_msgs.msg import BoolStamped
from std_msgs.msg import Float64

class TestCalcStopWatch(unittest.TestCase):

    def setUp(self):
        _pub_topic_1 = '/test/start'
        _pub_topic_2 = '/test/stop'
        _sub_topic_1 = '/test/duration'

        rospy.init_node('test_calc_stop_watch', anonymous=True)
        self.pub_1 = rospy.Publisher(_pub_topic_1, BoolStamped, latch=True, queue_size=10)
        self.pub_2 = rospy.Publisher(_pub_topic_2, BoolStamped, latch= True, queue_size=10)

        rospy.Subscriber(_sub_topic_1, Float64, callback=self.callback_1, queue_size=10)

        self.duration = None

    def test_expected(self):

        # pub start signal
        pub_signal = BoolStamped()
        pub_signal.header.stamp = rospy.Time.now()
        pub_signal.data = True
        self.pub_1.publish(pub_signal)
        rospy.sleep(2.0)

        pub_signal.header.stamp = rospy.Time.now()
        pub_signal.data = True
        self.pub_2.publish(pub_signal)
        
        while self.duration is None:
            time.sleep(0.1)

        self.assertAlmostEqual(self.duration, 2.0, None, msg='{} is not equal to 2.0'.format(self.duration), delta=0.05)

    def callback_1(self, msg):
        self.duration = msg.data

if __name__ == '__main__':
    pkg = 'araig_calculators'
    name = 'test_calc_stop_watch'
    rostest.rosrun(pkg, name, TestCalcStopWatch)