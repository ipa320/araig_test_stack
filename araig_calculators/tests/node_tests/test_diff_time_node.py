#!/usr/bin/env python

import sys
import unittest
import time
import rostest
import rospy
from geometry_msgs.msg import PoseStamped
from  araig_msgs.msg import BoolStamped, Float64Stamped

class TestDiffTime(unittest.TestCase):

    def setUp(self):
        _pub_topic_1 = '/test/start'
        _pub_topic_2 = '/test/stop'
        _sub_topic_1 = '/test/duration'

        rospy.init_node('test_diff_time', anonymous=True)
        self.pub_start = rospy.Publisher(_pub_topic_1, BoolStamped, latch=True, queue_size=10)
        self.pub_stop = rospy.Publisher(_pub_topic_2, BoolStamped, latch= True, queue_size=10)

        rospy.Subscriber(_sub_topic_1, Float64Stamped, callback=self.callback_1, queue_size=10)

        self.duration = None
        self.msg_seq = 0
        self.count = 0

    def test_expected(self):
        while self.count < 3:
            self.count += 1
            # pub stop signal first
            pub_signal = BoolStamped()
            pub_signal.header.stamp = rospy.Time.now()
            pub_signal.data = True
            self.pub_stop.publish(pub_signal)
            rospy.sleep(1)

            # pub start signal
            pub_signal.header.stamp = rospy.Time.now()
            pub_signal.data = True
            self.pub_start.publish(pub_signal)
            rospy.sleep(0.1)
            # reset stop signal
            pub_signal.data = False
            self.pub_stop.publish(pub_signal)
            rospy.sleep(0.3)

            pub_signal.header.stamp = rospy.Time.now()
            pub_signal.data = True
            self.pub_stop.publish(pub_signal)
        
            while self.duration is None:
                time.sleep(0.1)
            self.assertAlmostEqual(self.duration, 0.4, None, msg='test{}: {} is not equal to 0.3'.format(self.count, self.duration), delta=0.05)
            self.assertEqual(self.msg_seq, self.count, 'msg published {} times'.format(self.msg_seq))

            # reset
            self.duration = None
            pub_signal.data = False
            pub_signal.header.stamp = rospy.Time.now()
            self.pub_start.publish(pub_signal)
            rospy.sleep(0.1)
            pub_signal.header.stamp = rospy.Time.now()
            self.pub_stop.publish(pub_signal)
            rospy.sleep(0.1)

        # pub start signal
        pub_signal.header.stamp = rospy.Time.now()
        pub_signal.data = True
        self.pub_start.publish(pub_signal)
        rospy.sleep(0.1)
        rospy.sleep(0.3)

        pub_signal.header.stamp = rospy.Time.now()
        pub_signal.data = True
        self.pub_stop.publish(pub_signal)
    
        while self.duration is None:
            time.sleep(0.1)
        self.assertAlmostEqual(self.duration, 0.4, None, msg='test{}: {} is not equal to 0.3'.format(self.count+1, self.duration), delta=0.05)
        self.assertEqual(self.msg_seq, self.count+1, 'msg published {} times'.format(self.msg_seq))

    def callback_1(self, msg):
        self.duration = msg.data
        self.msg_seq = msg.header.seq

if __name__ == '__main__':
    pkg = 'araig_calculators'
    name = 'test_diff_time'
    rostest.rosrun(pkg, name, TestDiffTime)