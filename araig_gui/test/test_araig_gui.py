#!/usr/bin/env python2.7
import rospy
from araig_msgs.msg import BoolStamped

import numpy as np

class node():
    def __init__(self):
        rospy.init_node('test_node')
        self._pub_completed = rospy.Publisher("/signal/runner/test_completed", BoolStamped, queue_size=10)
        self._pub_succ = rospy.Publisher("/signal/runner/test_succeeded", BoolStamped, queue_size=10)
        self._pub_fail = rospy.Publisher("/signal/runner/test_failed", BoolStamped, queue_size=10)
        self._sub_start = rospy.Subscriber("/signal/ui/start_test", BoolStamped, getattr(self, "callback1"))
        self._sub_stop = rospy.Subscriber("/signal/ui/interrupt_test", BoolStamped, getattr(self, "callback2"))
        self._sub_reset = rospy.Subscriber("/signal/ui/reset_test", BoolStamped, getattr(self, "callback3"))
        self._msg_completed = BoolStamped()
        self._msg_succ = BoolStamped()
        self._msg_fail = BoolStamped()
    
    def callback1(self, msg):
        if(msg.data):
            rospy.loginfo("[TESTNODE]: Received start signal!")
            rospy.loginfo("[TESTNODE]: Test started!")
            rospy.sleep(3)
            self._msg_completed.data = True
            self._pub_completed.publish(self._msg_completed)
            rospy.loginfo("[TESTNODE]: Test completed!")
            if(np.random.random(size=1)[0]<0.5):
                self._msg_succ.data = True
                self._pub_succ.publish(self._msg_succ)
                rospy.loginfo("[TESTNODE]: Test succeeded!")
            else:
                self._msg_fail.data = True
                self._pub_fail.publish(self._msg_fail)
                rospy.loginfo("[TESTNODE]: Test failed!")
        else:
            pass
    
    def callback2(self, msg):
        if(msg.data):
            rospy.loginfo("[TESTNODE]: Received terminate signal!")
            rospy.loginfo("[TESTNODE]: Test terminated!")
            rospy.loginfo("[TESTNODE]: Please reset test!")
        else:
            pass

    def callback3(self, msg):
        if(msg.data):
            rospy.loginfo("[TESTNODE]: Received reset signal!")
            rospy.loginfo("[TESTNODE]: Test reseted!")
        else:
            pass
    

if __name__ == '__main__':
    try:        
        node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
