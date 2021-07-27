#!/usr/bin/env python
import rospy
import threading
from std_msgs.msg import Float64

from araig_msgs.msg import BoolStamped
from base_classes.base_calculator import BaseCalculator

"""Compare bool from a topic, publish and latch a level shift
    pub_list = {"out_high": "BoolStamped", "out_low": "BoolStamped"}
    sub_list = {"in_bool": "BoolStamped"}
    rosparam
    inherit Base, only modify compare function"""
class edgeDetector(BaseCalculator):
    _pub_topic_high = "/out_high"
    _pub_topic_low = "/out_low"
    _sub_topic = "/in_bool"
    def __init__(self,
        sub_dict = {_sub_topic: BoolStamped}, 
        pub_dict = {_pub_topic_high: BoolStamped,
                    _pub_topic_low: BoolStamped},
        rate = None):

        self.pre_state = None
        self.msg_high = BoolStamped()
        self.msg_low = BoolStamped()

        super(edgeDetector, self).__init__(
            sub_dict = sub_dict,
            pub_dict = pub_dict,
            rate = rate)

    def calculate(self):
        with BaseCalculator.LOCK[self._sub_topic]:
            current_val = BaseCalculator.MSG[self._sub_topic]

        if current_val != None:
            if current_val.data != self.pre_state:
                self.msg_high.header = current_val.header
                self.msg_low.header = current_val.header
                if current_val.data:
                    self.msg_low.data = False
                    self.PubDiag[self._pub_topic_low].publish(self.msg_low)
                    self.msg_high.data = True
                    self.PubDiag[self._pub_topic_high].publish(self.msg_high)
                    self.pre_state = True
                else:
                    self.msg_high.data = False
                    self.PubDiag[self._pub_topic_high].publish(self.msg_high)
                    self.msg_low.data = True
                    self.PubDiag[self._pub_topic_low].publish(self.msg_low)
                    self.pre_state = False
