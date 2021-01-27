#!/usr/bin/env python
from multipledispatch import dispatch as Override
import rospy

from araig_msgs.msg import BoolStamped, Float64Stamped
from base_classes.base_calculator import BaseCalculator

'''Compare data from two topics, output data_type: Float64Stamped
    pub_dict =  {"/out_duration": Float64}
    sub_dict = {"/in_start": "BoolStamped", "/in_stop", "BoolStamped"}
    inherence Comparator, only modify calculate function '''
class diffTime(BaseCalculator):
    _sub_topic_start = "/in_start"
    _sub_topic_stop = "/in_stop"
    _pub_topic = "/out_duration"
    def __init__(self,
        sub_dict = {_sub_topic_start: BoolStamped,
                    _sub_topic_stop: BoolStamped}, 
        pub_dict = {_pub_topic: Float64Stamped},
        rate = None):

            self._prestate_start = False
            self._prestate_stop = False

            self._timestamp_start = None
            self._timestamp_stop = None

            super(diffTime, self).__init__(
                sub_dict = sub_dict,
                pub_dict = pub_dict,
                rate = rate)

    @Override()
    def calculate(self):
        temp = {}

        with BaseCalculator.LOCK[self._sub_topic_start]:
            temp[self._sub_topic_start] = BaseCalculator.MSG[self._sub_topic_start]
        with BaseCalculator.LOCK[self._sub_topic_stop]:    
            temp[self._sub_topic_stop] = BaseCalculator.MSG[self._sub_topic_stop]

        if self._prestate_start == False:
            if temp[self._sub_topic_stop] != None and temp[self._sub_topic_stop].data == True:
                if self._prestate_stop == False:
                    rospy.logwarn("{}: get {} before {}".format(rospy.get_name(), self._sub_topic_stop, self._sub_topic_start))
                    self._prestate_stop = True
        
        if temp[self._sub_topic_stop] != None and \
            self._prestate_stop ==  True and \
            temp[self._sub_topic_stop].data == False:
            self._prestate_stop = False

        if temp[self._sub_topic_start] != None: 
            if self._prestate_start == False and \
                temp[self._sub_topic_start].data == True:
                    self._timestamp_start = temp[self._sub_topic_start].header.stamp
                    rospy.loginfo("{}: Started".format(rospy.get_name()))
            
            self._prestate_start = temp[self._sub_topic_start].data

        if self._prestate_start == True:
            if temp[self._sub_topic_stop] != None:
                if self._prestate_stop == False and \
                    temp[self._sub_topic_stop].data == True:
                
                    self._timestamp_stop = temp[self._sub_topic_stop].header.stamp
                    rospy.loginfo("{}: Stopped".format(rospy.get_name()))
            
                    stopwatch = self._timestamp_stop - self._timestamp_start
                    pub_msg = self.PubDict[self._pub_topic]()
                    duration = float(stopwatch.secs + float(stopwatch.nsecs*(1e-9)))
                    pub_msg.data = duration
                    pub_msg.header.stamp = rospy.Time.now()

                    self.PubDiag[self._pub_topic].publish(pub_msg)
                    rospy.loginfo("{}: Duration: {}".format(rospy.get_name(), duration))

                self._prestate_stop = temp[self._sub_topic_stop].data