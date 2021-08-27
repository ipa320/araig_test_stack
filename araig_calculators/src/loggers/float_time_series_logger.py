#!/usr/bin/env python
import rospy
import yaml
import csv
from base_classes.base_time_series_logger import TimeSeriesLoggerClass
from std_msgs.msg import Float64

class FloatSeriesLoggerClass(TimeSeriesLoggerClass):
    def __init__(self):
        super(FloatSeriesLoggerClass, self).__init__()
        rospy.Subscriber("/in_float", Float64, self.data_subscriber)
        self.start_logger()

    def data_subscriber(self, msg):
        with self.lock_data:
            self.data_list = [rospy.Time.now().to_sec(), msg.data]
            self.data_available = True
