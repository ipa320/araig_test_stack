#!/usr/bin/env python
from multipledispatch import dispatch as Override
import rospy
import math
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from base_classes.base_calculator import BaseCalculator

"""Compare data from two topics, output data_type: bool
    pub_dict =  {"/out_disp_angular": Float64, "/out_disp_position": Float64}
    sub_dict = {"/in_obj_1": "PoseStamped", "/in_obj_2", "PoseStamped"}
    inherence Comparator, only modify compare function"""
class diffPosesSpatial(BaseCalculator):
    _sub_topic_object_1 = "/in_obj_1"
    _sub_topic_object_2 = "/in_obj_2"
    _pub_topic_angular = "/out_disp_angular"
    _pub_topic_position = "/out_disp_position"
    def __init__(self,
            sub_dict = {_sub_topic_object_1: PoseStamped,
                        _sub_topic_object_2: PoseStamped}, 
            pub_dict = {_pub_topic_angular: Float64,
                        _pub_topic_position: Float64},
            rate = None):
        
            super(diffPosesSpatial, self).__init__(
                sub_dict = sub_dict,
                pub_dict = pub_dict,
                rate = rate)

    def get_yaw_from_quaternion(self, orientation):
        quaternion = [orientation.x,
                        orientation.y,
                        orientation.z,
                        orientation.w]
        rot = Rotation.from_quat(quaternion)
        euler = rot.as_euler('xyz', degrees = True)
        return euler[2]

    @Override()
    def calculate(self):
        self._pub_msg_position = self.PubDict[self._pub_topic_angular]
        self._pub_msg_angular = self.PubDict[self._pub_topic_angular]

        temp = {}
        for topic in self.SubDict.keys():
            with BaseCalculator.LOCK[topic]:
                temp[topic] = BaseCalculator.MSG[topic]
        
        flag_test_ready = True
        for value in temp.values():
            if value == None:
                flag_test_ready = False
        
        if flag_test_ready == True:
            delta_x = abs(temp[self._sub_topic_object_1].pose.position.x - temp[self._sub_topic_object_2].pose.position.x)
            delta_y = abs(temp[self._sub_topic_object_1].pose.position.y - temp[self._sub_topic_object_2].pose.position.y)

            self._pub_msg_angular = abs(self.get_yaw_from_quaternion(temp[self._sub_topic_object_1].pose.orientation) - \
                                self.get_yaw_from_quaternion(temp[self._sub_topic_object_2].pose.orientation) )
            self._pub_msg_position = math.sqrt((delta_x*delta_x) + (delta_y*delta_y))

            self.PubDiag[self._pub_topic_angular].publish(self._pub_msg_angular)
            self.PubDiag[self._pub_topic_position].publish(self._pub_msg_position)