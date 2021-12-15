#! usr/bin/python3
import rospy
from araig_msgs import BoolStamped

import numpy as np

def callback(data):
    if(data.data):
        rospy.sleep(3)
        if(np.random.random(size=1)[0]<0.5):
            

def init(topic):
    pub = rospy.Publisher(topic, BoolStamped, queue_size=10)
    sub = rospy.Subscriber('start_test', BoolStamped, callback)
    rospy.init_node('test_node', anonymous=True)
    

if __name__ == '__main__':
    try:
        rospy.init_node