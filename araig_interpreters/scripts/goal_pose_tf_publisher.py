#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf 
import tf.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math



rospy.init_node("goal_tf_caster")

pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)
msg = rospy.get_param("interpreters/goal_interpreter/goal")
print(msg)

def cb_caster(msg):
        t = TransformStamped()
        t.header.frame_id = "/map"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "/goal"
        t.transform.translation.x = msg['position']['x']
        t.transform.translation.y = msg['position']['y']
        t.transform.translation.z = msg['position']['z']

        t.transform.rotation = quaternion_from_euler(0,0,math.degrees(msg['yaw']), axes='sxyz')

        tfm = tf.msg.tfMessage([t])
        pub_tf.publish(tfm)

cb_caster(msg)
while not rospy.is_shutdown():
    pass
