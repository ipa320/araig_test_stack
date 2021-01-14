#!/usr/bin/env python
import rospy
import threading
from araig_msgs.msg import BoolStamped
from geometry_msgs.msg import Twist

class CallbackList():
    def callback_start(self, msg, args):
        VelocityInterpreterClass.LOCK[args].acquire()
        VelocityInterpreterClass.DATA[args] = msg.data
        VelocityInterpreterClass.LOCK[args].release()

    def callback_stop(self, msg, args):
        VelocityInterpreterClass.LOCK[args].acquire()
        VelocityInterpreterClass.DATA[args] = msg.data
        VelocityInterpreterClass.LOCK[args].release()   

class VelocityInterpreterClass():
    LOCK = {}
    DATA = {}
    def __init__(self,
        pub_msg,
        loop_rate_hz = 1):

        self.pub_topic = "/velocity"
        self.sub_list = ["/start", "/stop"]

        self.pub_msg = pub_msg
        self.rate = rospy.Rate(loop_rate_hz)

        self.pub_init()
        self.sub_init()

        # init FLAG and LOCK
        for topic in self.sub_list:
            VelocityInterpreterClass.DATA[topic] = False
            VelocityInterpreterClass.LOCK[topic] = threading.Lock()
        
        self.main()
        
    def pub_init(self):
        self.pub_diag = rospy.Publisher(
                self.pub_topic, 
                Twist, 
                queue_size=10 
            )

    def sub_init(self):
        self.sub_diag = {}

        for topic in self.sub_list:
           
            call_back_name = topic.replace('/','_')
            
            callback = getattr(CallbackList(), 'callback'+ call_back_name)
            self.sub_diag[topic] = rospy.Subscriber(topic, BoolStamped, callback, (topic))
      
    def main(self):
        flag_both_high = False
        try:
            while not rospy.is_shutdown():
                temp = {}

                for topic in self.sub_list:
                    VelocityInterpreterClass.LOCK[topic].acquire()
                    temp[topic]= VelocityInterpreterClass.DATA[topic]
                    VelocityInterpreterClass.LOCK[topic].release()

                if temp["/start"] == 1 and temp["/stop"] == 0:
                    self.pub_diag.publish(self.pub_msg["/start"])
        
                if temp["/start"] == 0 and temp["/stop"] == 1:
                    self.pub_diag.publish(self.pub_msg["/stop"])

                if temp["/start"] == 1 and temp["/stop"] == 1 and flag_both_high == False:
                    rospy.logwarn(rospy.get_name() +": Both  START and STOP signals are high")
                    flag_both_high = True
                self.rate.sleep()   
        except rospy.ROSException:
            pass