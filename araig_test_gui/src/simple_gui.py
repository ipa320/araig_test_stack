#!/usr/bin/env python3.6
import rospy
from std_msgs.msg import Bool
from kivymd.app import MDApp
from kivy.lang import Builder

class TutorialApp(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.screen=Builder.load_file('/home/aulon/gui_ws/src/gui/src/ros_gui.kv')

    def build(self):
        return self.screen
    
    def my_function(self,*args):
        print("button pressed")

        self.screen.ids.my_label.text='button pressed'

        msg=True
        pub.publish(msg)



if __name__ == '__main__':

    pub=rospy.Publisher('/button',Bool,queue_size=1)
    rospy.init_node('simple_gui',anonymous=True)
    TutorialApp().run()
    
