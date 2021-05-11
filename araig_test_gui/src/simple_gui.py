#!/usr/bin/env python3.6
import rospy
from std_msgs.msg import Bool
from kivymd.app import MDApp
from kivy.clock import Clock
from kivy.uix.image import Image
from kivy.lang import Builder
from kivy.uix.widget import Widget
import threading
from kivy.properties import ListProperty, OptionProperty, BooleanProperty, NumericProperty, StringProperty
from araig_msgs.msg import BoolStamped, Float64Stamped

class Led(Image):
    """Led widget for kivy"""
    # state of led
    state = OptionProperty("off", options=["on", "off"])
    # led_type : what does change when state changes (color, source or both)
    led_type = OptionProperty('color',options=['color','source','both'])
    # color_on : color when state is on
    color_on = ListProperty([0.22,0.79,1,1])
    # color_off : color when state is off
    color_off = ListProperty([0.05,0.175,0.225,1])
    # source_on : image source when state is on
    source_on = StringProperty('')
    # source_off : image source when state is off
    source_off = StringProperty('')
    # auto_off : switch auto to off state when True
    auto_off=BooleanProperty(False)
    # auto_off_delay : delay before switching off
    auto_off_delay=NumericProperty(0.2)
    
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        # force setting according to off state
        self.on_state(None,'off')
        
    def toggle_state(self):
        """toggle_state : change state"""
        if self.state == 'on':
            self.state = 'off'
        else:
            self.state = 'on'
    
    def on_state(self,instance,state):
        if state=='on':
            if self.led_type == 'color' or self.led_type == 'both':
                self.color=self.color_on
            if self.led_type == 'source' or self.led_type == 'both':
                self.source=self.source_on
            if self.auto_off:
                Clock.schedule_once(lambda dt: self.set_off(),self.auto_off_delay)
        else: # state==off
            if self.led_type == 'source' or self.led_type == 'both':
                self.source=self.source_off
            if self.led_type == 'color' or self.led_type == 'both':
                self.color=self.color_off
            
    def set_off(self):
        self.state='off'
    
    def on_source_on(self,instance,src):
        if self.state == 'on' and (self.led_type == 'source' or self.led_type == 'both'):
            self.source=self.source_on
            
    def on_source_off(self,instance,src):
        if self.state == 'off' and (self.led_type == 'source' or self.led_type == 'both'):
            self.source=self.source_off
            
    def on_color_on(self,instance,clr):
        if self.state == 'on' and (self.led_type == 'color' or self.led_type == 'both'):
            self.color=self.color_on
            
    def on_color_off(self,instance,clr):
        if self.state == 'off' and (self.led_type == 'color' or self.led_type == 'both'):
            self.color=self.color_off
            
    def on_led_type(self, instance, tp):
        if tp == 'source':
            # set color to white when led_type = 'source'
            self.color = [1,1,1,1]

# class Spinner(Widget):

#    def __init__(self, **kwargs):
#        super().__init__(**kwargs)
#        # force setting according to off state
#        self.on_state(None,'off')

#    def spinner_clicked(self)    


class App(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.screen=Builder.load_file('ros_gui.kv')
        self.mutex_lock_float_1 = threading.Lock()
        self.mutex_lock_float_2 = threading.Lock()
        rospy.Subscriber('/float_1', Float64Stamped,  self.callback_float_1)
        rospy.Subscriber('/float_2', Float64Stamped, self.callback_float_2)
        self.float_1 = 0
        self.float_2 = 1
        print('called constructor')

        """rospy.Subscriber('/signal/runner/test_completed', self.callback_test_completed)
        self.mutex_lock = threading.lock()
        self.completed

        rospy.Subscriber('/signal/runner/test_succeeded', self.callback_test_succeeded)
        self.mutex_lock_2 = threading.lock()
        self.succeeded  """ 
    def callback_float_1(self, msg):
        print('asdf')
        with self.mutex_lock_float_1():
            print("Got data: {}".format(msg.data))
            self.root.ids['result1_v_label'].text = msg.data

    def callback_float_2(self, msg):
        with self.mutex_lock_float_2():
            self.root.ids['result2_v_label'].text = msg.data

    def callback_test_completed(self, msg):
        with self.mutex_lock():
            self.completed = msg.data

    def callback_test_failed(self, msg):
        with self.mutex_lock():
            self.failed = msg.data

    def callback_test_succeeded(self, msg):
        with self.mutex_lock_2():
            self.succeeded = msg.data

    def update_led(self):
        with self.mutex_lock_2():
            succ = self.succeeded

        with self.mutex_lock_3():
            fail = self.failed

        if succ:
            print('succ')
            #my_led.color = green 
        elif fail:
            print('fail')
            #my_led.color = red 
        else:
            print('nothing changed')
            #my_led.color = gray

    def build(self):
        return self.screen
    
    def toggle_led(self):
        for k in self.root.ids:
            if "led_" in k:
                self.root.ids[k].toggle_state()
    
    def press_start(self,*args):


        msg=BoolStamped() #alte
        msg.header.stamp = rospy.Time.now() 
        msg.data = True
        pub1.publish(msg) 
        if self.testsetup == 'test1_breaking':
            print(self.testsetup +" has been started")
            #sub

        elif self.testsetup == 'test1_emergency':
            print(self.testsetup +" has been started")
            #

        elif self.testsetup == 'test4':
            print(self.testsetup +" has been started")
            #

        elif self.testsetup == 'test5_with_nav':
            print(self.testsetup +" has been started")

        else:
            print(self.testsetup +" has been started")

        
        
        self.root.ids['led_animated'].toggle_state()

    def press_interrupt(self,*args):
        print('Interrupt button pressed')
        msg=True
        pub2.publish(msg)
        self.root.ids['led_animated'].set_off()
        self.root.ids['led_typeboth1'].toggle_state()
    
    
    def press_reset(self,*args):
        print('Reset button pressed')
        msg=True
        pub3.publish(msg)
        self.root.ids['led_animated'].set_off()
        self.root.ids['led_typeboth1'].toggle_state()


    def spinner_clicked(self,value):
        self.testsetup=value
        #"test1_breaking", "test1_emergency","test4", "test5_with_nav", "test5_without_nav"]
        if value == 'test1_breaking' or value == 'test1_emergency':
            print('You chose '+value)
            self.root.ids['result1_label'].text='breaking time: '
            self.root.ids['result2_label'].text='breaking distance: '
            self.root.ids['result1_v_label'].text='0'
            self.root.ids['result2_v_label'].text='0'
        #elif value == 'test1_emergency':
        #    print('You chose '+value)
        #    self.root.ids['result1_label'].text='breaking time: '
        #    self.root.ids['result2_label'].text='breaking distance: '
        #    self.root.ids['result1_v_label'].text='0'
        #    self.root.ids['result2_v_label'].text='0'
        elif value == 'test4' or value == 'test5_with_nav':
            self.root.ids['result1_label'].text='goal time: '
            self.root.ids['result2_label'].text=' '
            self.root.ids['result1_v_label'].text='0'
            self.root.ids['result2_v_label'].text=' '
        else:
            self.root.ids['result1_label'].text='Robot stop gap: '
            self.root.ids['result2_label'].text=' '
            self.root.ids['result1_v_label'].text='0'
            self.root.ids['result2_v_label'].text=' '


        # subscribe to the value
        #case 2
        #self.root.ids['result1_label'].text='breaking time'
        #self.root.ids['result2_label'].text='breaking distance'
        #case 3

        #case 4



       # self.root.ids['spinner_id']
       # self.root.ids['spinner_clicked']


if __name__ == '__main__':
    print('Hallo Tejas')
    pub1=rospy.Publisher('/button1',BoolStamped,queue_size=1)
    pub2=rospy.Publisher('/button2',BoolStamped,queue_size=1)
    pub3=rospy.Publisher('/button3',BoolStamped,queue_size=1)
    rospy.init_node('simple_gui',anonymous=True)
    spin_thread = threading.Thread(target=lambda: rospy.spin())
    spin_thread.start()
    #my_app = App()
    App().run()

    