#!/usr/bin/env python

import rospy
import subprocess, shlex
from araig_msgs.msg import BoolStamped
from base_classes.base_logger import BaseLogger
from base_classes.base import create_logging_folder, get_sub_folder, get_root_folder, create_file
import threading
from datetime import datetime
import os

def get_folder_name(root):
    try: 
        os.listdir(root)
    except OSError as err:
        if err.errno == 2:
            os.makedirs(root) 
    
    size = len(os.listdir(root))
    size += 1
    folder_name = root + str(size)
    return folder_name

def create_folder(folder_name):
    try:
        os.mkdir(folder_name)
        rospy.loginfo(rospy.get_name() + ": Successfully created the directory " + folder_name)
    except OSError as err:
        rospy.logerr(rospy.get_name() + ": Failed to create " + folder_name + ", error msg: "+ str(err))

class FolderBagger(BaseLogger):
    def __init__(self):

        self.node_name = rospy.get_name()

        extend_subscribers_dict = {
            "test_failed"          : "/test_failed",
            "test_succeeded"       : "/test_succeeded",
        }

        param_list = [
            self.node_name + "/start_offset",
            self.node_name + "/stop_offset",
            self.node_name + "/blacklist",
            self.node_name + "/whitelist",
            "/test_type"]

        super(FolderBagger, self).__init__(sub_dict= extend_subscribers_dict, param_list = param_list)

        # Default whitelist topics that should always be recorded for every test
        self.whitelist = ["/signal/", "/data", "usb_cam", "/tf"]
        # Optional whitelist topics that are test specific
        self.whitelist = self.whitelist + self.config_param[self.node_name + "/whitelist"]

        try:
            while not rospy.is_shutdown():
                self.main_loop() 
        except rospy.ROSException:
            pass 

    def prepare_topics(self):
        list_of_topics = rospy.get_published_topics()
        list_of_topics.sort()
        topics_string = ""

        for topic,msg in list_of_topics:
            topic_allowed = False
            for white_string in self.whitelist:
                if white_string in topic:
                    topic_allowed = True
                    break
            for black_string in self.config_param[self.node_name + "/blacklist"]:
                if black_string in topic:
                    topic_allowed = False
                    break
            if topic_allowed:
                 topics_string = topics_string + topic + " "

        return topics_string

    def main_loop(self):
        rospy.loginfo(rospy.get_name() + ": Waiting for start signal...")
        # Wait for start signal
        start = False
        stop = False

        while not start and not rospy.is_shutdown():
            self._rate.sleep()
            start = self.getSafeFlag("start")

         # Create folder -> sleep -> start recording
        if start == True and stop == False:
            now = datetime.now()
            dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")

            root_folder = get_root_folder()
            
            rospy.loginfo(rospy.get_name() + ": Start received. Sleep {}s to prepare..."
                .format(self.config_param[self.node_name + "/start_offset"]))
            
            create_logging_folder(root_folder)
            
            rospy.sleep(self.config_param[self.node_name + "/start_offset"])

            current_folder = get_sub_folder()

            topics_string = self.prepare_topics()
            command = "rosbag record -o " + current_folder + "/" + self.config_param["/test_type"] + " " + topics_string

            self.startCommandProc(command)

        # Wait for stop signal
        while not stop and not rospy.is_shutdown():
            self._rate.sleep()
            stop = self.getSafeFlag("stop")
        
        # Sleep -> Kill recorder (other loggers finish their tasks) -> rename folder
        if start == True:
            if stop or rospy.is_shutdown():
                rospy.loginfo(rospy.get_name() + ": Stop received. Sleep {}s to settle..."
                    .format(self.config_param[self.node_name + "/stop_offset"]))
                rospy.sleep(self.config_param[self.node_name + "/stop_offset"])
                self.killCommandProc()
                rospy.sleep(0.5) # Sleep again to let process die properly
                if self.getSafeFlag("test_failed"):
                    create_file(current_folder, "failed_" + dt_string)
                    rospy.loginfo(rospy.get_name() + ": Test failed, create file " + "failed_" + dt_string)
                elif self.getSafeFlag("test_succeeded"):
                    create_file(current_folder, "succeeded_" + dt_string)
                    rospy.loginfo(rospy.get_name() + ": Test succeeded, create file " + "succeeded_" + dt_string)

        rospy.loginfo(rospy.get_name() + ": Waiting for trigger signals to reset")
        # Wait for stop and start to go low
        while (stop or start) and not rospy.is_shutdown():
                self._rate.sleep()
                stop = self.getSafeFlag("stop")
                start = self.getSafeFlag("start")
        rospy.loginfo(rospy.get_name() + ": Resetting.")
