#!/usr/bin/env python
import rospy
import yaml
from base_classes.base_logger import BaseLogger
from base_classes.base import get_sub_folder, check_folder
from araig_msgs.msg import Float64Stamped
import threading


class ResultsLoggerClass(BaseLogger):
    def __init__(self):

        self.node_name = rospy.get_name()
        self.namespace_list = ["calculators"]

        param_list = [
            self.node_name + "/start_offset",
            self.node_name + "/stop_offset",
            self.node_name + "/logginng_topics",
        ]

        self.config_param = {}
        self.getConfig(param_list)
        self.result_topics_list = self.config_param[self.node_name +
                                                    "/logginng_topics"]

        extend_subscribers_dict = {
            "test_failed": "/test_failed",
            "test_succeeded": "/test_succeeded",
        }

        super(ResultsLoggerClass, self).__init__(
            param_list=param_list,
            result_list=self.result_topics_list,
            sub_dict=extend_subscribers_dict)

        try:
            while not rospy.is_shutdown():
                self.main_loop()
        except rospy.ROSException:
            pass

    def main_loop(self):
        rospy.loginfo(rospy.get_name() + ": Waiting for start signal...")

        start = False
        stop = False
        test_succeeded = False
        while not start and not rospy.is_shutdown():
            self._rate.sleep()
            start = self.getSafeFlag("start")

        rospy.loginfo(
            rospy.get_name() +
            ": Start recevied. Waiting for test completion...")
        while not stop and not rospy.is_shutdown():
            self._rate.sleep()
            stop = self.getSafeFlag("stop")

        test_succeeded = self.getSafeFlag("test_succeeded")

        if start and stop:
            if test_succeeded:
                # Wait for folder to be created before starting
                rospy.loginfo(rospy.get_name() + ": Start received. Sleep {}s to prepare..."
                              .format(self.config_param[self.node_name + "/start_offset"]))
                rospy.sleep(
                    self.config_param[self.node_name + "/start_offset"])
                result = {}
                for topic in self.result_topics_list:
                    result[topic] = self.getSafeFlag(topic)

                folder = get_sub_folder()
                if check_folder(folder):
                    filename = folder + "/result.yaml"
                    rospy.loginfo(
                        rospy.get_name() +
                        ": Test succeeded, writing results into {}..." .format(filename))
                    with open(filename, 'w+') as yaml_file:
                        yaml.dump(result, yaml_file, default_flow_style=False)

                rospy.sleep(self.config_param[self.node_name + "/stop_offset"])
                rospy.loginfo(
                    rospy.get_name() +
                    ": Finished writing into file")

            else:
                rospy.loginfo(rospy.get_name() +
                              ": Test did not succeed, no results written")

        # Wait for stop and start to go low
        while (stop or start) and not rospy.is_shutdown():
            self._rate.sleep()
            stop = self.getSafeFlag("stop")
            start = self.getSafeFlag("start")
        rospy.loginfo(rospy.get_name() + ": Resetting.")
