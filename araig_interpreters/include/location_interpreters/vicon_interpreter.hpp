#ifndef VICON_INTERPRETER_HPP
#define VICON_INTERPRETER_HPP

#include <ros/ros.h>
#include <map>
#include <vector>
#include <string>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/thread.hpp>

class ViconInterpreter
{
private:
    boost::mutex topic_mutex_;

    std::vector<std::string> vicon_topics_;
    std::vector<std::string> vicon_topics_new_;
    ros::master::V_TopicInfo vicon_topic_infos_;

    tf::TransformListener tf_ls_;

    std::map<std::string, ros::Publisher> vicon_pub_list_;
    std::map<std::string, ros::Subscriber> vicon_sub_list_;

    std::map<std::string, geometry_msgs::PoseStamped> output_pose_msg_;
public:
    ViconInterpreter(ros::NodeHandle &nh);
    geometry_msgs::PoseStamped stampedTransToPose(const geometry_msgs::TransformStamped& trans_msg);
    geometry_msgs::PoseStamped doTransform(const geometry_msgs::PoseStamped& pose_msg);
    bool checkTopicExist(const std::string& topic_name, const std::vector<std::string>& topic_vector);
    void getViconTopic();
    void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg, std::string topic_name);
    void createThreadPubSub(ros::NodeHandle &nh);
    void makePubSub(ros::NodeHandle& nh);
};

#endif // VICON_INTERPRETER_HPP
