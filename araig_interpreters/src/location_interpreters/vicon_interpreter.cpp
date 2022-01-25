#include <ros/master.h>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <algorithm>
#include "location_interpreters/vicon_interpreter.hpp"

ViconInterpreter::ViconInterpreter(ros::NodeHandle &nh)
{
    while (ros::ok())
    {
        boost::mutex::scoped_try_lock lock(topic_mutex_);
        if (lock.owns_lock())
        {
            if (ros::master::getTopics(vicon_topic_infos_) && vicon_topics_new_.empty())
            {
                getViconTopic();
            }
            else
            {
                lock.unlock();
                createThreadPubSub(nh);
            }
        }
    }
}

bool ViconInterpreter::checkTopicExist(const std::string& topic_name, const std::vector<std::string>& topic_vector)
{
    return std::find(topic_vector.begin(), topic_vector.end(), topic_name)!=topic_vector.end();
}

void ViconInterpreter::getViconTopic()
{
    std::vector<std::string> lv_elems;
    for (ros::master::V_TopicInfo::iterator it = vicon_topic_infos_.begin();
         it != vicon_topic_infos_.end(); it++)
    {
        if(it->name.find("/vicon")==0 && it->datatype == "geometry_msgs/TransformStamped")
        {
            std::string topic_name = it->name.substr(6);
            if (!checkTopicExist(topic_name, vicon_topics_) && !checkTopicExist(topic_name, vicon_topics_new_))
            {
                vicon_topics_new_.push_back(topic_name);
            }
        }
    }
}

void ViconInterpreter::viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg, std::string topic_name)
{
    vicon_pub_list_[topic_name].publish(doTransform(stampedTransToPose(*msg)));
}

geometry_msgs::PoseStamped ViconInterpreter::stampedTransToPose(const geometry_msgs::TransformStamped& trans_msg)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.pose.position.x = trans_msg.transform.translation.x;
    pose_msg.pose.position.y = trans_msg.transform.translation.y;
    pose_msg.pose.position.z = trans_msg.transform.translation.z;

    pose_msg.pose.orientation.w = trans_msg.transform.rotation.w;
    pose_msg.pose.orientation.x = trans_msg.transform.rotation.x;
    pose_msg.pose.orientation.y = trans_msg.transform.rotation.y;
    pose_msg.pose.orientation.z = trans_msg.transform.rotation.z;

    pose_msg.header = trans_msg.header;
    pose_msg.header.stamp = ros::Time(0);

    return pose_msg;
}

geometry_msgs::PoseStamped ViconInterpreter::doTransform(const geometry_msgs::PoseStamped &pose_msg)
{
    geometry_msgs::PoseStamped pose_msg_tf;
    try
    {
        tf_ls_.waitForTransform("/map", "/vicon/world", ros::Time(0), ros::Duration(1.0));
        tf_ls_.transformPose("/map", pose_msg, pose_msg_tf);
        return pose_msg_tf;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void ViconInterpreter::createThreadPubSub(ros::NodeHandle& nh)
{
    boost::thread PubSubThread(&ViconInterpreter::makePubSub, this, nh);
    PubSubThread.join();
}

void ViconInterpreter::makePubSub(ros::NodeHandle& nh)
{
    boost::mutex::scoped_lock lock(topic_mutex_);
    if (lock.owns_lock())
    {
        for (auto it = vicon_topics_new_.begin();
             it != vicon_topics_new_.end(); it++)
        {
            std::string topic_name = (*it).data();
            ROS_INFO_STREAM("[Interpreter] creating pub and sub on topic" << topic_name);
            std::string pub_name = "/interpreter" + topic_name;
            std::string sub_name = "/vicon" + topic_name;

            ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(pub_name, 10);
            vicon_pub_list_.insert(std::make_pair(topic_name, pub));

            ros::Subscriber sub = nh.subscribe<geometry_msgs::TransformStamped>(sub_name, 10, boost::bind(&ViconInterpreter::viconCallback, this, _1, topic_name));
            vicon_sub_list_.insert(std::make_pair(topic_name, sub));
        }
        vicon_topics_.insert(vicon_topics_.end(), vicon_topics_new_.begin(), vicon_topics_new_.end());
        vicon_topics_new_.clear();
        lock.unlock();
    }
}
