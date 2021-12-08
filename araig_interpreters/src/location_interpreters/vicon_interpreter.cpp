#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <ros/master.h>
#include <vector>
#include <map>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <algorithm>

class vicon_interpreter
{
private:
    boost::mutex topic_mutex_;
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    std::vector<std::string> viconTopics;
    std::vector<std::string> viconTopicsNew;
    ros::master::V_TopicInfo viconTopicInfos;

    tf::TransformListener tf_ls;
    tf::Transform trans_ls;

    geometry_msgs::PoseStamped pose_msg;

    std::map<std::string, ros::Publisher> viconPubList;
    std::map<std::string, ros::Subscriber> viconSubList;

    std::map<std::string, geometry_msgs::PoseStamped> OutputPoseMsg;

public:

    void stampedTrans2Pose(geometry_msgs::TransformStamped transMsg, 
        geometry_msgs::PoseStamped& poseMsg){
            
            poseMsg.pose.position.x = transMsg.transform.translation.x;
            poseMsg.pose.position.y = transMsg.transform.translation.y;
            poseMsg.pose.position.z = transMsg.transform.translation.z;

            poseMsg.pose.orientation.w = transMsg.transform.rotation.w;
            poseMsg.pose.orientation.x = transMsg.transform.rotation.x;
            poseMsg.pose.orientation.y = transMsg.transform.rotation.y;
            poseMsg.pose.orientation.z = transMsg.transform.rotation.z;

            poseMsg.header = transMsg.header;
    }

    void doTransform(geometry_msgs::PoseStamped& pose_msg, geometry_msgs::PoseStamped& pose_msg_tf)
    {
        ros::Time now = ros::Time(0);
        try{
            tf_ls.waitForTransform("/map", "/vicon/world", now, ros::Duration(1.0));
            tf_ls.transformPose("/map", pose_msg, pose_msg_tf);
        }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
    }

    void getViconTopic()
    {
        std::vector<std::string> lv_elems;
        for(ros::master::V_TopicInfo::iterator it = viconTopicInfos.begin();
            it != viconTopicInfos.end(); it++){
                boost::algorithm::split( lv_elems, it->name, boost::algorithm::is_any_of( "/" ) );
                //ROS_INFO_STREAM("examining topic "<<it->name);

                std::string topicName = "";
                //use substring
                for(uint i=0; i<lv_elems.size(); ++i){
                    if(i>1){
                        topicName += "/" + lv_elems[i];
                    }
                }
                
                // result of split is like /vicon/string: " ", "vicon", "string"   
                  if ( lv_elems[1] == "vicon" && it->datatype=="geometry_msgs/TransformStamped")
                {
                    if(std::find(viconTopics.begin(), viconTopics.end(), topicName) == viconTopics.end()
                        && std::find(viconTopicsNew.begin(), viconTopicsNew.end(), topicName) == viconTopicsNew.end()){
                        viconTopicsNew.push_back(topicName);
                        ROS_INFO_STREAM("get a topic of vicon "<<topicName);
                        ROS_INFO_STREAM("length of viconTopicsNew is "<<viconTopicsNew.size());
                    }
                }
            }
    }

    void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg, std::string topicName)
    {
        ROS_INFO_STREAM("sub of topic: "<<topicName<<" gets a msg");
        geometry_msgs::PoseStamped poseMsg;
        geometry_msgs::PoseStamped poseMsgTf;
        stampedTrans2Pose(*msg, poseMsg);
        ros::Time now = ros::Time(0);

        poseMsg.header.stamp = now;

        doTransform(poseMsg, poseMsgTf);
        viconPubList[topicName].publish(poseMsgTf);
    }

    void createThreadPubSub()
    {
        boost::thread PubSubThread(&vicon_interpreter::makePubSub, this);
        PubSubThread.join();
    }

    void makePubSub()
    {
        boost::mutex::scoped_lock lock(topic_mutex_);
        if(lock.owns_lock()){
            for(auto it=viconTopicsNew.begin();
            it != viconTopicsNew.end(); it++){
                std::string topic_name = (*it).data();
                ROS_INFO_STREAM("creating pub and sub on topic"<<topic_name);
                std::string pub_name = "/interpreter" + topic_name;
                std::string sub_name = "/vicon" + topic_name;

                ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(pub_name, 10);
                viconPubList.insert(std::make_pair(topic_name, pub));

                ros::Subscriber sub = nh.subscribe<geometry_msgs::TransformStamped>(sub_name, 10, boost::bind(&vicon_interpreter::viconCallback, this, _1, topic_name));
                viconSubList.insert(std::make_pair(topic_name, sub));
            }
            viconTopics.insert(viconTopics.end(), viconTopicsNew.begin(), viconTopicsNew.end());
            viconTopicsNew.clear();
            lock.unlock();
            ROS_INFO_STREAM("lock for PubSub thread cancelled");
        }
    }

    vicon_interpreter()
    {
        while(ros::ok()){
            boost::mutex::scoped_try_lock lock(topic_mutex_);
            if(lock.owns_lock()){
                if(ros::master::getTopics(viconTopicInfos) && viconTopicsNew.empty()){
                    getViconTopic();
                }else{
                    lock.unlock();
                    createThreadPubSub();
                }
            }
        }

    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_interpreter");
    ROS_INFO_STREAM("Interpreter node generated!");
    ros::AsyncSpinner aspin(1);
    aspin.start();
    vicon_interpreter interpreter;
    aspin.stop();
    //ros::spin();
    return 0;
}
