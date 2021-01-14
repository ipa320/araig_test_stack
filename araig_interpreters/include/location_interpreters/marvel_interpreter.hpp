#ifndef MARVEL_INTERPRETER_HPP
#define MARVEL_INTERPRETER_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "geometry_msgs/PoseStamped.h"
#include <algorithm>
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class MarvelInterpreter
{
public:
  MarvelInterpreter();
  void hedgeCallback(const marvelmind_nav::hedge_pos_aConstPtr& msg, ros::NodeHandle nh);
  void checkMap();

  ros::Subscriber sub_beacon_pos_;

private:

  std::map <int, ros::Publisher> address_publisher_;
  tf::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

#endif // MARVEL_INTERPRETER_HPP
