#include "location_interpreters/vicon_interpreter.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_interpreter");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Interpreter node generated!");
    ros::AsyncSpinner aspin(1);
    aspin.start();
    ViconInterpreter interpreter(nh);
    aspin.stop();
    return 0;
}
