cmake_minimum_required(VERSION 3.0.2)
project(araig_test_runners)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES araig_test_runners
   CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
   scripts/test_1_braking
   scripts/test_5_obstacle_avoidance
   scripts/test_5_without_goal
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
