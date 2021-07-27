cmake_minimum_required(VERSION 3.0.2)
project(araig_calculators)

find_package(catkin REQUIRED COMPONENTS
  rospy
  araig_msgs
)

catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS rospy araig_msgs
)

catkin_install_python(PROGRAMS
   scripts/diff_pose_temporal_node
   scripts/diff_time_node
   scripts/comp_param_node
   scripts/comp_poses_node
   scripts/comp_topics_node
   scripts/folder_bagger_node
   scripts/float_series_logger_node
   scripts/pose_series_logger_node
   scripts/edge_detector_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_rostest(tests/node_tests/test_edge_detector_node.test)
  # TODO:Fix tests and add them back later. Refer issue 37
  # add_rostest(tests/node_tests/test_diff_pose_temporal_node.test)
  add_rostest(tests/node_tests/test_diff_time_node.test)
  # TODO:Fix tests and add them back later. Refer issue 37
  # add_rostest(tests/node_tests/test_diff_poses_spatial_node.test)
  add_rostest(tests/node_tests/test_comp_topics_node.test)
  # TODO:Fix tests and add them back later. Refer issue 37
  # add_rostest(tests/node_tests/test_comp_poses_node.test)
  add_rostest(tests/node_tests/test_comp_param_node.test)
endif()