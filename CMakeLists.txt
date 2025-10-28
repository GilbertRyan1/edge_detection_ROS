cmake_minimum_required(VERSION 3.0.2)
project(edge_detector_pkg2)

# basic packages i needed
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

# service for edge detection
add_service_files(
  FILES
  DetectEdges.srv
)

# generate stuff for the service
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# tell catkin what we use
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# install python files so rosrun works later
catkin_install_python(
  PROGRAMS
    scripts/run_edge_detection.py
    scripts/edge_service_node.py
    scripts/edge_service_client.py
    scripts/rgb_edge_node.py
    scripts/edge_points_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# nothing fancy below, just leaving the default test stuff commented
# catkin_add_gtest(${PROJECT_NAME}-test test/test_edge_detector_pkg2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
