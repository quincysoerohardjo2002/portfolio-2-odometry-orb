cmake_minimum_required(VERSION 3.0.2)
project(orb_slam_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/orb_slam_node.py
  src/pointcloud_visualizer_node.py
  src/sensor_fusion_node.py
  src/path_viewer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
