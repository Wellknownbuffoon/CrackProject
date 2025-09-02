cmake_minimum_required(VERSION 3.0.2)
project(dual_camera_dense_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  message_filters
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs tf message_filters cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(dual_camera_dense_fusion_node
  src/dual_camera_dense_fusion_node.cpp
)
target_link_libraries(dual_camera_dense_fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
