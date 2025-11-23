cmake_minimum_required(VERSION 3.0.2)
project(grid_auto)  # Must match package.xml

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and system dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
   interactive_markers
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs interactive_markers
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Main executable (runs everything)
add_executable(grid_system
  src/main.cpp
  src/robot_allign.cpp
  src/grid_overlay.cpp
)

target_link_libraries(grid_system ${catkin_LIBRARIES})
add_dependencies(grid_system ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

