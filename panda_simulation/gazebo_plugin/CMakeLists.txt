cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  roscpp 
  std_msgs 
  std_srvs 
  geometry_msgs 
  nav_msgs tf 
  dynamic_reconfigure 
  message_generation
  octomap_msgs
  octomap_ros
msg_pkg
)

include_directories(include ${catkin_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

 
 

include_directories(/user/include/sdformat-5.2)
include_directories(/usr/include/gazebo-9/gazebo/)
include_directories(./include)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(BuildOctomapPlugin SHARED src/plugin_build_octomap.cpp)
target_link_libraries(BuildOctomapPlugin ${catkin_LIBRARIES} )
 

#############
## Install ##
#############
set_target_properties(BuildOctomapPlugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY /home/jieming/catkin_ws/devel/lib)
 
