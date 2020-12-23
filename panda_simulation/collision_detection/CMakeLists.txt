cmake_minimum_required(VERSION 2.8.3)
project(collision_detection)
set(CMAKE_MODULE_PATH   "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )

 add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  octomap_msgs
  octomap_server
  roscpp
  tf
  visualization_msgs
        geometric_shapes
        trajectory_msgs
  msg_pkg
)
find_package(CCD REQUIRED)
find_package(fcl 0.6 REQUIRED)
#include_directories(${CCD_INCLUDE_DIRS})



catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES test
        #  CATKIN_DEPENDS fcl kdl_parser octomap_msgs octomap_server roscpp tf
        #  DEPENDS system_lib
)

 include_directories(
  include
  ${catkin_INCLUDE_DIRS}
         ${OCTOMAP_INCLUDE_DIRS}
         ${orocos_kdl_INCLUDE_DIRS}
         ${fcl_INCLUDE_DIRS}
)




add_executable(collision_distance_node
        src/main_distance.cpp
        src/CollisionCheck.cpp
        src/fcl_utility.cpp
)
target_link_libraries(collision_distance_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
    fcl
    ${CCD_LIBRARIES}
)


add_executable(${PROJECT_NAME}_node
        src/main.cpp
        src/CollisionCheck.cpp
        src/fcl_utility.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
    fcl
    ${CCD_LIBRARIES}
)

