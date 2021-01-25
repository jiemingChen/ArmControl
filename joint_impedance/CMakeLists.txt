cmake_minimum_required(VERSION 3.0.2)
project(joint_impedance)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

list(APPEND CMAKE_PREFIX_PATH /home/franka/Projects/libfranka/build)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs
  sensor_msgs
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joint_impedance
#  CATKIN_DEPENDS roscpp std_msgs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(${PROJECT_NAME}_node src/joint_impedance.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
    /home/franka/Projects/libfranka/build/examples/libexamples_common.a
    Franka::Franka 
)

