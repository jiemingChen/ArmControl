cmake_minimum_required(VERSION 2.8.3)
project(panda_control)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH   "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  	roscpp
  	rospy
  	sensor_msgs
  	std_msgs
  	geometry_msgs
	msg_pkg
	kdl_parser
	trac_ik_lib
	nav_msgs
	trajectory_msgs
	visualization_msgs
	eigen_conversions

	octomap_msgs
	octomap_ros

  octomap_server
 geometric_shapes
		)

find_package(orocos_kdl REQUIRED)
find_package(octomap REQUIRED)
find_package(fcl REQUIRED )
find_package(OMPL   REQUIRED)
 find_package(Eigen3 REQUIRED)
find_package(CCD REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package( CATKIN_DEPENDS
	roscpp
	rospy
	sensor_msgs
	std_msgs
  geometry_msgs
)

include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${orocos_kdl_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
		${OCTOMAP_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}
		${OMPL_INCLUDE_DIRS}
		${fcl_INCLUDE_DIRS}
)



## Declare a C++ executable
#add_executable(${PROJECT_NAME}_INVDYN src/main.cpp src/Invdyn.cpp src/joint_impedancecontrol.cpp src/cartesian_impedance.cpp  src/common_func.cpp)

add_executable(${PROJECT_NAME}_PLAN
		src/plan_main.cpp   src/Panda.cpp
		src/motion_planner.cpp
		src/SEARCHER.cpp
		src/collision_detect/CollisionCheck.cpp
		src/collision_detect/fcl_utility.cpp
		include/visualize.h
		include/param.hpp
		)

add_dependencies(${PROJECT_NAME}_PLAN ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_PLAN
		${catkin_LIBRARIES}
		/home/jieming/catkin_ws/src/panda_simulation/panda_control/lib/libfranka_model.a
		${orocos_kdl_LIBRARIES}
		casadi
		${OCTOMAP_LIBRARIES}
		fcl
		${fcl_INCLUDE_DIRS}
		${OMPL_LIBRARIES}
		        ${CCD_LIBRARIES}
 		)



add_executable(${PROJECT_NAME}_INVDYN src/torque_main.cpp   src/Panda.cpp  src/cartesian_impedance.cpp)

add_dependencies(${PROJECT_NAME}_INVDYN ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_INVDYN
		${catkin_LIBRARIES}
		/home/jieming/catkin_ws/src/panda_simulation/panda_control/lib/libfranka_model.a
		${orocos_kdl_LIBRARIES}
)

add_executable(${PROJECT_NAME}_home src/reset_main.cpp src/Panda.cpp  src/motion_planner.cpp)

add_dependencies(${PROJECT_NAME}_home ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_home
		${catkin_LIBRARIES}
		/home/jieming/catkin_ws/src/panda_simulation/panda_control/lib/libfranka_model.a
		${orocos_kdl_LIBRARIES}
		casadi
		)


## Declare a C++ library
#add_library(franka_model_Coriolis SHARED
#		#src/model/CoriolisMatrix.cpp #src/model/FrictionTorque.cpp
#		#src/model/GravityVector.cpp  src/model/MassMatrix.cpp
#src/model/CoriolisMatrix.cpp
#)

#target_link_libraries(franka_model_Coriolis
#		casadi
#		)
#
#link_directories(${IPOPT_LIBRARY_DIRS}
#		)
