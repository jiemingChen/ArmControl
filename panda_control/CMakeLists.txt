cmake_minimum_required(VERSION 2.8.3)
project(panda_control)

add_compile_options(-std=c++11)


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

	eigen_conversions
)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
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
)
include_directories(${EIGEN3_INCLUDE_DIR})



## Declare a C++ executable
#add_executable(${PROJECT_NAME}_INVDYN src/main.cpp src/Invdyn.cpp src/joint_impedancecontrol.cpp src/cartesian_impedance.cpp  src/common_func.cpp)

add_executable(${PROJECT_NAME}_PLAN src/plan_main.cpp   src/Panda.cpp   src/motion_planner.cpp include/visualize.h
		)

add_dependencies(${PROJECT_NAME}_PLAN ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_PLAN
		${catkin_LIBRARIES}
		/home/jieming/catkin_ws/src/panda_simulation/panda_control/lib/libfranka_model.a
		${orocos_kdl_LIBRARIES}
		casadi
		)




add_executable(${PROJECT_NAME}_INVDYN src/torque_main.cpp   src/Panda.cpp  src/cartesian_impedance.cpp)

add_dependencies(${PROJECT_NAME}_INVDYN ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_INVDYN
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
