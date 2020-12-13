//
// Created by jieming on 13.10.20.
//

#ifndef PANDA_CONTROL_NMPC_H
#define PANDA_CONTROL_NMPC_H

#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdlib.h>

#include <casadi/casadi.hpp>


#include "franka_model.h"


using namespace casadi;

class NMPC {
public:
    NMPC(int whichRobot);
    ~NMPC();

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

    void initVariables();

    void computeActions();

    void NMPCController();

    int dataReady();

    void setGoal(std::vector<double> desiredPos);

private:
    Eigen::Matrix<double, 7, 1>  jointPos, jointVel;
    // Desired robot's states, column vector of 7 elements
    Eigen::Matrix<double, 7, 1> jointPos_d;
    // Control actions,  column vector of 7 elements
    Eigen::Matrix<double, 7, 1> u;
    // integration step
    double h;
    // Support variable to control the flow of the script
    int dataReceived;
    // ROS related Variables, node handle
    ros::NodeHandle nh;
    ros::Publisher tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, tauPub6, tauPub7;
    ros::Subscriber sensorSub;

    // Support variables to contain the torques for the joints
    std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7, F;
    // Values for direct kinematics computation using DH parameters
    Eigen::Matrix<double, 7, 1> DH_a, DH_d, DH_alpha;
    Eigen::Matrix<double, 4, 4> DH_T, DH_A, T;
    Eigen::Matrix<double, 3, 1> eePosition;
};


#endif //PANDA_CONTROL_NMPC_H
