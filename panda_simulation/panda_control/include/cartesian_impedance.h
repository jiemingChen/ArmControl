//
// Created by jieming on 20.10.20.
//

#ifndef PANDA_CONTROL_CARTESIAN_IMPEDANCE_H
#define PANDA_CONTROL_CARTESIAN_IMPEDANCE_H

#include "common.h"
#include "Panda.h"
 #include "msg_pkg/TransformationError.h"

using namespace std;
using Eigen::MatrixXd;


class Controller {
private:
    double translational_stiffness{200}; //800
    double rotational_stiffness{10};   //20
    MatrixXd stiffness;
    MatrixXd damping;

    Eigen::Matrix<double, 7, 1>  jointPos, jointVel;
    Eigen::Vector7d q_init_;
    vector<double>  gripperPos , gripperVel ;


    Eigen::Vector7d q_desired_, qdot_desired;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Matrix<double, 7, 1> u;      // Control actions,  column vector of 7 elements
//    double h{}{};
    int dataReceived;

    ros::NodeHandle nh;
    ros::Publisher tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, tauPub6, tauPub7, orientationErrorPub, positionErrorPub;
    ros::Publisher posPubLeft, posPubRight;
    ros::Subscriber sensorSub, trjSub_;

    std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7, F;
    std_msgs::Float64 pos_left_gripper, pos_right_gripper;
    std_msgs::Float64 error_ori;
    msg_pkg::TransformationError error_pos;

    Eigen::Affine3d cur_transform_;
    Eigen::Matrix<double, 6, 1> error_;
    Eigen::Vector7d compensation_;

    double time_main_;


public:
    Controller();
//    void cartesian_control(FRANKA &);
    void cartesianPDControl(Panda &);
    void jointPDControl(Panda &);


    void poseError(Panda&);

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void trajCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg);
    void pubError();
    void nullSpaceControl(Panda&);
    void setGoal(Panda &);  //vector<double> desiredPos

    void followTrajectory(Panda&);
    void followTrajectoryIK(Panda&);

    void computeActions();
    bool dataReady();
};


#endif //PANDA_CONTROL_CARTESIAN_IMPEDANCE_H
