//
// Created by jieming on 04.11.20.
//
#ifndef PANDA_CONTROL_PANDA_H
#define PANDA_CONTROL_PANDA_H
#include "common.h"

#include <iostream>
#include <array>
#include <Eigen/Dense>

#include <trac_ik/trac_ik.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/jntspaceinertiamatrix.hpp>

#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <eigen_conversions/eigen_kdl.h>

using namespace std;
using namespace KDL;

class Panda {
private:
    string urdf_file_;
    string base_link_, tip_link_;

    KDL::Tree tree_;
    KDL::Chain chain_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray  q0_;

    KDL::JntArray  tau_;
    KDL::JntArray  q_min_;
    KDL::JntArray  q_max_;

    unsigned int num_joints_;
    JntArray gravity_torque_, coriolis_torque_;
    KDL::JntSpaceInertiaMatrix D_;

    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;
    boost::scoped_ptr<KDL::ChainDynParam> jnt_to_dyn_;
    boost::scoped_ptr<TRAC_IK::TRAC_IK> ik_slover_;

    KDL::Frame     xd_;           // Tip desired pose
    KDL::Frame     x0_;
    KDL::Frame     x_;            // Tip pose

    KDL::Twist     xerr_;         // Cart error
    KDL::Twist     xdot_;         // Cart velocity
    KDL::Wrench    F_;            // Cart effort
    KDL::Jacobian  J_;            // Jacobian

    KDL::Twist     Kp_;           // Proportional gains
    KDL::Twist     Kd_;           // Derivative gains

public:
    Panda();
    void setJoints(const sensor_msgs::JointState::ConstPtr& msg);
    void setJoints(const Eigen::Matrix<double, 7, 1>& , const Eigen::Matrix<double, 7, 1>&);

    Eigen::Affine3d fkEE();
    Eigen::Vector7d iK(Eigen::Affine3d);

    void dyn();
    Eigen::VectorXd gravityTorque();
    Eigen::VectorXd coriolis();
    Eigen::MatrixXd jacobian();
    Eigen::MatrixXd mass();
};


#endif //PANDA_CONTROL_PANDA_H
