//
// Created by jieming on 08.11.20.
//

#ifndef PANDA_CONTROL_MOTION_PLANNER_H
#define PANDA_CONTROL_MOTION_PLANNER_H

#include "common.h"
#include "Panda.h"
#include <casadi/casadi.hpp>
using namespace casadi;

class MotionPlanner {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sensorSub_;
    ros::Publisher trajPub_;
    Eigen::Matrix<double, 7, 1> jointPos_;

    casadi::Opti opti_;
    Function solver_;
    DMDict arg_;

    int N_;
    SX X_;
    SX Q_;
    SX Qdot_;
    SX Q0_;
    DM position_desire_data_, x0_data_, orientation_desire_data_, Q0_data_, joint_position_desired_data_;
    SX position_desired_;
    SX orientation_desired_;
    SX feedback_variable_;
    SX joint_position_desired_;


    bool first_solve_;
    bool first_receive_;
    vector<double> last_x_;
    vector<double> max_limit;
    vector<double> min_limit;
    vector<double> dq_min_limit, dq_max_limit;
    vector<double> avg_limit_, delta_limit_;


    Eigen::Vector7d goal_joint_;
    Eigen::Matrix3d ori_desired_;
    Eigen::Vector3d goal_position_;

    vector<array<double, 3>> obs_;

public :
    MotionPlanner(ros::NodeHandle &);

    void buildIKModel();
    void buildModel();
    void buildModel2();


    Eigen::Vector7d iKSolv(const Eigen::Affine3d &, const Eigen::Vector7d &);
    Eigen::Vector7d MPCSolv(const Eigen::Affine3d &, const Eigen::Vector7d &);
    pair<bool, vector<Eigen::Vector7d>> MPCSolv(const Eigen::Vector7d &);


    Eigen::Vector7d followTrajectoryOptim(Panda &);
    Eigen::Vector7d generateTrajectory(Panda & );
    pair<bool, vector<Eigen::Vector7d>> generateJointTrajectory(Panda &);


    SX forwardKinematic(const SX&);
    vector<pair<SX, double>> forwardKinematicMultiLinks(const SX&);


    SX oriError(const SX &, const SX &);
    SX oriError2(const SX &, const SX &);
    SX oriError3(const SX &, const SX &);
    SX eluerError(const SX &, const SX &);
    SX quaternionError(const SX &, const SX &);


    void copy(SX &mx, const vector<vector<SX>> &data);

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
        if (!first_receive_)
            first_receive_ = true;
        for (int i = 0; i < 7; i++)
            jointPos_(i) = msg->position[i];
    }
    bool receiveFeedback() {
        return first_receive_ == true;
    }

    void setGoal(Panda&);
    void setObstacles(const vector<array<double,3>>&);
    void pubTrajectory(const Eigen::Vector7d &);

};

#endif //PANDA_CONTROL_MOTION_PLANNER_H

#if 0
void receivX(const Eigen::Affine3d &, const Eigen::Vector7d &);
Eigen::Vector7d copyOut(const DM &data);
void initial(const Eigen::Affine3d &, const Eigen::Vector7d &);
Eigen::Vector7d copyOut2(const DM &data);
#endif