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
    Eigen::Matrix<double, 7, 1>  jointPos_;

    casadi::Opti opti_;
    Function solver_;
    DMDict arg_;

    int N_;
    SX X_;
    SX Q_;
    SX Q0_;
    DM position_desire_data_, x0_data_, orientation_desire_data_, Q0_data_;
    SX position_desired_;
    SX orientation_desired_;

    vector<double> max_limit;
    vector<double> min_limit;
public :
    MotionPlanner(ros::NodeHandle& );
    Eigen::Vector7d followTrajectoryOptim(Panda&);

    SX forwardKinematic(const SX& Q);
    SX oriError(const SX&,  const SX&  );
    SX quaternionError(const SX& , const SX&);
    SX oriEqual(const SX&,  const SX&  );

    Eigen::Vector7d iKSolv(const Eigen::Affine3d &, const Eigen::Vector7d& );
    Eigen::Vector7d copyOut(const DM& data);
    void copy(SX & mx, const vector<vector<SX>>& data);
    void receivX(const Eigen::Affine3d&, const Eigen::Vector7d& );

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
        for( int i = 0; i < 7; i++ )
            jointPos_(i) = msg->position[i];
    }
    void pubTrajectory(const Eigen::Vector7d&);
};


#endif //PANDA_CONTROL_MOTION_PLANNER_H
