//
// Created by jieming on 23.11.20.
//

#include "common.h"
#include "Panda.h"
#include "motion_planner.h"

vector<array<double, 3>> ee_data;
vector<array<double, 3>> ee_desired_data;
vector<array<double, 3>> ee_calculated_desired_data;
vector<pair<DM, double>> solver_info;




int main(int argc, char **argv) {
     ros::init(argc, argv, "home_node");
     ros::NodeHandle nh;

     ros::Rate rate(100);
     ros::Publisher trajPub_;
     trajPub_ =  nh.advertise<trajectory_msgs::JointTrajectory>("/robot1/jointTrajectory", 2);

     while (ros::ok()) {
         ros::spinOnce();
         rate.sleep();

         Eigen::Vector7d home_q;
         home_q << 0, -0.785, 0, -2.356, 0, 1.57, 0.785;

         trajectory_msgs::JointTrajectoryPoint j_p;
         trajectory_msgs::JointTrajectory j_traj;
         for(auto i=0; i<7; i++)
             j_p.positions.push_back(home_q(i));
         j_traj.points.push_back(j_p);

         trajPub_.publish(j_traj);

     }
 }




