//
// Created by jieming on 29.12.20.
//

#ifndef PANDA_CONTROL_UTILS_H
#define PANDA_CONTROL_UTILS_H

#include "common.h"
#include "motion_planner.h"
#include <casadi/casadi.hpp>
using namespace casadi;

extern std::vector<std::array<double, 3>> ee_data;
extern std::vector<std::array<double, 3>> ee_desired_data;
extern std::vector<std::array<double, 3>> ee_calculated_desired_data;
extern std::vector<std::pair<DM, double>> solver_info;

extern std::array<double, 7> Initial_joints;
extern Eigen::Vector3d Goal_position;
extern std::array<double, 7> Goal_joints;

void write_ee_data();

void write_solver_data();

void load_data(std::string);

void showTF(MotionPlanner& planner, ros::NodeHandle& n_);

#endif //PANDA_CONTROL_UTILS_H
