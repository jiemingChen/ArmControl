//
// Created by jieming on 20.10.20.
//

#include <array>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>
using namespace std;
using Eigen::MatrixXd;

void jointImpedanceControl(){
    // Compliance parameters
    const double translational_stiffness{150};
    const double rotational_stiffness{10};
    MatrixXd stiffness(6,6), damping(6,6);
    stiffness.setZero();
    stiffness.topLeftCorner(3,3) << translational_stiffness*MatrixXd::Identity(3,3);
    stiffness.bottomRightCorner(3,3) << rotational_stiffness*MatrixXd::Identity(3,3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * MatrixXd::Identity(3,3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * MatrixXd::Identity(3,3);



}
