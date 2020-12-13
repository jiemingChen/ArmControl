//
// Created by jieming on 20.10.20.
//

#ifndef PANDA_CONTROL_COMMON_FUNC_H
#define PANDA_CONTROL_COMMON_FUNC_H

#include <array>
#include <Eigen/Dense>
#include "franka_model.h"

using std::array; using Eigen::Matrix;

#ifndef M_PI
#define M_PI     3.14159265358979323846  /* pi */
#endif
#ifndef M_PI_2
#define M_PI_2   1.57079632679489661923  /* pi/2 */
#endif
#ifndef M_PI_4
#define M_PI_4   0.78539816339744830962  /* pi/4 */
#endif
struct struct_franka_T
{
    std::array< std::array<double, 16>, 15 > T_all;
    std::array< std::array<double, 16>, 10 > T; // 0-6: Joint 1-7; 7: Flange; 8: Gripper; 9: Stiffness Frame
};

struct struct_franka_TJ
{
    std::array< std::array<double, 16>, 15 > T_all;
    std::array< std::array<double, 16>, 10 > T; // 0-6: Joint 1-7; 7: Flange; 8: Gripper; 9: Stiffness Frame
    std::array< std::array<double, 42>, 15 > J_all;
    std::array< std::array<double, 42>, 10 > J; // 0-6: Joint 1-7; 7: Flange; 8: Gripper; 9: Stiffness Frame
    std::array<double, 49> M;
    std::array<double, 7> G;
};

struct franka_FK_struct
{
    std::array< Eigen::Matrix<double, 4, 4>, 15 > A_all; // all POIs
    std::array< Eigen::Matrix<double, 4, 4>, 15 > T_all;
    std::array< Eigen::Matrix<double, 4, 4>, 10 > A_J; // only joints
    std::array< Eigen::Matrix<double, 4, 4>, 10 > T_J; // 0-6: Joint 1-7; 7: Flange; 8: Gripper; 9: Stiffness Frame
};

struct franka_model_parameters_DeLuca_struct
{
    bool is_initialised = false;

    const std::array<double, 7> q_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
    const std::array<double, 7> q_max = {{ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973}};
    double dp1, dp2, dp3, dp4, ap4, dp5, dp6, ap6, dp7;
    std::array<double, 8> m;
    std::array<double, 16> F_T_EE;
    std::array< Eigen::Matrix<double, 3, 3>, 8 > Is;
    std::array< Eigen::Matrix<double, 4, 1>, 8 > rc4s;
    std::array< Eigen::Matrix<double, 3, 1>, 8 > rcs;
    Eigen::Matrix<double, 3, 1> g;
};

class FRANKA{
private:
    franka_model_parameters_DeLuca_struct franka_model_parameters_DeLuca;

public:
    FRANKA();
    void franka_model_DeLuca_init(array<double, 16> F_T_EE, double m_ee, array<double, 3> F_x_Cee, array<double, 9> I_ee);

    struct franka_FK_struct fn_franka_AT( const Matrix<double, 7, 1>& q, const Matrix<double, 4, 4>& EE_T_K );
    struct_franka_TJ fn_franka_Jacobian( const Eigen::Matrix<double, 7, 1>& q, const Eigen::Matrix<double, 4, 4>& EE_T_K );

    Eigen::Matrix7d mass_matrix(const Eigen::Vector7d &q);

    Eigen::Matrix7d coriolis_matrix(const Eigen::Vector7d &q,const Eigen::Vector7d &dq);

    Eigen::Vector7d gravity_vector(const Eigen::Vector7d &q);

    Eigen::Vector7d friction(const Eigen::Vector7d &dq);



};


#endif //PANDA_CONTROL_COMMON_FUNC_H
