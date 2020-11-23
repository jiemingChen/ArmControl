//
// Created by jieming on 20.10.20.
//

#include "common_func.h"

/*** Franka Model: Dynamic Parameter Initialisation ***/
void FRANKA::franka_model_DeLuca_init( array<double, 16> F_T_EE, double m_ee, array<double, 3> F_x_Cee, array<double, 9> I_ee )
{
    // We assume that during operation the end effector will not be changed.

    franka_model_parameters_DeLuca.dp1 = 0.1420; //dp1+dp2=0.333
    franka_model_parameters_DeLuca.dp2 = 0.1910;
    franka_model_parameters_DeLuca.dp3 = 0.1940; //dp3+dp4=0.316
    franka_model_parameters_DeLuca.dp4 = 0.1220;
    franka_model_parameters_DeLuca.ap4 = 0.0825;
    franka_model_parameters_DeLuca.dp5 = 0.1360; //dp5+dp6=0.384
    franka_model_parameters_DeLuca.dp6 = 0.2480;
    franka_model_parameters_DeLuca.ap6 = 0.0880;
    franka_model_parameters_DeLuca.dp7 = 0.1070;


    franka_model_parameters_DeLuca.g << 0.0, 0.0, -9.8;
    franka_model_parameters_DeLuca.F_T_EE = F_T_EE;
    franka_model_parameters_DeLuca.is_initialised = true;

    franka_model_parameters_DeLuca.m = {{4.970684, 0.646926, 3.228604, 3.587895, 1.225946, 1.666555, 7.35522e-01, m_ee}};

    const std::array<double, 7> Ixx = {{7.0337e-01, 7.9620e-03, 3.7242e-02, 2.5853e-02, 3.5549e-02, 1.9640e-03, 1.2516e-02}};
    const std::array<double, 7> Ixy = {{-1.3900e-04, -3.9250e-03, -4.7610e-03, 7.7960e-03, -2.1170e-03, 1.0900e-04, -4.2800e-04}};
    const std::array<double, 7> Iyz = {{1.9169e-02, 7.0400e-04, -1.2805e-02, 8.6410e-03, 2.2900e-04, 3.4100e-04, -7.4100e-04}};
    const std::array<double, 7> Iyy = {{7.0661e-01, 2.8110e-02, 3.6155e-02, 1.9552e-02, 2.9474e-02, 4.3540e-03, 1.0027e-02}};
    const std::array<double, 7> Izx = {{6.7720e-03, 1.0254e-02, -1.1396e-02, -1.3320e-03, -4.0370e-03, -1.1580e-03, -1.1960e-03}};
    const std::array<double, 7> Izz = {{9.1170e-03, 2.5995e-02, 1.0830e-02, 2.8323e-02, 8.6270e-03, 5.4330e-03, 4.8150e-03}};

    for (int l = 0; l < 7; l++)
        franka_model_parameters_DeLuca.Is[l] << Ixx[l], Ixy[l], Izx[l],
                Ixy[l], Iyy[l], Iyz[l],
                Izx[l], Iyz[l], Izz[l];
    franka_model_parameters_DeLuca.Is[7] = Eigen::Map< Eigen::Matrix<double, 3, 3> >(I_ee.data());

    franka_model_parameters_DeLuca.rc4s[0] << 3.875e-03, 2.081e-03, 0.0/*any*/, 1.0;
    franka_model_parameters_DeLuca.rc4s[1] << -3.141e-03, -2.872e-02, 3.495e-03, 1.0;
    franka_model_parameters_DeLuca.rc4s[2] << 2.7518e-02, 3.9252e-02, -6.6502e-02, 1.0;
    franka_model_parameters_DeLuca.rc4s[3] << -5.317e-02, 1.04419e-01, 2.7454e-02, 1.0;
    franka_model_parameters_DeLuca.rc4s[4] << -1.1953e-02, 4.1065e-02, -3.8437e-02, 1.0;
    franka_model_parameters_DeLuca.rc4s[5] << 6.0149e-02, -1.4117e-02, -1.0517e-02, 1.0;
    franka_model_parameters_DeLuca.rc4s[6] << 1.0517e-02, -4.252e-03, 6.1597e-02, 1.0;
    franka_model_parameters_DeLuca.rc4s[7] << F_x_Cee[0], F_x_Cee[1], F_x_Cee[2], 1.0;
    for (int l = 0; l < 8; l++)
        franka_model_parameters_DeLuca.rcs[l] = franka_model_parameters_DeLuca.rc4s[l].head<3>();



    return;
}

// initialisation with default Franka Hand
FRANKA::FRANKA(){
     std::array<double, 16> F_T_EE_array = {{0.7071, -0.7071, 0.0, 0.0, 0.7071, 0.7071, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1034, 1.0}}; // using default Franka Hand
     double m_ee = 0.73;
     std::array<double, 3> F_x_Cee_array = {{-0.01, 0.0, 0.03}};
     std::array<double, 9> I_ee_array = {{0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017}};
    franka_model_DeLuca_init(F_T_EE_array, m_ee, F_x_Cee_array, I_ee_array);
}

franka_FK_struct FRANKA::fn_franka_AT( const Matrix<double, 7, 1>& q, const Matrix<double, 4, 4>& EE_T_K )
{
    double c1 = std::cos(q[0]); double s1 = std::sin(q[0]);
    double c2 = std::cos(q[1]); double s2 = std::sin(q[1]);
    double c3 = std::cos(q[2]); double s3 = std::sin(q[2]);
    double c4 = std::cos(q[3]); double s4 = std::sin(q[3]);
    double c5 = std::cos(q[4]); double s5 = std::sin(q[4]);
    double c6 = std::cos(q[5]); double s6 = std::sin(q[5]);
    double c7 = std::cos(q[6]); double s7 = std::sin(q[6]);

    franka_FK_struct output;
    /*base*/
    output.A_all[0] << 1.0,  0.0,  0.0,                                   0.0,
                       0.0,  1.0,  0.0,                                   0.0,
                       0.0,  0.0,  1.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*J1*/
    output.A_all[1] << 1.0,  0.0,  0.0,                                   0.0,
                       0.0,  1.0,  0.0,                                   0.0,
                       0.0,  0.0,  1.0,    franka_model_parameters_DeLuca.dp1,
                       0.0,  0.0,  0.0,                                   1.0;
    /*O1*/
    output.A_all[2] << c1,   -s1,  0.0,                                   0.0,
                       s1,    c1,  0.0,                                   0.0,
                      0.0,   0.0,  1.0,    franka_model_parameters_DeLuca.dp2,
                      0.0,   0.0,  0.0,                                   1.0;
    /*O2*/
    /*J2*/
    output.A_all[3] << c2,   -s2,  0.0,                                   0.0,
                      0.0,   0.0,  1.0,                                   0.0,
                      -s2,   -c2,  0.0,                                   0.0,
                      0.0,   0.0,  0.0,                                   1.0;
    /*J3*/
    output.A_all[4] << 1.0,  0.0,  0.0,                                   0.0,
                       0.0,  1.0,  0.0,   -franka_model_parameters_DeLuca.dp3,
                       0.0,  0.0,  1.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*O3*/
    /*LSE*/
    output.A_all[5] <<  c3,  -s3,  0.0,                                   0.0,
                       0.0,  0.0, -1.0,   -franka_model_parameters_DeLuca.dp4,
                        s3,   c3,  0.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*O4*/
    /*J4*/
    output.A_all[6] <<  c4,  -s4,  0.0,    franka_model_parameters_DeLuca.ap4,
                       0.0,  0.0, -1.0,                                   0.0,
                        s4,   c4,  0.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*USE*/
    output.A_all[7] << 1.0,  0.0,  0.0,   -franka_model_parameters_DeLuca.ap4,
                       0.0,  1.0,  0.0,                                   0.0,
                       0.0,  0.0,  1.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*J5*/
    output.A_all[8] << 1.0,  0.0,  0.0,                                   0.0,
                       0.0,  1.0,  0.0,    franka_model_parameters_DeLuca.dp5,
                       0.0,  0.0,  1.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*O5*/
    output.A_all[9] <<  c5,  -s5,  0.0,                                   0.0,
                       0.0,  0.0,  1.0,    franka_model_parameters_DeLuca.dp6,
                       -s5,  -c5,  0.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*O6*/
    /*J6*/
    output.A_all[10] << c6,  -s6,  0.0,                                   0.0,
                       0.0,  0.0, -1.0,                                   0.0,
                        s6,   c6,  0.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*O7*/
    output.A_all[11] << c7,  -s7,  0.0,    franka_model_parameters_DeLuca.ap6,
                       0.0,  0.0, -1.0,                                   0.0,
                        s7,   c7,  0.0,                                   0.0,
                       0.0,  0.0,  0.0,                                   1.0;
    /*OF*/
    output.A_all[12] <<1.0,  0.0,  0.0,                                   0.0,
                       0.0,  1.0,  0.0,                                   0.0,
                       0.0,  0.0,  1.0,    franka_model_parameters_DeLuca.dp7,
                       0.0,  0.0,  0.0,                                   1.0;

    output.A_all[13] = Eigen::Map< Eigen::Matrix<double, 4, 4> >(franka_model_parameters_DeLuca.F_T_EE.data());
    /*OEE*/
    output.A_all[14] = EE_T_K;
    /*OK*/

    output.T_all[0].setIdentity();
    for (unsigned int i = 1; i < 15; i++)
        output.T_all[i] = output.T_all[i - 1]*output.A_all[i];

    output.A_J[0]  = output.A_all[0]*output.A_all[1]*output.A_all[2]; // O1 (J1)
    output.A_J[1]  = output.A_all[3];                                 // O2 (J2)
    output.A_J[2]  = output.A_all[4]*output.A_all[5];                 // O3 (J3)
    output.A_J[3]  = output.A_all[6];                                 // O4 (J4)
    output.A_J[4]  = output.A_all[7]*output.A_all[8]*output.A_all[9]; // O5 (J5)
    output.A_J[5]  = output.A_all[10];                                // O6 (J6)
    output.A_J[6]  = output.A_all[11];                                // O7 (J7)
    output.A_J[7]  = output.A_all[12];                                // OF (Flange)
    output.A_J[8]  = output.A_all[13];                                // OEE (End Effector)
    output.A_J[9]  = output.A_all[14];                                // OK (Stiffness frame)

    output.T_J[0]  = output.T_all[2];  // O1 (J1)
    output.T_J[1]  = output.T_all[3];  // O2 (J2)
    output.T_J[2]  = output.T_all[5];  // O3 (J3)
    output.T_J[3]  = output.T_all[6];  // O4 (J4)
    output.T_J[4]  = output.T_all[9];  // O5 (J5)
    output.T_J[5]  = output.T_all[10]; // O6 (J6)
    output.T_J[6]  = output.T_all[11]; // O7 (J7)
    output.T_J[7]  = output.T_all[12]; // OF (Flange)
    output.T_J[8]  = output.T_all[13]; // OEE (End Effector)
    output.T_J[9]  = output.T_all[14]; // OK (Stiffness frame)

    return output;
}

struct_franka_TJ FRANKA::fn_franka_Jacobian( const Eigen::Matrix<double, 7, 1>& q, const Eigen::Matrix<double, 4, 4>& EE_T_K ){

    franka_FK_struct franka_FK = fn_franka_AT(q, EE_T_K);

    std::array< Eigen::Matrix<double, 6, 7>, 15 > J_all;
//    std::array< Eigen::Matrix<double, 6, 7>, 10 > J;

    std::array< Eigen::Matrix<double, 3, 1>, 15 > z;
    std::array< Eigen::Matrix<double, 3, 1>, 15 > d;

    const std::array<unsigned int, 15> rotate_with_joint = {{0, 0, 1, 2, 2, 3, 4, 4, 4, 5, 6, 7, 7, 7, 7}};
    const std::array<unsigned int, 10> i_J = {{2, 3, 5, 6, 9, 10, 11, 12, 13, 14}};

    const std::array<bool, 15> is_mass_attached = {{false, false, true, true, false, true, true, false, false, true, true, true, false, false, false}};

    int i_mI = 0;  // for mass matrix
    Eigen::Matrix<double, 7, 7> M = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Matrix<double, 7, 1> G = Eigen::Matrix<double, 7, 1>::Zero();
    for (unsigned int i = 0; i < 15; i++){
        z[i] = franka_FK.T_all[i].block<3, 1>(0, 2);
        d[i] = franka_FK.T_all[i].block<3, 1>(0, 3);
        J_all[i].setZero();

        for (unsigned int col = 0; col < rotate_with_joint[i]; col++){
            Eigen::Matrix<double, 3, 1> dd_col = d[i] - d[i_J[col]];
            Eigen::Matrix<double, 3, 1> Jv_col_vec = z[i_J[col]].cross(dd_col);
            J_all[i].block<3, 1>(0, col) = Eigen::Map< Eigen::Matrix<double, 3, 1> >(Jv_col_vec.data());
            J_all[i].block<3, 1>(3, col) = Eigen::Map< Eigen::Matrix<double, 3, 1> >(z[i_J[col]].data());
        }

        if (is_mass_attached[i])
        {
            Eigen::Matrix<double, 3, 7> Jvc_i = Eigen::Matrix<double, 3, 7>::Zero();
            for (unsigned int col = 0; col < rotate_with_joint[i]; col++)
            {
                Eigen::Matrix<double, 4, 1> d4_i = franka_FK.T_all[i]*franka_model_parameters_DeLuca.rc4s[i_mI];
                Eigen::Matrix<double, 3, 1> ddc_col = d4_i.head<3>() - d[i_J[col]];
                Eigen::Matrix<double, 3, 1> Jvc_col_vec = z[i_J[col]].cross(ddc_col);
                Jvc_i.col(col) = Eigen::Map< Eigen::Matrix<double, 3, 1> >(Jvc_col_vec.data());
            }
            Eigen::Matrix<double, 3, 3> R_i = franka_FK.T_all[i].topLeftCorner<3, 3>();
            Eigen::Matrix<double, 3, 7> Jwc_i = J_all[i].bottomRows<3>();
            M += franka_model_parameters_DeLuca.m[i_mI]*Jvc_i.transpose()*Jvc_i
                 + Jwc_i.transpose()*R_i*franka_model_parameters_DeLuca.Is[i_mI]*R_i.transpose()*Jwc_i;

            G -= franka_model_parameters_DeLuca.m[i_mI]*Jvc_i.transpose()*franka_model_parameters_DeLuca.g;

            i_mI++;
        }

    }

    struct_franka_TJ output;
    for (unsigned int i = 0; i < 15; i++){
        Eigen::Map< Eigen::Matrix<double, 4, 4> >(output.T_all[i].data()) = franka_FK.T_all[i];;
        Eigen::Map< Eigen::Matrix<double, 6, 7> >(output.J_all[i].data()) = J_all[i];
    }

    for (unsigned int i = 0; i < 10; i++){
        Eigen::Map< Eigen::Matrix<double, 4, 4> >(output.T[i].data()) = franka_FK.T_J[i];;
        Eigen::Map< Eigen::Matrix<double, 6, 7> >(output.J[i].data()) = J_all[i_J[i]];
    }
    Eigen::Map< Eigen::Matrix<double, 7, 7> >(output.M.data()) = M;
    Eigen::Map< Eigen::Matrix<double, 7, 1> >(output.G.data()) = G;
    return output; //J_all[14]
}

Eigen::Matrix7d FRANKA::mass_matrix(const Eigen::Vector7d &q){
    return MassMatrix(q);
}

Eigen::Matrix7d FRANKA::coriolis_matrix(const Eigen::Vector7d &q,const Eigen::Vector7d &dq){
    return CoriolisMatrix(q, dq);
}

Eigen::Vector7d FRANKA::gravity_vector(const Eigen::Vector7d &q){
    return GravityVector(q);
}

Eigen::Vector7d FRANKA::friction(const Eigen::Vector7d &dq){
    return Friction(dq);
}
