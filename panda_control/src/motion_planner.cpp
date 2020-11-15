//
// Created by jieming on 08.11.20.
//
#include "motion_planner.h"
extern vector<pair<DM, double>> solver_info;
extern vector<array<double, 3>> ee_data;
extern vector<array<double, 3>> ee_desired_data;
extern vector<array<double, 3>> ee_calculated_desired_data;

MotionPlanner::MotionPlanner(ros::NodeHandle& nh){
    nh_ = nh;
    sensorSub_ = nh_.subscribe("/robot1/joint_states", 1, &MotionPlanner::jointStatesCallback, this);
    trajPub_ =  nh_.advertise<trajectory_msgs::JointTrajectory>("/robot1/jointTrajectory", 2);

    position_desire_data_ = DM(1,3);
    orientation_desire_data_ = DM(3,3);
    Q0_data_ = DM(7,1);

    N_ = 0;
    x0_data_ = DM(19*(N_+1),1);
    position_desire_data_(0)=0.428; position_desire_data_(1)=0.425; position_desire_data_(2)=0.575;
    position_desired_ = SX::sym("pos_d",1,3);
    orientation_desired_ = SX::sym("ori_d",3,3);

    max_limit ={2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    min_limit ={ -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    vector<double> avg_limit_v = {0, 0, 0, -1.5708, 0, 1.8675, 0};
    vector<double> delta_limit_v = {2.8973*2, 1.7628*2, 2.8973*2, 3.0718-0.0698, 2.8973*2, 3.7525+0.0175, 2.8973*2};

    SX avg_limit(avg_limit_v);
    SX delta_limit(delta_limit_v);

    Slice all;
    Slice pos_slice(0,3);
    Slice ori_slice(3,12);
    Slice last_col(3,4);
     // ---- decision variables ---------
    X_ = SX::sym("X", 12,N_+1); // state trajectory
    Q_ = SX::sym("Q", 7,N_+1); // joint trajectory

    auto pos = X_(pos_slice,all);
    auto ori = X_(ori_slice,all);

    SX f;
    SX g(3*(N_+1), 1);  SX g_inital(6*(N_+1), 1);

    // ---- cost func --------
    SX weight = SX(1,7); weight(0)=8; weight(1)=7; weight(2)=6; weight(3)=5; weight(4)=4; weight(5)=3; weight(6)=2;
    for(uint i=0; i<N_+1; i++){
        for(uint j=0; j<7; j++) {

            f += pow((Q_(j, i) - avg_limit(j)) / delta_limit(j), 2);
        }
    }

     // ---- fk constraints --------
    for (int k=0;k<N_+1;++k) {
        SX transform =  forwardKinematic( Q_(all, k));

        g(0+k*12) = pos(0,k) - transform(0, last_col);
        g(1+k*12) = pos(1,k) - transform(1, last_col);
        g(2+k*12) = pos(2,k) - transform(2, last_col);


        g_inital(0+k*12) = pos(0,k) - position_desired_(0);
        g_inital(1+k*12) = pos(1,k) - position_desired_(1);
        g_inital(2+k*12) = pos(2,k) - position_desired_(2);

        auto ori_error = oriError(transform(Slice(0,3), Slice(0,3)), orientation_desired_);
        for(auto i=0; i<3; i++){
            g_inital(3+k*12+i) = ori_error(i);
        }

    }

    // ---- X, joint limit constraints --------

    vector<double> lbx_vec, ubx_vec;
    for(auto i=0; i<(N_+1)*3; i++){
        lbx_vec.push_back(-2);  // X range  TODO modify
        ubx_vec.push_back(2);
    }
    for(auto i=0; i<(N_+1)*9; i++){
        lbx_vec.push_back(-3);  // X range
        ubx_vec.push_back(3);
    }
    for(auto i=0; i<N_+1; i++){
        lbx_vec.insert(lbx_vec.end(), min_limit.begin(), min_limit.end());  // joint range
        ubx_vec.insert(ubx_vec.end(), max_limit.begin(), max_limit.end());
    }
    arg_["lbx"] =lbx_vec; arg_["ubx"] =ubx_vec;
    arg_["lbg"] = arg_["ubg"] = 0;

     SXDict nlp = {{"x", SX::vertcat({vec(X_), vec(Q_)})},
                   {"f", f},
                   {"g", vertcat(g, g_inital)},
                   {"p", vertcat(position_desired_, orientation_desired_) } };

    Dict opts;
    opts["ipopt.print_level"] = 0;

    // Create an NLP solver
    solver_ = nlpsol("solver", "ipopt", nlp, opts);
 //    cout << nlp["p"]  << endl;

    // --- q kinematic
    // bounds
    // colllision avoidance
}

Eigen::Vector7d MotionPlanner::iKSolv(const Eigen::Affine3d & goal_pose, const Eigen::Vector7d & current_q){
    receivX(goal_pose, current_q);

    arg_["x0"] = x0_data_;
    arg_["p"]  = vertcat(position_desire_data_,  orientation_desire_data_);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    DMDict res = solver_(arg_);
#if 0
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double tim = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    pair<DM, double> info = make_pair(res.at("g"), tim);
    solver_info.push_back(info);
#endif
//    cout << res.at("g") << endl;
    return copyOut( res.at("x"));

}

SX MotionPlanner::forwardKinematic(const SX& Q){
    auto c1 = cos(Q(0)); auto s1 = sin(Q(0));
    auto c2 = cos(Q(1)); auto s2 = sin(Q(1));
    auto c3 = cos(Q(2)); auto s3 = sin(Q(2));
    auto c4 = cos(Q(3)); auto s4 = sin(Q(3));
    auto c5 = cos(Q(4)); auto s5 = sin(Q(4));
    auto c6 = cos(Q(5)); auto s6 = sin(Q(5));
    auto c7 = cos(Q(6)); auto s7 = sin(Q(6));
    double dp1 = 0.1420; //dp1+dp2=0.333
    double dp2 = 0.1910;
    double dp3 = 0.1940; //dp3+dp4=0.316
    double dp4 = 0.1220;
    double ap4 = 0.0825;
    double dp5 = 0.1360; //dp5+dp6=0.384
    double dp6 = 0.2480;
    double ap6 = 0.0880;
    double dp7 = 0.1070;

    vector<vector<double>> A_all0_v = { {1.0,  0.0,  0.0, 0.0},
                                        {0.0,  1.0,  0.0, 0.0},
                                        {0.0,  0.0,  1.0, 0.0},
                                        {0.0,  0.0,  0.0, 1.0} };

    vector<vector<double>> A_all1_v = { {1.0,  0.0,  0.0,  0.0},
                                        {0.0,  1.0,  0.0,  0.0},
                                        {0.0,  0.0,  1.0,  dp1},
                                        {0.0,  0.0,  0.0,  1.0} };

    vector<vector<SX>>  A_all2_v =     { {c1,   -s1,  0.0,  0.0},
                                         {s1,    c1,  0.0,  0.0},
                                         {0.0,   0.0,  1.0, dp2},
                                         {0.0,  0.0,  0.0,  1.0} };

    vector<vector<SX>>  A_all3_v =     { {c2,   -s2,  0.0,  0.0},
                                         {0.0,   0.0,  1.0, 0.0},
                                         {-s2,   -c2,  0.0, 0.0},
                                         {0.0,  0.0,  0.0,  1.0} };

    vector<vector<double>>  A_all4_v = { {1.0,  0.0,  0.0, 0.0},
                                         {0.0,  1.0,  0.0, -dp3},
                                         {0.0,  0.0,  1.0, 0.0},
                                         {0.0,  0.0,  0.0, 1.0} };

    vector<vector<SX>>  A_all5_v =     { {c3,  -s3,   0.0,  0.0},
                                         {0.0,  0.0, -1.0,  -dp4},
                                         {s3,   c3,   0.0,  0.0},
                                         {0.0,  0.0,  0.0,  1.0} };

    vector<vector<SX>>  A_all6_v =     { {c4,  -s4,  0.0,  ap4},
                                         {0.0,  0.0, -1.0, 0.0},
                                         {s4,   c4,  0.0,  0.0},
                                         {0.0,  0.0,  0.0, 1.0} };

    vector<vector<double>>  A_all7_v = { {1.0,  0.0,  0.0, -ap4},
                                         {0.0,  1.0,  0.0, 0.0},
                                         {0.0,  0.0,  1.0, 0.0},
                                         {0.0,  0.0,  0.0, 1.0} };

    vector<vector<double>>  A_all8_v = { {1.0,  0.0,  0.0, 0.0},
                                         {0.0,  1.0,  0.0, dp5},
                                         {0.0,  0.0,  1.0, 0.0},
                                         {0.0,  0.0,  0.0, 1.0} };

    vector<vector<SX>>  A_all9_v =     { {c5,  -s5,  0.0,  0.0},
                                         {0.0,  0.0, 1.0,  dp6},
                                         {-s5,  -c5,  0.0,  0.0},
                                         {0.0,  0.0,  0.0, 1.0} };

    vector<vector<SX>>  A_all10_v =     { {c6,  -s6,  0.0,  0.0},
                                          {0.0,  0.0, -1.0,  0.0},
                                          {s6,   c6,  0.0,  0.0},
                                          {0.0,  0.0,  0.0, 1.0} };

    vector<vector<SX>>  A_all11_v =     { {c7,  -s7,  0.0,  ap6},
                                          {0.0,  0.0, -1.0,  0.0},
                                          {s7,   c7,  0.0,  0.0},
                                          {0.0,  0.0,  0.0, 1.0} };

    vector<vector<double>>  A_all12_v = { {1.0,  0.0,  0.0, 0.0},
                                          {0.0,  1.0,  0.0, 0.0},
                                          {0.0,  0.0,  1.0, dp7},
                                          {0.0,  0.0,  0.0, 1.0} };

    vector<vector<double>>  A_all13_v = { {0.7071068,  0.7071068,  0.0, 0.0},
                                          {-0.7071068, 0.7071068,  0.0, 0.0},
                                          {       0.0,       0.0,  1.0, 0.0584},
                                          {       0.0,       0.0,  0.0, 1.0} };
    SX A_all0(A_all0_v);
    SX A_all1(A_all1_v);
    SX A_all2(4,4); copy(A_all2, A_all2_v);
    SX A_all3(4,4); copy(A_all3, A_all3_v);
    SX A_all4(A_all4_v);
    SX A_all5(4,4); copy(A_all5, A_all5_v);
    SX A_all6(4,4); copy(A_all6, A_all6_v);
    SX A_all7(A_all7_v);
    SX A_all8(A_all8_v);
    SX A_all9(4,4); copy(A_all9, A_all9_v);
    SX A_all10(4,4); copy(A_all10, A_all10_v);
    SX A_all11(4,4); copy(A_all11, A_all11_v);
    SX A_all12(A_all12_v);
    SX A_all13(A_all13_v);

    SX T_J1 = mtimes( mtimes(A_all0, A_all1 ), A_all2);
    SX T_J2 = mtimes(T_J1, A_all3);
    SX T_J3 = mtimes(mtimes(T_J2, A_all4), A_all5);
    SX T_J4 = mtimes(T_J3, A_all6);
    SX T_J5 = mtimes(mtimes(mtimes(T_J4, A_all7), A_all8), A_all9);
    SX T_J6 = mtimes(T_J5, A_all10);
    SX T_J7 = mtimes(T_J6, A_all11);
    SX T_J8 = mtimes(T_J7, A_all12);
    SX T_J9 = mtimes(T_J8, A_all13);

    return  T_J9;
}

SX MotionPlanner::oriError(const SX& r, const SX& r2){

    auto r_reshaped = reshape(r,3,3);
    SX error = mtimes(r_reshaped, r2.T());
    SX res(3,1);
    res(0) =  error(2,1) - error(1,2);
    res(1) =  error(0,2) - error(2,0);
    res(2) =  error(1,0) - error(0,1);

//    SX I = SX::eye(3);
//    SX res =  norm_1(mtimes(r_reshaped, r2.T() ) - I);
    return res*0.5;
}

SX MotionPlanner::oriEqual(const SX& r, const SX& r2){
//    cout << r << endl;
    auto r_reshaped = reshape(r,3,3);
//    cout << r_reshaped << endl;
    SX res =  vec(r_reshaped -r2);
    return res;
}

//SX MotionPlanner::quaternionError(const SX& r, const SX& ori_desired){
////    cout << r << endl;
//    auto r_reshaped = reshape(r,3,3);
//    SX quat_desired(4,1);
//
//
//    return res;
//}


void MotionPlanner::copy(SX & mx, const vector<vector<SX>>& data){
    for(uint i=0; i<4; i++){
        for(uint j=0; j<4; j++){
            mx(i,j) = data[i][j];
        }
    }
}

Eigen::Vector7d MotionPlanner::copyOut(const DM& data){

    vector<double> vector_x = static_cast<std::vector<double>>(data);
    Eigen::Vector7d out;
    for(auto i=0; i<7; i++)
        out(i) = vector_x[i+12];
    //TODO modify speed up
    return out;
}

void MotionPlanner::receivX(const Eigen::Affine3d & goal_pose, const Eigen::Vector7d & current_q){
    Eigen::Vector3d position = goal_pose.translation();
    Eigen::Matrix3d ori = goal_pose.linear();

    for(uint i=0; i<3; i++){
        position_desire_data_(i) = position(i);
     }
    for(uint i=0; i<3; i++){
        for(uint j=0; j<3; j++){
            orientation_desire_data_(i,j) = ori(i,j);
        }
    }

    for(uint i=0; i<3; i++){
         x0_data_(i) = position(i);
    }
    for(uint i=0; i<3; i++){
        for(uint j=0; j<3; j++){
            x0_data_(3+j+i*3) = ori(j,i);
        }
    }

    for(uint i=12; i<19; i++){
        x0_data_(i) = current_q(i-12);
//        Q0_data_(i-12) = current_q(i-12);
    }


}


Eigen::Vector7d MotionPlanner::followTrajectoryOptim(Panda& robot){
    double x_d = 0.428;
    double y_d = 0.425;
    double z_d = 0.575;
    double time_max = 20.0;
    double radius = 0.2;

    std::array<double, 16> pose_goal = {0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, x_d, y_d, z_d, 1.0};

    auto time_main_ = ros::Time::now().toSec();

    if(time_main_>=10 && time_main_<=30){
        double angle = M_PI * (1 - std::cos(M_PI / time_max * (time_main_-10)));
        double delta_x = radius * std::sin(angle) * std::cos(angle);
        double delta_y = radius * std::sin(angle);
        pose_goal[12]=x_d-delta_x;
        pose_goal[13]=y_d-delta_y;
    }
    else if(time_main_>=30){
        pose_goal[12]=x_d;
        pose_goal[13]=y_d;
    }

    Eigen::Affine3d goal_transform(Eigen::Matrix4d::Map(pose_goal.data()));
     Eigen::Vector7d q_desired = iKSolv(goal_transform, jointPos_);

    if(time_main_>=10 && time_main_<=time_max+10){
        robot.setJoints(jointPos_,Eigen::Vector7d::Zero());
        auto cur_transform = robot.fkEE();
        Eigen::Vector3d cur_position(cur_transform.translation());

        ee_data.push_back({cur_position(0), cur_position(1), cur_position(2)}); // record data
    }
//    ee_desired_data.push_back({pose_goal[12], pose_goal[13], pose_goal[14]});

#if 0
    if(time_main_>=10 && time_main_<=time_max+10){
        robot.setJoints(q_desired, Eigen::Vector7d::Zero());
        Eigen::Affine3d desir_transform = robot.fkEE();
        Eigen::Vector3d desir_position(desir_transform.translation());

        ee_calculated_desired_data.push_back({desir_position(0), desir_position(1), desir_position(2)}); // record data
    }
#endif
    return q_desired;
}

void MotionPlanner::pubTrajectory(const Eigen::Vector7d& trajectory){
    trajectory_msgs::JointTrajectoryPoint j_p;
    trajectory_msgs::JointTrajectory j_traj;

    for(auto i=0; i<7; i++)
        j_p.positions.push_back(trajectory(i));

    j_traj.points.push_back(j_p);
    trajPub_.publish(j_traj);
}


//home
//    j_p.positions[0]=1.57019; j_p.positions[1]=-1.00367; j_p.positions[2]=-0.00379918;
//    j_p.positions[3]=-1.81187;j_p.positions[4]= 0.00660794; j_p.positions[5]= 0.807798; j_p.positions[6]=0.866066;

//for (auto&& s : res) {
//std::cout << std::setw(10) << s.first << ": " << std::vector<double>(s.second) << std::endl;
//}
//    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count() << "[ms]" << std::endl;
