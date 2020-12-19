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
    obsSub_ = nh_.subscribe("near_boxes", 1, &MotionPlanner::nearObsCallback, this);
    trajPub_ =  nh_.advertise<trajectory_msgs::JointTrajectory>("/robot1/jointTrajectory", 2);

    position_desire_data_ = DM(1,3);
    position_desire_data_(0)=0.428; position_desire_data_(1)=0.425; position_desire_data_(2)=0.575;

    N_ = 0;
    jointPos_ <<  0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785;

    max_limit ={2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    min_limit ={ -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    dq_min_limit = {-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100};
    dq_max_limit = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
    avg_limit_ = {0, 0, 0, -1.5708, 0, 1.8675, 0};
    delta_limit_ = {2.8973*2, 1.7628*2, 2.8973*2, 3.0718-0.0698, 2.8973*2, 3.7525+0.0175, 2.8973*2};



    first_solve_ = true; first_receive_=false;

    obs_.reserve(20);
//    buildModel();
//    buildIKModel();
//    buildModel2();
}



void MotionPlanner::buildIKModel(){
    Slice all;
    Slice pos_slice(0,3);
    Slice ori_slice(3,12);
    Slice last_col(3,4);

    x0_data_ = DM(19*(N_+1),1);
    orientation_desire_data_ = DM(3,3);
    // ---- decision variables ---------
    X_ = SX::sym("X", 12,N_+1); // state trajectory
    Q_ = SX::sym("Q", 7,N_+1); // joint trajectory
    auto pos = X_(pos_slice,all);
    auto ori = X_(ori_slice,all);
    // ---- parameter variables ---------
    position_desired_ = SX::sym("pos_d",1,3);
    orientation_desired_ = SX::sym("ori_d",3,3);
    // ---- cost function ---------
    SX f=0;
#if 0
    SX weight = SX(1,7); weight(0)=8; weight(1)=7; weight(2)=6; weight(3)=5; weight(4)=4; weight(5)=3; weight(6)=2;
    for(uint i=0; i<N_+1; i++){
        for(uint j=0; j<7; j++) {

            f += pow((Q_(j, i) - avg_limit(j)) / delta_limit(j), 2);
        }
    }
#endif
//    for (int k=0;k<N_+1;++k) {
//        SX transform =  forwardKinematic( Q_(all, k));
//        auto ori_error = eluerError(transform(Slice(0,3), Slice(0,3)), orientation_desired_);
//        f += pow(ori_error(0), 2);
//        f += pow(ori_error(1), 2);
//        f += pow(ori_error(2), 2);
//    }
        // ---- equality constraints ---------
    SX g(3*(N_+1), 1);  SX g_inital(4*(N_+1), 1); // SX g_inital(6*(N_+1), 1);
    // ---- fk constraints --------
    for (int k=0;k<N_+1;++k) {
        SX transform =  forwardKinematic( Q_(all, k));

        g(0+k*3) = pos(0,k) - transform(0, last_col);
        g(1+k*3) = pos(1,k) - transform(1, last_col);
        g(2+k*3) = pos(2,k) - transform(2, last_col);

        g_inital(0+k*6) = pos(0,k) - position_desired_(0);
        g_inital(1+k*6) = pos(1,k) - position_desired_(1);
        g_inital(2+k*6) = pos(2,k) - position_desired_(2);

////        g_inital(0+k*6) = pow(transform(0, last_col)- position_desired_(0), 2) + pow(transform(1, last_col)- position_desired_(1), 2) + pow(transform(2, last_col)- position_desired_(2), 2);
//        g_inital(0+k*6) =  pow(transform(0, last_col)- position_desired_(0), 2);
//        g_inital(1+k*6) =  pow(transform(1, last_col)- position_desired_(1), 2);
//        g_inital(2+k*6) =  pow(transform(2, last_col)- position_desired_(2), 2);

        auto ori_error = oriError3(transform(Slice(0,3), Slice(0,3)), orientation_desired_);
        g_inital(3+k*6) = ori_error(0);
//        g_inital(4+k*6) = ori_error(1);
//        g_inital(5+k*6) = ori_error(2);
    }
    // ----bounds--------
    vector<double> lbx_vec, ubx_vec;
    for(auto i=0; i<(N_+1)*3; i++){
        lbx_vec.push_back(-2);  // X range  TODO modify
        ubx_vec.push_back(2);
    }
    for(auto i=0; i<(N_+1)*9; i++){
        lbx_vec.push_back(-3);
        ubx_vec.push_back(3);
    }
    for(auto i=0; i<N_+1; i++){
        lbx_vec.insert(lbx_vec.end(), min_limit.begin(), min_limit.end());  // joint range
        ubx_vec.insert(ubx_vec.end(), max_limit.begin(), max_limit.end());
    }

    SXDict nlp = {{"x", SX::vertcat({vec(X_), vec(Q_)})},
                  {"f", f},
                  {"g", vertcat(g, g_inital)},
//                  {"g",  g_inital},
                  {"p", vertcat(position_desired_, orientation_desired_) } };

    // ----arg--------
    arg_["lbx"] =lbx_vec; arg_["ubx"] =ubx_vec;
    arg_["lbg"] = arg_["ubg"] = 0;

    Dict opts;
    opts["ipopt.print_level"] = 0;

     solver_ = nlpsol("solver", "ipopt", nlp, opts);
     // --- q kinematic
    // colllision avoidance
}

void MotionPlanner::buildModel2() {

    Slice all;
    Slice last_col(3,4);

    N_ = 10;
    // ---- parameter ---------
    position_desire_data_ = DM(3,1);
    joint_position_desired_data_ = DM(7,1);
    orientation_desire_data_ = DM(9,1);
    x0_data_ = DM(14*N_+7, 1);  // 7+7   q, qdot
    Q0_data_ = DM(7,1);

    // ---- decision variables ---------
    Q_ = SX::sym("Q", 7, N_+1); // joint trajectory
    Qdot_ = SX::sym("dQ", 7, N_); // joint trajectory
    double dt = 0.1;
     // ---- parameter variables ---------
    orientation_desired_ = SX::sym("ori_d",9,1);
    feedback_variable_   = SX::sym("feedback",7,1);
    joint_position_desired_ = SX::sym("jointDesir", 7, 1);
    position_desired_    = SX::sym("pos_d",3,1);

    // ---- cost function ---------
    SX f=0;
    // 1---- desired position cost --------
#if 0  // joint space
    for(uint k=1; k<N_; k++){
        for(uint i=0; i<7; i++)
            f += 5*pow(Q_(i,k) - joint_position_desired_(i), 2);
    }
    for(uint i=0; i<7; i++)
        f += 10*pow(Q_(i,N_) - joint_position_desired_(i), 2);
#else
    // work space
    for(uint k=1; k<N_; k++){
        SX transform =  forwardKinematic( Q_(all, k));
        for(uint i=0; i<3; i++)
            f += 10*pow(transform(i,3) - position_desired_(i), 2);
     }
    SX transform =  forwardKinematic( Q_(all, N_));
    for(uint i=0; i<3; i++)
        f += 20*pow(transform(i,3) - position_desired_(i), 2);
    f += 50*oriError3(transform(Slice(0,3), Slice(0,3)), orientation_desired_);

#endif

    // 2---- smooth cost --------
    for(uint k=1; k<N_; ++k){
        for(uint i=0; i<7; i++)
            f += pow(Qdot_(i, k)-Qdot_(i, k-1), 2)*0.3;
    }
    // 3---- collision cost --------
     for(uint k=1; k<N_+1; ++k) {
        auto links =  forwardKinematicMultiLinks( Q_(all, k) );
        for(auto& pairr: links){
            auto transform = pairr.first;
            auto link_radius = pairr.second;
            double field_radius = link_radius + 0.05;  // object radius
            for(uint j=0; j<obs_.size(); j++){
                auto dist = pow( pow(transform(0,3)-obs_[j][0], 2) + pow(transform(1,3)-obs_[j][1], 2) + pow(transform(2,3)-obs_[j][2], 2), 0.5 );
                f += if_else( dist<=SXElem(field_radius), pow( (1/dist- 1/field_radius), 2)* 40000,  SXElem(0));
                //f += if_else(dist<=SXElem(0.1), pow(dist-0.1, 2)*800000,  SXElem(0) );
            }
        }
     }
    // 4---- close to middle cost --------
    vector<double> joint_weight = {0.2, 0.2, 2, 1, 1.4, 1, 0.8};
    for(uint k=1; k<N_; ++k) {
        for (uint i = 0; i < 7; i++) {
            f += pow((Q_(i, k)-avg_limit_[i])/dq_max_limit[i], 2)*joint_weight[i]  ;
        }
    }

    // ---- constraints function ---------
    SX g_orientation(1*(N_), 1);  SX g_q_kin(7*(N_), 1); SX g_inital(7,1);   SX g_obs(N_*5,1);
    // 1---- orientation constraints --------
    for (int k=1;k<N_+1;++k) {
        SX transform =  forwardKinematic( Q_(all, k));
        auto ori_error = oriError3(transform(Slice(0,3), Slice(0,3)), orientation_desired_);
        g_orientation(0+(k-1)) = ori_error(0);
    }
    // 2---- joint differential kinematics --------
    for (int k=1; k<N_+1; ++k) {
        for(uint i=0; i<7; i++){
            g_q_kin(i+7*(k-1)) = Q_(i,k) - Q_(i,k-1) - Qdot_(i,k-1)*dt;
        }
    }
    // 3---- initial value constraints --------
    for(uint i=0; i<7; i++) {
        g_inital(i) = Q_(i, 0) - feedback_variable_(i);
    }
    // ---- obstacles constraints --------
     vector<double> lower_bound_v, upper_bound_v;
    // k:steps  i:multi-links  j:obs
    for(uint k=1; k<N_+1; ++k) {
        auto links =  forwardKinematicMultiLinks( Q_(all, k) );
        for(uint i=0; i<links.size(); i++){ //5
             auto pairr = links[i];
            auto transform = pairr.first;
            auto link_radius = pairr.second;
            double field_radius = link_radius + 0.05;  // object radius

             for(uint j=0; j<obs_.size(); j++){
                auto dist = pow(pow(transform(0,3)-obs_[j][0], 2) + pow(transform(1,3)-obs_[j][1], 2) + pow(transform(2,3)-obs_[j][2], 2), 0.5);
                g_obs(i + (k-1)*5 )= dist;
                 lower_bound_v.push_back(field_radius);
                 upper_bound_v.push_back(10000);
             }
        }
     }

         // ----bounds--------
    vector<double> lbx_vec, ubx_vec;
    for(auto i=0; i<N_+1; i++){
        lbx_vec.insert(lbx_vec.end(), min_limit.begin(), min_limit.end());  // joint range
        ubx_vec.insert(ubx_vec.end(), max_limit.begin(), max_limit.end());
    }
    for(auto i=0; i<N_; i++){
        lbx_vec.insert(lbx_vec.end(), dq_min_limit.begin(), dq_min_limit.end());  // joint speed range
        ubx_vec.insert(ubx_vec.end(), dq_max_limit.begin(), dq_max_limit.end());
    }

    vector<double> lbg_vec, ubg_vec;
    vector<double> zero_vec;

//    zero_vec = vector<double> (N_ , 0);
//    lbg_vec.insert(lbg_vec.end(), zero_vec.begin(), zero_vec.end());  // orientation constraints
//    ubg_vec.insert(ubg_vec.end(), zero_vec.begin(), zero_vec.end());
    zero_vec = vector<double>(7*N_,0);
    lbg_vec.insert(lbg_vec.end(), zero_vec.begin(), zero_vec.end());  // kinematics constraints
    ubg_vec.insert(ubg_vec.end(), zero_vec.begin(), zero_vec.end());
    zero_vec = vector<double>(7,0);
    lbg_vec.insert(lbg_vec.end(), zero_vec.begin(), zero_vec.end());  //  initial value constraints
    ubg_vec.insert(ubg_vec.end(), zero_vec.begin(), zero_vec.end());

//    lbg_vec.insert(lbg_vec.end(), lower_bound_v.begin(), lower_bound_v.end());  //  obstacles constraints
//    ubg_vec.insert(ubg_vec.end(), upper_bound_v.begin(), upper_bound_v.end());

    SXDict nlp = {{"x", SX::vertcat({vec(Q_), vec(Qdot_)})},
                  {"f", f},
//                  {"g", vertcat(g_orientation, g_q_kin, g_inital) }, //
                  {"g", vertcat(  g_q_kin, g_inital)},
                   {"p", vertcat(joint_position_desired_, orientation_desired_, feedback_variable_, position_desired_) } };

    // ----arg--------
    arg_["lbx"] = lbx_vec;  arg_["ubx"] = ubx_vec;
    arg_["lbg"] = lbg_vec;  arg_["ubg"] = ubg_vec;

    Dict opts;
    opts["ipopt.print_level"] = 0;

    solver_ = nlpsol("solver", "ipopt", nlp, opts);

}

void MotionPlanner::buildMPC() {

    Slice all;
    Slice last_col(3,4);

    N_ = 10;
    // ---- parameter ---------
    position_desire_data_ = DM(3,1);
    joint_position_desired_data_ = DM(7,1);
    orientation_desire_data_ = DM(9,1);
    x0_data_ = DM(14*N_+7, 1);  // 7+7   q, qdot
    Q0_data_ = DM(7,1);

    // ---- decision variables ---------
    Q_ = SX::sym("Q", 7, N_+1); // joint trajectory
    Qdot_ = SX::sym("dQ", 7, N_); // joint trajectory
    double dt = 0.1;
    // ---- parameter variables ---------
    orientation_desired_ = SX::sym("ori_d",9,1);
    feedback_variable_   = SX::sym("feedback",7,1);
    joint_position_desired_ = SX::sym("jointDesir", 7, 1);
    position_desired_    = SX::sym("pos_d",3,1);

    // ---- cost function ---------
    // 1---- desired position cost --------
    f_ =0;
#if 0  // joint space
    for(uint k=1; k<N_; k++){
        for(uint i=0; i<7; i++)
            f += 5*pow(Q_(i,k) - joint_position_desired_(i), 2);
    }
    for(uint i=0; i<7; i++)
        f += 10*pow(Q_(i,N_) - joint_position_desired_(i), 2);
#else
    // work space
    for(uint k=1; k<N_; k++){
        SX transform =  forwardKinematic( Q_(all, k));
        for(uint i=0; i<3; i++)
            f_ += 10*pow(transform(i,3) - position_desired_(i), 2);
    }
    SX transform =  forwardKinematic( Q_(all, N_));
    for(uint i=0; i<3; i++)
        f_ += 20*pow(transform(i,3) - position_desired_(i), 2);
    f_ += 50*oriError3(transform(Slice(0,3), Slice(0,3)), orientation_desired_);
#endif

    // 2---- smooth cost --------
    for(uint k=1; k<N_; ++k){
        for(uint i=0; i<7; i++)
            f_ += pow(Qdot_(i, k)-Qdot_(i, k-1), 2)*0.3;
    }
    // 3---- close to middle cost --------
    vector<double> joint_weight = {0.2, 0.2, 2, 1, 1.4, 1, 0.8};
    for(uint k=1; k<N_; ++k) {
        for (uint i = 0; i < 7; i++) {
            f_ += pow((Q_(i, k)-avg_limit_[i])/dq_max_limit[i], 2)*joint_weight[i]  ;
        }
    }

    // ---- constraints function ---------
    g_orientation_ = SX(1*(N_), 1);
    g_q_kin_       = SX(7*(N_), 1);
    g_inital_      = SX(7,1);
    g_obs_         = SX(N_*5,1);
    // 1---- orientation constraints --------
    for (int k=1;k<N_+1;++k) {
        SX transform =  forwardKinematic( Q_(all, k));
        auto ori_error = oriError3(transform(Slice(0,3), Slice(0,3)), orientation_desired_);
        g_orientation_(0+(k-1)) = ori_error(0);
    }
    // 2---- joint differential kinematics --------
    for (int k=1; k<N_+1; ++k) {
        for(uint i=0; i<7; i++){
            g_q_kin_(i+7*(k-1)) = Q_(i,k) - Q_(i,k-1) - Qdot_(i,k-1)*dt;
        }
    }
    // 3---- initial value constraints --------
    for(uint i=0; i<7; i++) {
        g_inital_(i) = Q_(i, 0) - feedback_variable_(i);
    }
    // ---- obstacles constraints --------
    vector<double> lower_bound_v, upper_bound_v;
    // k:steps  i:multi-links  j:obs
    for(uint k=1; k<N_+1; ++k) {
        auto links =  forwardKinematicMultiLinks( Q_(all, k) );
        for(uint i=0; i<links.size(); i++){ //5
            auto pairr = links[i];
            auto transform = pairr.first;
            auto link_radius = pairr.second;
            double field_radius = link_radius + 0.05;  // object radius

            for(uint j=0; j<obs_.size(); j++){
                auto dist = pow(pow(transform(0,3)-obs_[j][0], 2) + pow(transform(1,3)-obs_[j][1], 2) + pow(transform(2,3)-obs_[j][2], 2), 0.5);
                g_obs_(i + (k-1)*5 )= dist;
                lower_bound_v.push_back(field_radius);
                upper_bound_v.push_back(10000);
            }
        }
    }

    // ----bounds--------
    vector<double> lbx_vec, ubx_vec;
    for(auto i=0; i<N_+1; i++){
        lbx_vec.insert(lbx_vec.end(), min_limit.begin(), min_limit.end());  // joint range
        ubx_vec.insert(ubx_vec.end(), max_limit.begin(), max_limit.end());
    }
    for(auto i=0; i<N_; i++){
        lbx_vec.insert(lbx_vec.end(), dq_min_limit.begin(), dq_min_limit.end());  // joint speed range
        ubx_vec.insert(ubx_vec.end(), dq_max_limit.begin(), dq_max_limit.end());
    }

    vector<double> lbg_vec, ubg_vec;
    vector<double> zero_vec;

//    zero_vec = vector<double> (N_ , 0);
//    lbg_vec.insert(lbg_vec.end(), zero_vec.begin(), zero_vec.end());  // orientation constraints
//    ubg_vec.insert(ubg_vec.end(), zero_vec.begin(), zero_vec.end());
    zero_vec = vector<double>(7*N_,0);
    lbg_vec.insert(lbg_vec.end(), zero_vec.begin(), zero_vec.end());  // kinematics constraints
    ubg_vec.insert(ubg_vec.end(), zero_vec.begin(), zero_vec.end());
    zero_vec = vector<double>(7,0);
    lbg_vec.insert(lbg_vec.end(), zero_vec.begin(), zero_vec.end());  //  initial value constraints
    ubg_vec.insert(ubg_vec.end(), zero_vec.begin(), zero_vec.end());

//    lbg_vec.insert(lbg_vec.end(), lower_bound_v.begin(), lower_bound_v.end());  //  obstacles constraints
//    ubg_vec.insert(ubg_vec.end(), upper_bound_v.begin(), upper_bound_v.end());

    nlp_ = {{"x", SX::vertcat({vec(Q_), vec(Qdot_)})},
            {"f", f_},
            //{"g", vertcat(g_orientation, g_q_kin, g_inital) }, //
            {"g", vertcat(  g_q_kin_, g_inital_)},
            {"p", vertcat(joint_position_desired_, orientation_desired_, feedback_variable_, position_desired_) } };

    // ----arg--------
    arg_["lbx"] = lbx_vec;  arg_["ubx"] = ubx_vec;
    arg_["lbg"] = lbg_vec;  arg_["ubg"] = ubg_vec;

    Dict opts;
    opts["ipopt.print_level"] = 0;

    solver_ = nlpsol("solver", "ipopt", nlp_, opts);

}

void MotionPlanner::addObstaclesToMPC() {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Slice all;
    Slice last_col(3,4);

    // ---- cost function ---------
    SX f=0;
    // 3---- collision cost --------
    for(uint k=1; k<N_+1; ++k) {
        auto links =  forwardKinematicMultiLinks( Q_(all, k) );
        for(auto& pairr: links){
            auto transform = pairr.first;
            auto link_radius = pairr.second;
            double field_radius =0;
            for(uint j=0; j<obs_.size(); j++){
                field_radius  = link_radius + obs_[j][3];  // object radius

                auto dist = pow(pow(transform(0,3)-obs_[j][0], 2) + pow(transform(1,3)-obs_[j][1], 2)
                        +pow(transform(2,3)-obs_[j][2], 2), 0.5 );
                f += if_else( dist<=SXElem(field_radius), pow( (1/dist- 1/field_radius), 2)* 40000,  SXElem(0));
                //f += if_else(dist<=SXElem(0.1), pow(dist-0.1, 2)*800000,  SXElem(0) );
            }
        }
    }


    nlp_["f"] = f_ + f;


    Dict opts;
    opts["ipopt.print_level"] = 0;

    solver_ = nlpsol("solver", "ipopt", nlp_, opts);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double tim = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    cout << "consume time:  " << tim << "ms"<< endl;
}


//******************** casadi solve function ik, work space, joint space *******************************
Eigen::Vector7d MotionPlanner::iKSolv(const Eigen::Affine3d & goal_pose, const Eigen::Vector7d & current_q){
//    receivX(goal_pose, current_q);
    // 1. initialize
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
     }

    // 2. solve IK
    arg_["x0"] = x0_data_;
    arg_["p"]  = vertcat(position_desire_data_,  orientation_desire_data_);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    DMDict res = solver_(arg_);

#if 1
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double tim = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    pair<DM, double> info = make_pair(res.at("g"), tim);
    solver_info.push_back(info);
#endif

    // 3. return
    vector<double> vector_x = static_cast<std::vector<double>>(res.at("x"));

    Eigen::Vector7d out;
    for(auto i=0; i<7; i++)
        out(i) = vector_x[i+12];

    return out;
//    cout << res.at("g") << endl;
}

Eigen::Vector7d MotionPlanner::MPCSolv(const Eigen::Affine3d & goal_pose, const Eigen::Vector7d & current_q){
    // 1. initialize
    Eigen::Vector3d position = goal_pose.translation();
    Eigen::Matrix3d ori = goal_pose.linear();

    for(uint i=0; i<3; i++){
        position_desire_data_(i) = position(i);
    }
    for(uint i=0; i<3; i++){
        for(uint j=0; j<3; j++){
            orientation_desire_data_(j+i*3) = ori(j,i);
        }
    }
    for(uint i=0; i<7; i++){
        Q0_data_(i) = current_q(i);
    }

    if(first_solve_){
        for(uint k=0; k<N_+1; k++){
            for(uint i=0; i<3; i++)
                x0_data_(i+k*3) = position(i);
        }
        for(uint k=0; k<N_+1; k++){
            for(uint i=0; i<7; i++)
                x0_data_(3*(N_+1) + i + k*7) = current_q(i);
        }
        for(uint k=0; k<N_; k++){
            for(uint i=0; i<7; i++)
                x0_data_(10*(N_+1) + i + k*7) = 0;
        }
    }
    else{
        x0_data_ = last_x_;
    }

    // 2. solve
    arg_["x0"] = x0_data_;
//    cout << position_desire_data_ << endl;
    arg_["p"]  = vertcat(position_desire_data_,  orientation_desire_data_, Q0_data_);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    DMDict res = solver_(arg_);

#if 0
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double tim = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    pair<DM, double> info = make_pair(res.at("g"), tim);
    solver_info.push_back(info);
#endif
     if(first_solve_)
        first_solve_ = false;

    vector<double> vector_x = static_cast<std::vector<double>>(res.at("x"));
    last_x_ = vector_x;

    Eigen::Vector7d out;
     for(auto i=0; i<7; i++)
        out(i) = vector_x[i+3*(N_+1)+7];

    return out;
}

pair<bool, vector<Eigen::Vector7d>> MotionPlanner::MPCSolv(const Eigen::Vector7d & current_q){
    // 1. initialize
    for(uint i=0; i<3; i++){
        for(uint j=0; j<3; j++){
            orientation_desire_data_(j+i*3) = ori_desired_(j,i);
        }
    }
    for(uint i=0; i<3; i++){
        position_desire_data_(i) = goal_position_(i);
    }


    for(uint i=0; i<7; i++){
        Q0_data_(i) = current_q(i);
    }
    for(uint i=0; i<7; i++) {
        joint_position_desired_data_(i) = goal_joint_(i);
    }

    if(first_solve_){
        Eigen::Vector7d delta = (goal_joint_ - current_q) / (N_+1);
        for(uint k=0; k<N_+1; k++){
            for(uint i=0; i<7; i++)
                x0_data_(i + k*7) = current_q(i) + delta(i)*k;
        }

        for(uint k=0; k<N_; k++){
            for(uint i=0; i<7; i++)
                x0_data_(7*(N_+1) + i + k*7) = 0;
        }
    }
    else {
        for (uint i = 0; i < 7; i++)
            x0_data_(i) = current_q(i);
        for (uint i = 7; i < 7 * (N_); i++)
            x0_data_(i) = last_x_[i + 7];
        for (uint i = 7*N_; i < 7 * (N_+1); i++)
            x0_data_(i) = last_x_[i];

        for(uint i=7*(N_+1); i< 7*(N_+1)+7*(N_-1); i++)
            x0_data_(i) = last_x_[i+7];
        for (uint i = 7*(N_+1)+7*(N_-1); i < 7*(N_+1)+7*(N_); i++)
            x0_data_(i) = last_x_[i];
    }
    // 2. solve nmpc
    arg_["x0"] = x0_data_;
    arg_["p"]  = vertcat(joint_position_desired_data_,  orientation_desire_data_, Q0_data_, position_desire_data_);
//    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    DMDict res = solver_(arg_);
    bool feasible =  solver_.stats().at("success");
    if(feasible==false){
        cout << "infeasible JM!!!!!" << endl;
    }
    //    cout << res.at("f") << endl
#if 0
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double tim = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    pair<DM, double> info = make_pair(res.at("g"), tim);
    solver_info.push_back(info);
#endif

     if(first_solve_)
        first_solve_ = false;

    // 3. return Q1
    vector<double> vector_x = static_cast<std::vector<double>>(res.at("x"));
    last_x_ = vector_x;
    vector<Eigen::Vector7d> out(N_);
    for(auto i=1; i<N_+1; i++)
        for(auto j=0; j<7; j++)
            out[i-1](j) = vector_x[j+i*7];

    return make_pair(feasible, out);
}



//******************** up-level function to call solve function *******************************
Eigen::Vector7d MotionPlanner::followTrajectoryOptim(Panda& robot){
    double x_d = 0.428;
    double y_d = 0.425;
    double z_d = 0.575;
    double time_max = 40.0;
    double radius = 0.2;

    std::array<double, 16> pose_goal = {0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, x_d, y_d, z_d, 1.0};
//    pose_goal = {1, 0.0, 0, 0.0, 0.0, 1.0, 0.0, 0.0, 0, 0.0, 1, 0.0, x_d, y_d, z_d, 1.0};
    pose_goal = {-1, 0, -0, 0,  0, 1, 0, 0,  0, 0, -1, 0,  x_d, y_d, z_d, 1.0};  //y
    pose_goal = { 1, 0,  0, 0,  0,-1, 0, 0,  0, 0, -1, 0,  x_d, y_d, z_d, 1.0};  //x

    auto time_main_ = ros::Time::now().toSec();
    if(time_main_>=10 && time_main_<=50){
        double angle = M_PI * (1 - std::cos(M_PI / time_max * (time_main_-10)));
        double delta_x = radius * std::sin(angle) * std::cos(angle);
        double delta_y = radius * std::sin(angle);
        pose_goal[12]=x_d-delta_x;
        pose_goal[13]=y_d-delta_y;
    }
    else if(time_main_>=50){
        pose_goal[12]=x_d;
        pose_goal[13]=y_d;
    }
//    pose_goal = {1,0,0,0, 0,1,0,0, 0,0,1,0 , 0.5, 0.1, 0.5, 1};
//    pose_goal = {0,1,0,0, -1,0,0,0, 0,0,1,0 , 0.8, 0, 0.6, 1};

    Eigen::Affine3d goal_transform(Eigen::Matrix4d::Map(pose_goal.data()));
    Eigen::Vector7d q_desired = iKSolv(goal_transform, jointPos_);

    if(time_main_>=10 && time_main_<=time_max+10){
        robot.setJoints(jointPos_,Eigen::Vector7d::Zero());
        auto cur_transform = robot.fkEE();
        Eigen::Vector3d cur_position(cur_transform.translation());

        ee_data.push_back({cur_position(0), cur_position(1), cur_position(2)}); // record data
    }
//    ee_desired_data.push_back({pose_goal[12], pose_goal[13], pose_goal[14]});

#if 1
    if(time_main_>=10 && time_main_<=time_max+10){
        robot.setJoints(q_desired, Eigen::Vector7d::Zero());
        Eigen::Affine3d desir_transform = robot.fkEE();
        Eigen::Vector3d desir_position(desir_transform.translation());

        ee_calculated_desired_data.push_back({desir_position(0), desir_position(1), desir_position(2)}); // record data
    }
#endif
    return q_desired;
}

Eigen::Vector7d MotionPlanner::generateTrajectory(Panda& robot){
    double x_d = 0.428;
    double y_d = 0.425;
    double z_d = 0.575;
    double time_max = 20.0;
    double radius = 0.2;

//    std::array<double, 16> pose_goal = {0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, x_d, y_d, z_d, 1.0};
    std::array<double, 16> pose_goal = {1, 0, -0, 0,   0, -1, 0.0, 0.0,   0, 0.0, -1, 0,  x_d, y_d, z_d, 1.0};


    auto time_main_ = ros::Time::now().toSec();
    if(time_main_<=10  ){
        pose_goal[12]=x_d-0.8;
        pose_goal[13]=y_d;
    }
    else  {
        pose_goal[12]=x_d+0.8;
        pose_goal[13]=y_d;
    }

    pose_goal[12]=0.088;
    pose_goal[13]=0;
    pose_goal[14]= 0.8676;

    Eigen::Affine3d goal_transform(Eigen::Matrix4d::Map(pose_goal.data()));
    Eigen::Vector7d q_desired = MPCSolv(goal_transform, jointPos_);

    if(time_main_>=10 && time_main_<=time_max+10){
        robot.setJoints(jointPos_,Eigen::Vector7d::Zero());
        auto cur_transform = robot.fkEE();
        Eigen::Vector3d cur_position(cur_transform.translation());

        ee_data.push_back({cur_position(0), cur_position(1), cur_position(2)}); // record data
    }
//    ee_desired_data.push_back({pose_goal[12], pose_goal[13], pose_goal[14]});

#if 1
    if(time_main_>=10 && time_main_<=time_max+10){
        robot.setJoints(q_desired, Eigen::Vector7d::Zero());
        Eigen::Affine3d desir_transform = robot.fkEE();
        Eigen::Vector3d desir_position(desir_transform.translation());

        ee_calculated_desired_data.push_back({desir_position(0), desir_position(1), desir_position(2)}); // record data
    }
#endif
     return q_desired;
}

pair<bool, vector<Eigen::Vector7d>> MotionPlanner::generateJointTrajectory(Panda& robot){
    auto res = MPCSolv(jointPos_);

    robot.setJoints(jointPos_,Eigen::Vector7d::Zero());
    auto cur_transform = robot.fkEE();
    ee_data.push_back({cur_transform.translation()(0), cur_transform.translation()(1), cur_transform.translation()(2)}); // record data

    return res;
}



//******************** build MPC Model helper function *******************************
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

vector<pair<SX, double>> MotionPlanner::forwardKinematicMultiLinks(const SX& Q){
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

    // for collision
    SX T_1 = mtimes(T_J2, A_all4);
    SX T_2 =mtimes(mtimes(T_J4, A_all7), A_all8);

    vector<pair<SX, double>> res;
    res.emplace_back(make_pair(T_1, 0.04));
    res.emplace_back(make_pair(T_2, 0.04));
    res.emplace_back(make_pair(T_J5, 0.04));
    res.emplace_back(make_pair(T_J7, 0.05));
    res.emplace_back(make_pair(T_J9, 0.05));

    return  res;
}

SX MotionPlanner::oriError3(const SX& r, const SX& r2){

    auto r_reshaped2 = reshape(r2,3,3);

    Slice all;
    SX res_temp(3,1);
    SX res(1,1);

    res_temp(0) = mtimes(r(all, 0).T(), r_reshaped2(all, 0 )) - 1;
    res_temp(1) = mtimes(r(all, 1).T(), r_reshaped2(all, 1 )) - 1;
    res_temp(2) = mtimes(r(all, 2).T(), r_reshaped2(all, 2 )) - 1;

    res = pow(res_temp(0),2) + pow(res_temp(1), 2) + pow(res_temp(2),2);
    return res;
//    return res*0.5;

}

void MotionPlanner::copy(SX & mx, const vector<vector<SX>>& data){
    for(uint i=0; i<4; i++){
        for(uint j=0; j<4; j++){
            mx(i,j) = data[i][j];
        }
    }
}



//******************** parameters interface of MPC Model *******************************
Eigen::Vector3d MotionPlanner::setGoal(Panda& robot, std::array<double, 7>& goal){
    //    std::array<double, 7> joint_goal = {1.92, -0.78, 0, -2.35, 0, 1.57, 0.79};
//    std::array<double, 7> joint_goal = {1.34, 0.48, 0.09, -1.51, 0.09, 1.99, 0.95};  //test.bt

    std::array<double, 7> joint_goal = goal;
    goal_joint_ = Eigen::Vector7d::Map(joint_goal.data());

    robot.setJoints(goal_joint_, Eigen::Vector7d::Zero());
    auto goal_transform =  robot.fkEE();

    ori_desired_ = goal_transform.linear();
    goal_position_ = goal_transform.translation();
    return goal_position_;
}

void MotionPlanner::setObstacles(const vector<array<double,4>>& obs) {
    obs_ = obs;
}

void MotionPlanner::nearObsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if(msg->data[0]==0){
        obs_.clear();
     }
    else{
        uint nums = msg->data[0];
        vector<array<double, 4>> receive_obs(nums);
        for(size_t i=0; i<nums; i++){
            receive_obs[i] = {msg->data[1+i*4], msg->data[2+i*4], msg->data[3+i*4], msg->data[4+i*4]};//x,y,z,size
        }
        obs_ = receive_obs;
    }
}



//******************** publish data *******************************
void MotionPlanner::pubTrajectory(const Eigen::Vector7d& trajectory){
    trajectory_msgs::JointTrajectoryPoint j_p;
    trajectory_msgs::JointTrajectory j_traj;

    for(auto i=0; i<7; i++)
        j_p.positions.push_back(trajectory(i));

    j_traj.points.push_back(j_p);
    trajPub_.publish(j_traj);
}



//******************** legacy code *******************************
void MotionPlanner::buildModel() {

    Slice all;
    Slice pos_slice(0,3);
    Slice ori_slice(3,12);
    Slice last_col(3,4);

    N_ = 10;
    // ---- parameter ---------
    orientation_desire_data_ = DM(9,1);
    x0_data_ = DM(17*N_+10, 1);  // 3+7+7  x,y,z,  q, qdot
    position_desire_data_ = DM(3,1);
    position_desire_data_(0)=0.428; position_desire_data_(1)=0.425; position_desire_data_(2)=0.575;
    Q0_data_ = DM(7,1);

    // ---- decision variables ---------
    X_ = SX::sym("X", 3, N_+1); // state trajectory
    Q_ = SX::sym("Q", 7, N_+1); // joint trajectory
    Qdot_ = SX::sym("dQ", 7, N_); // joint trajectory
    double dt = 0.2;
    auto pos = X_(pos_slice,all);

    // ---- parameter variables ---------
    position_desired_    = SX::sym("pos_d",3,1);
    orientation_desired_ = SX::sym("ori_d",9,1);
    feedback_variable_   = SX::sym("feedback",7,1);

    // ---- cost function ---------
    SX f=0;
#if 0
    SX weight = SX(1,7); weight(0)=8; weight(1)=7; weight(2)=6; weight(3)=5; weight(4)=4; weight(5)=3; weight(6)=2;
    for(uint i=0; i<N_+1; i++){
        for(uint j=0; j<7; j++) {

            f += pow((Q_(j, i) - avg_limit(j)) / delta_limit(j), 2);
        }
    }
#endif
    for(uint k=1; k<N_; ++k){
        for(uint i=0; i<3; i++)
            f += pow(pos(i,k) - position_desired_(i), 2);
    }
    for(uint i=0; i<3; i++)
        f += 100*pow(pos(i,N_) - position_desired_(i), 2);

#if 0
    for(uint k=0; k<N_; ++k){
        for(uint i=0; i<7; i++)
            f += pow(Qdot_(i, k), 2)*0.1;
    }
#endif

    SX g(3*(N_+1), 1); SX g_orientation(1*(N_), 1);  SX g_q_kin(7*(N_), 1); SX g_inital(7,1); SX g_end(3,1);
    // ---- fk constraints --------
    for (int k=0;k<N_+1;++k) {
        SX transform =  forwardKinematic( Q_(all, k));
        g(0+k*3) = pos(0,k) - transform(0, last_col);
        g(1+k*3) = pos(1,k) - transform(1, last_col);
        g(2+k*3) = pos(2,k) - transform(2, last_col);
    }
    // ---- orientation constraints --------
    for (int k=1;k<N_+1;++k) {
        SX transform =  forwardKinematic( Q_(all, k));
        auto ori_error = oriError3(transform(Slice(0,3), Slice(0,3)), orientation_desired_);
        g_orientation(0+(k-1)) = ori_error(0);
    }
    // ---- joint differential kinematics --------
    for (int k=1; k<N_+1; ++k) {
        for(uint i=0; i<7; i++){
            g_q_kin(i+7*(k-1)) = Q_(i,k) - Q_(i,k-1) - Qdot_(i,k-1)*dt;
        }
    }
    // ---- initial value constraints --------
    for(uint i=0; i<7; i++) {
        g_inital(i) = Q_(i, 0) - feedback_variable_(i);
    }
    // ---
    g_end(0) = pos(0,N_) - position_desired_(0);
    g_end(1) = pos(1,N_) - position_desired_(1);
    g_end(2) = pos(2,N_) - position_desired_(2);

    // ----bounds--------
    vector<double> lbx_vec, ubx_vec;
    for(auto i=0; i<(N_+1)*3; i++){
        lbx_vec.push_back(-2);  // X range  TODO modify
        ubx_vec.push_back(2);
    }
    for(auto i=0; i<N_+1; i++){
        lbx_vec.insert(lbx_vec.end(), min_limit.begin(), min_limit.end());  // joint range
        ubx_vec.insert(ubx_vec.end(), max_limit.begin(), max_limit.end());
    }
    for(auto i=0; i<N_; i++){
        lbx_vec.insert(lbx_vec.end(), dq_min_limit.begin(), dq_min_limit.end());  // joint speed range
        ubx_vec.insert(ubx_vec.end(), dq_max_limit.begin(), dq_max_limit.end());
    }

    SXDict nlp = {{"x", SX::vertcat({vec(X_), vec(Q_), vec(Qdot_)})},
                  {"f", f},
                  {"g", vertcat(g, g_orientation, g_q_kin, g_inital)}, //
//                  {"g", vertcat(g, g_q_kin, g_inital, g_end)},
                  {"p", vertcat(position_desired_, orientation_desired_, feedback_variable_) } };

    // ----arg--------
    arg_["lbx"] =lbx_vec; arg_["ubx"] =ubx_vec;
    arg_["lbg"] = arg_["ubg"] = 0;

    Dict opts;
    opts["ipopt.print_level"] = 0;

    solver_ = nlpsol("solver", "ipopt", nlp, opts);
//    solver_ = nlpsol("solver", "worhp", nlp, opts);

    // colllision avoidance
}

SX MotionPlanner::oriError(const SX& r, const SX& r2){

//    auto r_reshaped = reshape(r,3,3);
    auto r_reshaped2 = reshape(r2,3,3);


    SX error = mtimes(r, r_reshaped2.T());
    SX res(3,1);
    res(0) =  error(2,1) - error(1,2);
    res(1) =  error(0,2) - error(2,0);
    res(2) =  error(1,0) - error(0,1);

    return res;
//    return res*0.5;

}

SX MotionPlanner::oriError2(const SX& r, const SX& r2){

    auto r_reshaped2 = reshape(r2,3,3);

    Slice all;
    SX res(3,1);

    res(0) = mtimes(r(all, 0).T(), r_reshaped2(all, 0 )) - 1;
    res(1) = mtimes(r(all, 1).T(), r_reshaped2(all, 1 )) - 1;
    res(2) = mtimes(r(all, 2).T(), r_reshaped2(all, 2 )) - 1;

    return res;
//    return res*0.5;

}

SX MotionPlanner::eluerError(const SX& r, const SX& r2){


    auto r2_m = reshape(r2,3,3);
    Slice all;

    // convert to euler angle
    SX r_euler(3,1);  SX r_euler2(3,1);
    r_euler(0) = atan2( r(1,0), r(0,0));  // z
    r_euler(1) = atan2( -r(2,0), pow(pow(r(0,0),2)+pow(r(1,0),2) ,0.5));  // y
    r_euler(2) = atan2( r(2,1), r(2,2));  // x

    r_euler2(0) = atan2( r2_m(1,0), r2_m(0,0));  // z
    r_euler2(1) = atan2( -r2_m(2,0), pow(pow(r2_m(0,0),2)+pow(r2_m(1,0),2), 0.5));  // y
    r_euler2(2) = atan2( r2_m(2,1), r2_m(2,2));  // x

    // represent euler error

    SX res(3,1);
    res = r_euler - r_euler2;

    return res;

}
#if 1
SX MotionPlanner::quaternionError(const SX& r, const SX& r2){
    auto r_reshaped = reshape(r,3,3);
    auto ori_desired = reshape(r2,3,3);
    Slice all;

    // convert to quaternion
//    cout << r << endl;
    SX quat_desired(4,1);
    quat_desired(0) = 0.5*pow(1+ori_desired(0,0)+ori_desired(1,1)+ori_desired(2,2), 0.5);
    quat_desired(1) = (-ori_desired(2,1)+ori_desired(1,2)) / (4*quat_desired(0));
    quat_desired(2) = (-ori_desired(0,2)+ori_desired(2,0)) / (4*quat_desired(0));
    quat_desired(3) = (-ori_desired(1,0)+ori_desired(0,1)) / (4*quat_desired(0));

    SX quat_cur_conguate(4,1);
    quat_cur_conguate(0) = 0.5*pow(1+r_reshaped(0,0)+r_reshaped(1,1)+r_reshaped(2,2), 0.5);
    quat_cur_conguate(1) = -(-r_reshaped(2,1)+r_reshaped(1,2)) / (4*quat_cur_conguate(0));
    quat_cur_conguate(2) = -(-r_reshaped(0,2)+r_reshaped(2,0)) / (4*quat_cur_conguate(0));
    quat_cur_conguate(3) = -(-r_reshaped(1,0)+r_reshaped(0,1)) / (4*quat_cur_conguate(0));

    SX quat_cur(4,1);
    quat_cur(0) = 0.5*pow(1+r_reshaped(0,0)+r_reshaped(1,1)+r_reshaped(2,2), 0.5);
    quat_cur(1) = (-r_reshaped(2,1)+r_reshaped(1,2)) / (4*quat_cur_conguate(0));
    quat_cur(2) = (-r_reshaped(0,2)+r_reshaped(2,0)) / (4*quat_cur_conguate(0));
    quat_cur(3) =(-r_reshaped(1,0)+r_reshaped(0,1)) / (4*quat_cur_conguate(0));
#if 0
    // represent  error
    SX quat_error(4,1);
    quat_error(0) = quat_desired(0)*quat_cur_conguate(0) - quat_desired(1)*quat_cur_conguate(1) - quat_desired(2)*quat_cur_conguate(2) - quat_desired(3)*quat_cur_conguate(3);
    quat_error(1) = quat_desired(0)*quat_cur_conguate(1) + quat_desired(1)*quat_cur_conguate(0) + quat_desired(2)*quat_cur_conguate(3) - quat_desired(3)*quat_cur_conguate(2);
    quat_error(2) = quat_desired(0)*quat_cur_conguate(2) - quat_desired(1)*quat_cur_conguate(3) + quat_desired(2)*quat_cur_conguate(0) + quat_desired(3)*quat_cur_conguate(1);
    quat_error(3) =  quat_desired(0)*quat_cur_conguate(3) + quat_desired(1)*quat_cur_conguate(2) - quat_desired(2)*quat_cur_conguate(1) + quat_desired(3)*quat_cur_conguate(0);

    SX res(3,1);
    res(0) = - quat_error(0) *quat_error(1);
    res(1) = - quat_error(0) *quat_error(2);
    res(2) = - quat_error(0) *quat_error(3);
#endif
    SX res(1,1);
    res = quat_desired(0)*quat_cur(0)+ quat_desired(1)*quat_cur(1)+ quat_desired(2)*quat_cur(2)+ quat_desired(3)*quat_cur(3) - 1;
    return res;

}
#endif







//home
//    j_p.positions[0]=1.57019; j_p.positions[1]=-1.00367; j_p.positions[2]=-0.00379918;
//    j_p.positions[3]=-1.81187;j_p.positions[4]= 0.00660794; j_p.positions[5]= 0.807798; j_p.positions[6]=0.866066;

//for (auto&& s : res) {
//std::cout << std::setw(10) << s.first << ": " << std::vector<double>(s.second) << std::endl;
//}
//    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count() << "[ms]" << std::endl;
