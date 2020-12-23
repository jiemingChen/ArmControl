//
// Created by jieming on 20.10.20.
//

#include "cartesian_impedance.h"


Controller::Controller(){
    stiffness = MatrixXd (6,6);
    damping = MatrixXd (6,6);

    stiffness.setZero();
    stiffness.topLeftCorner(3,3) << translational_stiffness*MatrixXd::Identity(3,3);
    stiffness.bottomRightCorner(3,3) << rotational_stiffness*MatrixXd::Identity(3,3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2 * sqrt(translational_stiffness) * MatrixXd::Identity(3,3);
    damping.bottomRightCorner(3, 3) << 2 * sqrt(rotational_stiffness) * MatrixXd::Identity(3,3);

    dataReceived = 0;
    u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    tauPub1 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint1_controller/command", 20);
    tauPub2 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint2_controller/command", 20);
    tauPub3 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint3_controller/command", 20);
    tauPub4 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint4_controller/command", 20);
    tauPub5 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint5_controller/command", 20);
    tauPub6 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint6_controller/command", 20);
    tauPub7 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint7_controller/command", 20);

    posPubLeft = nh.advertise<std_msgs::Float64>("/robot1/panda_leftfinger_controller/command", 20);
    posPubRight = nh.advertise<std_msgs::Float64>("/robot1/panda_rightfinger_controller/command", 20);

    sensorSub = nh.subscribe("/robot1/joint_states", 1, &Controller::jointStatesCallback, this);
    trjSub_ =  nh.subscribe ("/robot1/jointTrajectory", 4, &Controller::trajCallback, this);

    orientationErrorPub =  nh.advertise<std_msgs::Float64>("/robot1/orientation_error", 20);
    positionErrorPub =  nh.advertise<msg_pkg::TransformationError>("/robot1/position_error", 20);

    //q_init_ << 0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785;  // home joints
    q_init_ << 0, -0.58, 0, -1.76, -0.23, 2.73, 0.79;

     jointPos = q_init_; q_desired_=q_init_;
    jointVel.setZero();

    time_main_ = 0;
    error_pos.pos_error.reserve(3);

    gripperPos.resize(2); gripperVel.resize(2);
}

void Controller::poseError(Panda & robot){
    cur_transform_ = robot.fkEE();
    Eigen::Vector3d cur_position(cur_transform_.translation());
    Eigen::Quaterniond cur_orientation(cur_transform_.linear());
     error_.head(3) << position_d_ - cur_position;

    if (orientation_d_.coeffs().dot(cur_orientation.coeffs()) < 0.0)
        cur_orientation.coeffs() << -cur_orientation.coeffs();

    Eigen::Quaterniond error_quaternion(orientation_d_ * cur_orientation.inverse() );

    // compute "orientation error"
#if 0
    error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error_.tail(3) << cur_transform_.linear() * error_.tail(3);
#endif
    // convert to axis angle
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    // compute "orientation error"
    error_.tail(3) <<   error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

    //  auto temp =  error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
    //  cout << error_(3)-temp(0) << " " << error_(4)-temp(1) << " " << error_(5)-temp(2) << endl;
    //  error.tail(3) << 0,0,0;
}

void Controller::cartesianPDControl(Panda & robot){
    // get state variables
    robot.setJoints(jointPos, jointVel);
    robot.dyn();
    Eigen::Matrix<double, 6, 7> jacobian = robot.jacobian();
    compensation_ =  robot.gravityTorque() + robot.coriolis(); //+ Friction(jointVel)

    poseError(robot);

    Eigen::Vector7d tau_task;
    tau_task << jacobian.transpose() * ( stiffness * error_ + damping * (- jacobian * jointVel) );
    u << tau_task + compensation_;

    computeActions();
}

void Controller::jointPDControl(Panda & robot){
    // get state variables
    robot.setJoints(jointPos, jointVel);
    robot.dyn();
    compensation_ =  robot.gravityTorque() + robot.coriolis(); //+ Friction(jointVel)


    Eigen::Vector7d joint_error;
    for(auto i=0; i<7; i++){
        joint_error(i) = (q_desired_(i) - jointPos(i) )*10 - 6*jointVel(i) ;
//        joint_error(i) = (q_desired_(i) - jointPos(i) )*30 - 6*jointVel(i) ;  //test for gripper
    }

    Eigen::Vector7d tau_task;
    tau_task << robot.mass()*joint_error;
    u << tau_task + compensation_;

    computeActions();
}

void Controller::followTrajectory(Panda& robot){
    double x_d = 0.428;
    double y_d = 0.425;
    double z_d = 0.575;
    double time_max = 20.0;
    double radius = 0.2;

    std::array<double, 16> pose_goal = {0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, x_d, y_d, z_d, 1.0};

    time_main_ = ros::Time::now().toSec();
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
    position_d_ = goal_transform.translation();
    orientation_d_ = goal_transform.linear();

#if 0
    if(time_main_>=8){
        cur_transform_ = robot.fkEE();
        Eigen::Vector3d cur_position(cur_transform_.translation());
        ee_data.push_back({cur_position(0), cur_position(1), cur_position(2)}); // record data
    }
    ee_desired_data.push_back({pose_goal[12], pose_goal[13], pose_goal[14]});
#endif
    cartesianPDControl(robot);

}

void Controller::followTrajectoryIK(Panda& robot){
    double x_d = 0.428;
    double y_d = 0.425;
    double z_d = 0.575;
    double time_max = 20.0;
    double radius = 0.2;

    std::array<double, 16> pose_goal = {0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, x_d, y_d, z_d, 1.0};

    time_main_ = ros::Time::now().toSec();
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

    robot.setJoints(jointPos, jointVel);
    Eigen::Vector7d q_desired = robot.iK(goal_transform);
//    cout << q_desired << endl;
//    jointPDControl(robot, q_desired);


}

void Controller::setGoal(Panda & robot){
    Eigen::Matrix4d desired;
    desired << 0, 0, 1, 0.428,
               0, 1, 0, 0.425,
               -1,0, 0, 0.575,
               0, 0, 0, 1;

//     Eigen::Affine3d desired_transform(Eigen::Matrix4d::Map(desired.data()));


//    robot.setJoints(q_init_, Eigen::Vector7d::Zero());
    Eigen::Affine3d desired_transform = robot.fkEE();


    position_d_ = desired_transform.translation();
    orientation_d_ = desired_transform.linear();
#if 0
    Eigen::AngleAxisd orientation_d_aa(orientation_d);
    cout << "position_d" << position_d.transpose() << endl;
    cout << "orientation_d " << "angle is: " << orientation_d_aa.angle() * (180 / M_PI)
         << " axis is: " << orientation_d_aa.axis().transpose() << endl;
#endif
    Eigen::Vector3d orientation_d_eular = orientation_d_.matrix().eulerAngles(2,1,0);
    cout << "position_d" << position_d_.transpose() << endl;
    cout << "yaw(z) pitch(y) roll(x) = " << orientation_d_eular.transpose() /M_PI *180 << endl;

}

void Controller::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){

    for( int i = 0; i < 2; i++ ) {
        gripperPos[i] = msg->position[i];
        gripperVel[i] = msg->velocity[i];
    }
    for( int i = 2; i < 9; i++ ) {
        jointPos(i-2) = msg->position[i];
        jointVel(i-2) = msg->velocity[i];
    }
    if (dataReceived == 0){
        dataReceived = 1;
    }
}

void Controller::trajCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg){
    auto points = msg->points;

    q_desired_ = Eigen::Map<Eigen::Vector7d>(points[0].positions.data());
}

void Controller::pubError(){
     error_ori.data = error_.tail(3).norm();

     error_pos.pos_error[0] = error_[0];
     error_pos.pos_error[1] = error_[1];
     error_pos.pos_error[2] = error_[2];

    positionErrorPub.publish(error_pos);
     orientationErrorPub.publish(error_ori);
}

bool Controller::dataReady(){
    return dataReceived==1;
}

void Controller::computeActions(){

    tau1.data = u(0); tau2.data = u(1); tau3.data = u(2); tau4.data = u(3);
    tau5.data = u(4); tau6.data = u(5); tau7.data = u(6);

    tauPub1.publish(tau1); tauPub2.publish(tau2); tauPub3.publish(tau3);
    tauPub4.publish(tau4); tauPub5.publish(tau5); tauPub6.publish(tau6);
    tauPub7.publish(tau7);

    pos_left_gripper.data = gripperPos[0] *3;
    pos_right_gripper.data = gripperPos[1] *3;

    posPubLeft.publish(pos_left_gripper);
    posPubRight.publish(pos_right_gripper);
}

void Controller::nullSpaceControl(Panda& robot){
    robot.setJoints(jointPos, jointVel);
    robot.dyn();
    Eigen::Matrix<double, 6, 7> jacobian = robot.jacobian();
    Eigen::Matrix<double, 7, 7> mass = robot.mass();
    compensation_ =  robot.gravityTorque() + robot.coriolis();
}

#if 0
void CartesianImpedance::cartesian_control(FRANKA & robot){
    // get state variables
    std::array<double, 42> jacobian_array = robot.fn_franka_Jacobian( jointPos, Eigen::Matrix4d::Identity() ).J_all[12];

//    std::array<double, 7> G_array = robot.fn_franka_Jacobian( jointPos, Eigen::Matrix4d::Identity() ).G;
//    Eigen::Map<Eigen::Matrix<double, 7, 1>> G(G_array.data());

    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    Eigen::Matrix<double, 4, 4> current_ee = robot.fn_franka_AT( jointPos, Eigen::Matrix4d::Identity() ).T_J[7]; //6
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_ee.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error
    Eigen::Matrix<double, 6, 1> error;
//        cout << position_d << endl;

    error.head(3) << position - position_d;
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
        orientation.coeffs() << -orientation.coeffs();

//    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    Eigen::Quaterniond error_quaternion(orientation.conjugate() * orientation_d);

    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << -transform.linear() * error.tail(3);
//    error.tail(3) << 0,0,0;

    Eigen::Vector7d compens =  robot.coriolis_matrix(jointPos, jointVel)*jointVel + robot.gravity_vector(jointPos) ; //+ Friction(jointVel)
//    Eigen::Vector7d compens =  robot.gravity_vector(jointPos)  ;

    Eigen::VectorXd tau_task(7); //, tau_d(7);

    // Spring damper system with damping ratio=1
    tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * jointVel));
//    tau_task << jacobian.transpose() * (-stiffness * error );

    u << tau_task + compens;
//    u << compens;

    computeActions();
}
#endif

