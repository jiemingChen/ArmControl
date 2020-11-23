//
// Created by jieming on 13.10.20.
//

#include "NMPC.h"

NMPC::NMPC(int whichRobot){
    // Initialize publishers on the topics /robot1/panda_joint*_controller/command for the joint efforts
    tauPub1 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint1_controller/command", 20);
    tauPub2 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint2_controller/command", 20);
    tauPub3 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint3_controller/command", 20);
    tauPub4 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint4_controller/command", 20);
    tauPub5 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint5_controller/command", 20);
    tauPub6 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint6_controller/command", 20);
    tauPub7 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint7_controller/command", 20);
    sensorSub = nh.subscribe("/robot1/joint_states", 1, &NMPC::jointStatesCallback, this);
    // Initialize the variables for thr AIC
    initVariables();
}

NMPC::~NMPC() {}

void NMPC::initVariables(){
    // Support variable
    dataReceived = 0;

    // Initialize control actions
    u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Integration step
    h = 0.001;
}

void NMPC::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for( int i = 0; i < 7; i++ ) {
        jointPos(i) = msg->position[i];
        jointVel(i) = msg->velocity[i];
    }
    if (dataReceived == 0){
        dataReceived = 1;
    }
}

void NMPC::computeActions(){

    tau1.data = u(0); tau2.data = u(1); tau3.data = u(2); tau4.data = u(3);
    tau5.data = u(4); tau6.data = u(5); tau7.data = u(6);

    tauPub1.publish(tau1); tauPub2.publish(tau2); tauPub3.publish(tau3);
    tauPub4.publish(tau4); tauPub5.publish(tau5); tauPub6.publish(tau6);
    tauPub7.publish(tau7);
}

void NMPC::NMPCController(){
    int N = 10; // number of control intervals
    auto opti = casadi::Opti(); // Optimization problem
    Slice all;
    Slice position_part(0,6);
    auto X = opti.variable(14,N+1);
    auto U = opti.variable(7,N);

    MX obj;
    DM Q(14,14);
    DM R(7,7);
    R(0,0)=0.5; R(1,1)=0.5;
    Q(0,0)=10; Q(1,1)=10; Q(2,2)=10; Q(3,3)=10; Q(4,4)=10; Q(5,5)=10; Q(6,6)=10;
    for(auto i=0; i<N; i++){
        auto x= X(all, i);
        obj += mtimes( mtimes(x.T(), Q), x);
    }

    cout << obj << endl;


    computeActions();
}

int NMPC::dataReady(){
    return dataReceived==1;
}

void NMPC::setGoal(std::vector<double> desiredPos){
    for(int i=0; i<desiredPos.size(); i++){
        jointPos_d(i) = desiredPos[i];
    }
}

//    Eigen::MatrixXd Q(7,7);
//    Q << 10, 0, 0, 0, 0, 0, 0,
//          0,10, 0, 0, 0, 0, 0,
//          0, 0,10, 0, 0, 0, 0,
//          0, 0, 0,10, 0, 0, 0,
//          0, 0, 0, 0,10, 0, 0,
//          0, 0, 0, 0, 0,10, 0,
//          0, 0, 0, 0, 0, 0, 10;