//
// Created by jieming on 05.10.20.
//

#include "Invdyn.h"
Invdyn::Invdyn(int whichRobot){
    // Initialize publishers on the topics /robot1/panda_joint*_controller/command for the joint efforts
    tauPub1 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint1_controller/command", 20);
    tauPub2 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint2_controller/command", 20);
    tauPub3 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint3_controller/command", 20);
    tauPub4 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint4_controller/command", 20);
    tauPub5 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint5_controller/command", 20);
    tauPub6 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint6_controller/command", 20);
    tauPub7 = nh.advertise<std_msgs::Float64>("/robot1/panda_joint7_controller/command", 20);
    sensorSub = nh.subscribe("/robot1/joint_states", 1, &Invdyn::jointStatesCallback, this);

    // Initialize the variables for thr AIC
    Invdyn::initVariables();
}

Invdyn::~Invdyn() {}

void Invdyn::initVariables(){
    // Support variable
    dataReceived = 0;

    // Initialize control actions
    u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Integration step
    h = 0.001;
}

void Invdyn::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for( int i = 0; i < 7; i++ ) {
        jointPos(i) = msg->position[i];
        jointVel(i) = msg->velocity[i];
    }
    if (dataReceived == 0){
        dataReceived = 1;
    }
}

void Invdyn::computeActions(){

    tau1.data = u(0); tau2.data = u(1); tau3.data = u(2); tau4.data = u(3);
    tau5.data = u(4); tau6.data = u(5); tau7.data = u(6);

    tauPub1.publish(tau1); tauPub2.publish(tau2); tauPub3.publish(tau3);
    tauPub4.publish(tau4); tauPub5.publish(tau5); tauPub6.publish(tau6);
    tauPub7.publish(tau7);
}

void Invdyn::invdynMethod(){

    /* n(q,dq) = C(q,dq)dq + Ddq + g(q) */
    /*[-3*self.dq1-1*self.q1+3.14],[-3*self.dq2-1*self.q2-1.57*1]*/
    Eigen::Vector7d error;
    for(auto i=0; i<7; i++){
        error(i) = -100*jointVel(i) - 100*jointPos(i) + jointPos_d(i)*100;
    }
    u= MassMatrix(jointPos)*error + CoriolisMatrix(jointPos, jointVel)*jointVel   + GravityVector(jointPos); //+ Friction(jointVel)

    computeActions();
}

int Invdyn::dataReady(){
    return dataReceived==1;
}

void Invdyn::setGoal(std::vector<double> desiredPos){
    for(int i=0; i<desiredPos.size(); i++){
        jointPos_d(i) = desiredPos[i];
    }
}