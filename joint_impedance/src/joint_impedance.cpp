#include <array>
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <exception> 
#include <mutex>
#include <thread>
#include <cmath>
#include <csignal>
#include <cerrno>
#include <sys/stat.h>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include "/home/franka/Projects/libfranka/examples/examples_common.h"
using namespace std;

/*GLOBAL VARIABLE*/
const double object_grasp_width = 0.02;
const std::string ROBOT_IP("172.16.0.2");

const std::array<double, 7> KP = {{60.0, 60.0, 60.0, 60.0, 40.0, 20.0, 10.0}};
const std::array<double, 7> KD = {{10.0, 10.0, 10.0, 8.0, 5.0, 2.0, 2.0}};


int main(int argc, char** argv) {
    // ros initialisation
    ros::init(argc, argv, "lowlevel_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    
    ros::Publisher jointsPub = nh.advertise<sensor_msgs::JointState>("/robot1/joint_states", 2);
    ros::Publisher fkPub     = nh.advertise<std_msgs::Float64MultiArray>("/robot1/fk", 2);
    //ros msg initialisation
    sensor_msgs::JointState msg;
    msg.name.resize(9);
    msg.position.resize(9);
    msg.velocity.resize(9);
    for(size_t i=0; i<7; i++){
        msg.name[i]     = "panda_joint" + to_string(i+1);
    }
    for(size_t i=0; i<2; i++){
        msg.name[i+7]     = "panda_finger_joint" + to_string(i+1);
    }
    std_msgs::Float64MultiArray cur_trans;
    cur_trans.data.resize(16);
    //
    std::thread recev_cmd;
    bool start_running = false;
try{
    //franka initialisation
    franka::Robot robot(ROBOT_IP);
    franka::Gripper gripper(ROBOT_IP);
        
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        
    /* set stiffness frame (object relative to EE)
     Set to EE instead of the object so that the end effector force can be directly read*/
    robot.setK({{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}});
        
    /*initial pose*/
#if 1
    ROS_INFO("go back to home position");
    std::array<double,7> q_init;
    //q_init = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    //q_init = {{ 0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785 }};
    q_init = {{ 0, -0.58, 0, -1.76, -0.23, 2.73, 0.79 }};
    MotionGenerator motion_generator(0.2, q_init);
    gripper.stop(); // to make sure homing executes successfully
    ROS_INFO("go back to home position finished");
#endif
    double width_gripper_open = gripper.readOnce().max_width;
    if (width_gripper_open < 0.079){
        ROS_ERROR("Failed initialising the gripper! Make sure nothing is between the fingers when homing.");
    }
    else
       robot.control(motion_generator);
        

     /*user confirmation*/
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
        
    /***   main func    ***/
    /*load the kinematics and dynamics model*/
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    
    /*q_cmd used in control callback*/
    std::array<double, 7> q_cmd_now = initial_state.q;
        
    /*define callback for the torque control loop*/
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> joint_PD_control_callback = [&](const franka::RobotState& robot_state, franka::Duration dt)->franka::Torques{
            std::array<double, 7> coriolis = model.coriolis(robot_state);
                    
            std::array<double, 7> tau_jpd;
            for (int i = 0; i < 7; i++){
                if(!start_running){
                    tau_jpd[i] = KP[i]*(robot_state.q[i] - robot_state.q[i]) - KD[i]*robot_state.dq[i] + coriolis[i];
                    ROS_INFO("stop mode");
                }
                else
                    tau_jpd[i] = 0.3*KP[i]*(q_cmd_now[i] - robot_state.q[i]) - KD[i]*robot_state.dq[i] + coriolis[i];
            }
         //   cout << "i am running fine" << endl;

        	for(size_t i=0; i<7; i++){
                msg.position[i] = robot_state.q[i];
                msg.velocity[i] = robot_state.dq[i];
            }

            for(size_t i=0; i<2; i++){
                msg.position[i+7] = width_gripper_open / 2.0;
                msg.velocity[i+7] = 0;
         	    //auto gripper_state = gripper.readOnce(); not work 
            }
            
          	msg.header.stamp = ros::Time::now();
            jointsPub.publish(msg);
        
            cur_trans.data.insert(cur_trans.data.begin(), robot_state.O_T_EE.begin(), robot_state.O_T_EE.end());
            fkPub.publish(cur_trans); 
            cur_trans.data.clear();

            // send torque command
            //cout << q_cmd_now[0] << "torque" << endl;
            return tau_jpd;
    }; 
    
    boost::function<void(const trajectory_msgs::JointTrajectoryConstPtr&)> callback = [&](const trajectory_msgs::JointTrajectoryConstPtr& msg){
        //cout<<"hello i am in callback"<<endl; 
        auto points = msg->points;
        for(size_t i=0; i<7; i++){
            q_cmd_now[i] = points[0].positions[i];
        }
    };
    boost::function<void(const std_msgs::String::ConstPtr&)> startcallback = [&](const std_msgs::String::ConstPtr& msg){
        //cout<<"hello i am in callback"<<endl; 
        if(msg->data == "start")
            start_running = true;
        else
            start_running = false;
    };
    /*run it*/
    ros::Subscriber sub = nh.subscribe("robot1/jointTrajectory", 4, callback);
    ros::Subscriber start_sub = nh.subscribe("robot1/start", 4, startcallback);
    recev_cmd = std::thread([&](){
        while(ros::ok()){
            rate.sleep();
            ros::spinOnce();
        }
    });
    //recev_cmd.join();
    robot.control(joint_PD_control_callback);
    
 
}
catch(const franka::Exception& ex){
	cout << ex.what() << endl;
    cerr << "error" << endl;
}
if(recev_cmd.joinable())
    recev_cmd.join();
std::cout << "Quitted robot control loop." << std::endl;
std::cout << "Program ended." << std::endl;
return 0;
}





// define thread for receive q_d, qdot_d
//std::thread receive_thread([&Q_DSIR, &nh]() {
//});
