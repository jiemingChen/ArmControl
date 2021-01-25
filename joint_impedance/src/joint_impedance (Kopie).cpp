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
#include "sensor_msgs/JointState.h"

using namespace std;

/*GLOBAL VARIABLE*/
const double object_grasp_width = 0.02;
const std::string ROBOT_IP("172.16.0.2");

const std::array<double, 7> KP = {{60.0, 60.0, 60.0, 60.0, 40.0, 20.0, 10.0}};
const std::array<double, 7> KD = {{10.0, 10.0, 10.0, 8.0, 5.0, 2.0, 2.0}};

std::array<double, 7> Q_INIT, Q_DESIR, QD_DESIR;

/*callback used for receive q_desired*/        
void trajCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg){
    auto points = msg->points;
    //Q_DESIR     = points[0].positions;
    cout << "i am in callback " << endl;
}

int main(int argc, char** argv) {
try{
    ros::init(argc, argv, "lowlevel_node");
    ros::NodeHandle nh;
    nh.subscribe("robot1/jointTrajectory", 4, trajCallback);
    ros::Publisher jointsPub, fkPub;
    jointsPub = nh.advertise<sensor_msgs::JointState>("/robot1/joint_states", 2);
    fkPub     = nh.advertise<std_msgs::Float64MultiArray>("/robot1/fk", 2);

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
#if 0
    Q_INIT = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.2, q_init);
    gripper.stop(); // to make sure homing executes successfully
#endif
    cout << "gripper"<<endl;
    gripper.homing();
    sleep(2); /*2sec wait for homing operation*/
    double width_gripper_open = gripper.readOnce().max_width;
    if (width_gripper_open < 0.079){
        ROS_ERROR("Failed initialising the gripper! Make sure nothing is between the fingers when homing.");
    }
    //else
    //    robot.control(motion_generator);
        

    std::thread send_data_thread;
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
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> joint_PD_control_callback = [&model, q_cmd_now](const franka::RobotState& robot_state, franka::Duration dt)->franka::Torques
    {
            std::array<double, 7> coriolis = model.coriolis(robot_state);
                    
            std::array<double, 7> tau_jpd;
            for (int i = 0; i < 7; i++)
                tau_jpd[i] = KP[i]*(q_cmd_now[i] - robot_state.q[i]) - KD[i]*robot_state.dq[i] + coriolis[i];
            cout << "i am running fine" << endl;
           return tau_jpd;
    };

    send_data_thread = std::thread([&]()
    {
        franka::RobotState   state; 
        franka::GripperState gripper_state;
        std::chrono::steady_clock::time_point wakeup_timer = std::chrono::steady_clock::now();
        while (ros::ok()){
            wakeup_timer += std::chrono::milliseconds(50);
            std::this_thread::sleep_until(wakeup_timer);

        	state         = robot.readOnce(); 
        	gripper_state = gripper.readOnce(); 
        
        	for(size_t i=0; i<7; i++){
                msg.position[i] = state.q[i];
                msg.velocity[i] = state.dq[i];
            }
            for(size_t i=0; i<2; i++){
                msg.position[i+7] = gripper_state.width / 2.0;
                msg.velocity[i+7] = 0;
            }
            msg.header.stamp = ros::Time::now();
            jointsPub.publish(msg);
        
            cur_trans.data.insert(cur_trans.data.begin(), state.O_T_EE.begin(), state.O_T_EE.end());
            fkPub.publish(cur_trans); 
            cur_trans.data.clear();

            //ros::spinOnce();
            //loop_rate.sleep();
        }
        return;
    });
   std::cout << "send_data thread created." << std::endl;
   std::this_thread::sleep_for(std::chrono::milliseconds(200)); 

    /*run it*/
    robot.control(joint_PD_control_callback);
    
    
    if (send_data_thread.joinable()){
        send_data_thread.join();
        std::cout << "[send_data] Thread terminated." << std::endl;
    }
}
catch(exception& e){
    cerr << "error" << endl;
}

    std::cout << "Quitted robot control loop." << std::endl;
    //franka::MotionFinished(tau_cmd_rate_limited);
    std::cout << "Program ended." << std::endl;
    return 0;
}





// define thread for receive q_d, qdot_d
//std::thread receive_thread([&Q_DSIR, &nh]() {
//});
