//
// Created by jieming on 29.12.20.
//
#include "utils.h"

void showTF(MotionPlanner& planner, ros::NodeHandle& n_){
    ros::Rate rate(100);
    ros::Publisher publisher;
    publisher = n_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("myTF",1);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        auto msg = planner.showCollisionPoints();
        publisher.publish(msg);
    }
}

void load_data(string filename){
    Initial_joints = {0, -0.785,  0.0, -2.356, 0.0,  1.57, 0.785};  // home joints
//    Goal_joints = {1.27, 0.48, 0.28, -1.37, 0.04, 1.91, 0.39};  // side wall
    Goal_joints    = {0,   0.43,    0, -1.76, -0.23, 2.03,  1.11}; // down wall
    Goal_joints    = {0,   0.97,    0, -0.98,  0.00, 3.49,  0.79}; // down wall  and orientation constraint
}

void write_ee_data(){
    std::cout << "writing ee into file..." << std::endl;
    ofstream output_file_q("/home/jieming/catkin_ws/data/solverinfo/data_ee_optim.txt");
    ofstream output_file_desir("/home/jieming/catkin_ws/data/data_ee_desired_optim.txt");
    ofstream output_file("/home/jieming/catkin_ws/data/solverinfo/data_ee_calculated.txt");

//    for (uint i = 0; i < ee_data.size(); i++){
//        for (uint j = 0; j < 2; j++)
//            output_file_q << ee_data[i][j] << "  " ;
//        output_file_q << "\n";
//    }
//    for (uint i = 0; i < ee_desired_data.size(); i++){
//        for (uint j = 0; j < 2; j++)
//            output_file_desir << ee_desired_data[i][j] << "  " ;
//        output_file_desir << "\n";
//    }

    for (uint i = 0; i < ee_calculated_desired_data.size(); i++){
        for (uint j = 0; j < 2; j++)
            output_file << ee_calculated_desired_data[i][j] << "  " ;
        output_file << "\n";
    }

}

void write_solver_data(){
    std::cout << "writing solver info into file..." << std::endl;
    ofstream output_file("/home/jieming/catkin_ws/data/solverinfo/time_obs_closemidterm.txt");

    for (uint i = 0; i < solver_info.size(); i++){
        double time = solver_info[i].second;
//        vector<double> vector_x = static_cast<std::vector<double>>(solver_info[i].first);
        output_file << time << "  " ;
//        for (uint j = 0; j < vector_x.size(); j++)
//            output_file << vector_x[j] << "  " ;
        output_file << "\n";
    }
}