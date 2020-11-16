//
// Created by jieming on 05.10.20.
//
#include "common.h"
 #include "Panda.h"
#include "motion_planner.h"
#include "visualize.h"


vector<array<double, 3>> ee_data;
vector<array<double, 3>> ee_desired_data;
vector<array<double, 3>> ee_calculated_desired_data;
vector<pair<DM, double>> solver_info;


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
    ofstream output_file("/home/jieming/catkin_ws/data/solverinfo/1.txt");

    for (uint i = 0; i < solver_info.size(); i++){
        double time = solver_info[i].second;
//        vector<double> vector_x = static_cast<std::vector<double>>(solver_info[i].first);
        output_file << time << "  " ;
//        for (uint j = 0; j < vector_x.size(); j++)
//            output_file << vector_x[j] << "  " ;
        output_file << "\n";
    }
}

void planningThread(MotionPlanner&, Panda&, Visual& );

#if 1
int main(int argc, char **argv){
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    Panda robot;
    Visual visual(nh);
    MotionPlanner planner(nh);

    planningThread(planner, robot, visual);

//    write_ee_data();
}
#endif

void planningThread(MotionPlanner& planner,  Panda& robot, Visual& visual){
    ros::Rate rate(100);
    double i=0;
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        auto cal_q= planner.followTrajectoryOptim(robot);
        planner.pubTrajectory(cal_q);
//        planner.generateTrajectory(robot);
        visual.pubPath();

    }

}


