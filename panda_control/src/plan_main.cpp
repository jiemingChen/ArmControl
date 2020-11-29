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
    write_solver_data();
 }
#endif

void planningThread(MotionPlanner& planner,  Panda& robot, Visual& visual){
    ros::Rate rate(100);

    vector<array<double,3>> obs;
//    obs ={{0.25, 0.15, 0.67}, {0.2, 0.15, 0.5}};
//    obs ={{0.25, 0.162, 0.67}, {0.2, 0.15, 0.5}};  //!!
    obs ={
            {0.34, 0.19,0.47}, {0.30, 0.19, 0.47}, {0.26, 0.19, 0.47}, {0.22, 0.19, 0.47},{0.18, 0.19, 0.47}, {0.14, 0.19,0.47}, {0.10, 0.19, 0.47}, {0.06, 0.19, 0.47},  //};
//           {0.34, 0.19,0.57}, {0.30, 0.19, 0.57}, {0.26, 0.19, 0.57}, {0.22, 0.19, 0.57},{0.18, 0.19, 0.57}, {0.14, 0.19,0.57}, {0.10, 0.19, 0.57}, {0.06, 0.19, 0.57},
           {0.34, 0.19,0.87}, {0.30, 0.19, 0.87}, {0.26, 0.19, 0.87}, {0.22, 0.19, 0.87},{0.18, 0.19, 0.87}, {0.14, 0.19,0.87}, {0.10, 0.19, 0.87}, {0.06, 0.19, 0.87}
    };


    planner.setGoal(robot);
    planner.setObstacles(obs);
    planner.buildModel2();
    visual.setObstacleInfo(obs);
    visual.pubObs();

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        if(!planner.receiveFeedback())
            continue;
        if(ros::Time::now().toSec()<=10 )
            continue;

//        auto cal_q= planner.followTrajectoryOptim(robot);
//        auto cal_q=  planner.generateTrajectory(robot);
        auto rst=  planner.generateJointTrajectory(robot);
        if(rst.first){
            planner.pubTrajectory(rst.second[0]);
            visual.pubPredictPath(rst.second, robot);
        }
        visual.pubPath();
    }

}


