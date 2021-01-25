//
// Created by jieming on 05.10.20.
//
#include "common.h"
#include "Panda.h"
#include "visualize.h"
#include "xxx/param.hpp"

#include "motion_planner.h"
#include "SEARCHER.h"
//-----------------data record----------------
vector<array<double, 3>> ee_data;
vector<array<double, 3>> ee_desired_data;
vector<array<double, 3>> ee_calculated_desired_data;
vector<pair<DM, double>> solver_info;
//-----------------config variable----------------
std::array<double, 7> Initial_joints;
Eigen::Vector3d Goal_position;
std::array<double, 7> Goal_joints;


void write_ee_data();
void write_solver_data();
void load_data(std::string);
void initialze_modules(SEARCHER& searcher, MotionPlanner& planner, Panda& robot);

void planningThread(MotionPlanner&, Panda&, Visual& );
void planningThreadMap(MotionPlanner&, Panda&, Visual&,    vector<array<double,7>>& );
vector<array<double,7>> mapSearchThread(SEARCHER*);


int main(int argc, char **argv){
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    Panda robot;
    SEARCHER searcher;
    MotionPlanner planner(nh);
    Visual visual(nh);

    //1. Initialize start and goal position
    std::string filename = "/home/jieming/catkin_ws/src/panda_simulation/panda_control/config/input.cfg";
    load_data(filename);
    initialze_modules(searcher, planner, robot);

    //2. Initial guess from RRT style method
    vector<array<double,7>> waypoints;
     waypoints = mapSearchThread(&searcher);
    visual.pubWaypoints(waypoints, robot);

    //3. MPC running
    planningThreadMap(planner, robot, visual, waypoints);

    //4. record data
    write_solver_data();
}

vector<array<double,7>> mapSearchThread(SEARCHER* searcher_ptr){
    vector<array<double,7>>waypoints;

    optional< vector<array<double,7>>>  waypoints_opt = searcher_ptr->plan();
    if(waypoints_opt.has_value()){
      waypoints = waypoints_opt.value();
    }
    return waypoints;
}


void planningThreadMap(MotionPlanner& planner,  Panda& robot, Visual& visual,   vector<array<double,7>>& guess){
    ros::Rate rate(100);

//    planner.setWaypoint(guess, robot);
    planner.guess(guess, robot);

    while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();

//          planner.guess(guess, robot);
//            planner.goNext(robot);

            auto rst=  planner.generateJointTrajectory(robot);

            if(rst.first){
                planner.pubTrajectory(rst.second[0]);
                visual.pubPredictPath(rst.second, robot);
            }
            visual.pubPath();
    }
//    std::cout << "Time difference = " << planner.averageTime() << "[ms]" << std::endl;
}




void initialze_modules(SEARCHER& searcher, MotionPlanner& planner, Panda& robot){
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        if(ros::Time::now().toSec()<=10 )
            continue;
        if (planner.receiveFeedback()){
            Initial_joints = planner.getJoints();
            break;
        }
    }
    searcher.setGoal(Goal_joints);
    searcher.setStart(Initial_joints);

    planner.setGoal(robot, Goal_joints);
    //planner.buildModel2();
    planner.buildMPC();
//    planner.buildMPCNormalize();

}

#if 0
void planningThread(MotionPlanner& planner,  Panda& robot, Visual& visual){
    ros::Rate rate(100);

    vector<array<double,4>> obs;
    //    obs ={{0.25, 0.162, 0.67}, {0.2, 0.15, 0.5}};  //!!
    // two wall--- high&low
    obs ={
            {0.34, 0.19,0.47}, {0.30, 0.19, 0.47}, {0.26, 0.19, 0.47}, {0.22, 0.19, 0.47},{0.18, 0.19, 0.47}, {0.14, 0.19,0.47}, {0.10, 0.19, 0.47}, {0.06, 0.19, 0.47},
//            {0.34, 0.19,0.87}, {0.30, 0.19, 0.87}, {0.26, 0.19, 0.87}, {0.22, 0.19, 0.87},{0.18, 0.19, 0.87}, {0.14, 0.19,0.87}, {0.10, 0.19, 0.87}, {0.06, 0.19, 0.87}
    };
    // one wall
    obs ={
            {0.38, 0.19,0.47},  {0.34, 0.19,0.47}, {0.30, 0.19, 0.47}, {0.26, 0.19, 0.47}, {0.22, 0.19, 0.47},{0.18, 0.19, 0.47}, {0.14, 0.19,0.47}, {0.10, 0.19, 0.47}, {0.06, 0.19, 0.47},
            {0.38, 0.19,0.55}, {0.34, 0.19,0.55}, {0.30, 0.19, 0.55}, {0.26, 0.19, 0.55}, {0.22, 0.19, 0.55},{0.18, 0.19, 0.55}, {0.14, 0.19,0.55}, {0.10, 0.19, 0.55}, {0.06, 0.19, 0.55},
    };

    planner.setGoal(robot, Goal_joints);
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

        auto rst=  planner.generateJointTrajectory(robot);
        if(rst.first){
            planner.pubTrajectory(rst.second[0]);
            visual.pubPredictPath(rst.second, robot);
        }
        visual.pubPath();
    }
}
#endif

void load_data(string filename){
    Initial_joints = {0, -0.785,  0.0, -2.356, 0.0,  1.57, 0.785};  // home joints
//    Goal_joints = {1.27, 0.48, 0.28, -1.37, 0.04, 1.91, 0.39};  // side wall
    Goal_joints    = {0,   0.43,    0, -1.76, -0.23, 2.03,  1.11}; // down wall
    Goal_joints    = {0,   0.97,    0, -0.98,  0.00, 3.49,  0.79}; // down wall  and orientation constraint

#if 0
    param::parameter param(filename);
    if (!param) {
        std::cerr << "Could not find file " << filename << std::endl;
        std::abort();
    }
    param.get<bool>("bool");
#endif
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

