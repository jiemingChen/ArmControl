//
// Created by jieming on 05.10.20.
//
#include <SEARCHERCARTESIAN.h>
#include "common.h"
#include "Panda.h"
#include "visualize.h"
#include "param.hpp"

#include "motion_planner.h"
#include "SEARCHER.h"
#include "SEARCHERCARTESIAN.h"

//-----------------data record----------------
vector<array<double, 3>> ee_data;
vector<array<double, 3>> ee_desired_data;
vector<array<double, 3>> ee_calculated_desired_data;
vector<pair<DM, double>> solver_info;
//-----------------config variable----------------
std::array<double, 7> Initial_joints;
std::array<double, 3>  Initial_position;
std::array<double, 3> Goal_position;

std::array<double, 7> Goal_joints;


void write_ee_data();
void write_solver_data();
void load_data(std::string);
//void initialze_modules(SEARCHER& searcher, MotionPlanner& planner, Panda& robot);
void initialze_modules(SEARCHER_CARTESIAN& searcher, MotionPlanner& planner, Panda& robot);

void planningThread(MotionPlanner&, Panda&, Visual& );
void planningThreadMap(MotionPlanner&, Panda&, Visual& );
//vector<array<double,7>> mapSearchThread(SEARCHER*);
vector<array<double,3>> mapSearchThread(SEARCHER_CARTESIAN*);


int main(int argc, char **argv){
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    Panda robot;
//    SEARCHER searcher();
    SEARCHER_CARTESIAN searcher(&robot);

    MotionPlanner planner(nh);
    Visual visual(nh);

    std::string filename = "/home/jieming/catkin_ws/src/panda_simulation/panda_control/config/input.cfg";

    load_data(filename);
    initialze_modules(searcher, planner, robot);
    ros::Rate rate(4);
    for(auto i=0; i<40; i++){
        auto waypoints = mapSearchThread(&searcher);
        visual.pubWaypoints(waypoints, robot);
        rate.sleep();
    }


//    planningThreadMap(planner, robot, visual);

 }

vector<array<double,3>> mapSearchThread(SEARCHER_CARTESIAN* searcher_ptr){
    vector<array<double,3>>waypoints;

    optional< vector<array<double,3>>>  waypoints_opt = searcher_ptr->plan();
    if(waypoints_opt.has_value()){
      waypoints = waypoints_opt.value();
    }
    return waypoints;
}


void planningThreadMap(MotionPlanner& planner,  Panda& robot, Visual& visual){
    ros::Rate rate(100);

    vector<array<double,3>> obs;

    planner.setObstacles(obs);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

//        auto rst=  planner.generateJointTrajectory(robot);
//        if(rst.first){
//            planner.pubTrajectory(rst.second[0]);
//            visual.pubPredictPath(rst.second, robot);
//        }
//        visual.pubPath();
    }
}

void planningThread(MotionPlanner& planner,  Panda& robot, Visual& visual){
    ros::Rate rate(100);

    vector<array<double,3>> obs;
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

void initialze_modules(SEARCHER_CARTESIAN& searcher, MotionPlanner& planner, Panda& robot){
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

    robot.setJoints(Eigen::Map<Eigen::Vector7d>(Initial_joints.data()), Eigen::Vector7d::Zero());
    searcher.setStart(robot.fkEE());

    robot.setJoints(Eigen::Map<Eigen::Vector7d>(Goal_joints.data()), Eigen::Vector7d::Zero());
    searcher.setGoal(robot.fkEE());

    planner.setGoal(robot, Goal_joints);
    planner.buildModel2(); //TODO varying num of obstacles

//    searcher.robot_->setJoints(Eigen::Map<Eigen::Vector7d>(Initial_joints.data()), Eigen::Vector7d::Zero());
    searcher.set_joints_for_IK(Initial_joints);
}


void load_data(string filename){
    Initial_joints = { 0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785};  // home joints
    Goal_joints = {1.27, 0.48, 0.28, -1.37, 0.04, 1.91, 0.39};
//    Goal_joints = {-0.46, -0.36, 2.88, -0.85, -2.63, 0.55, 0.95};


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

