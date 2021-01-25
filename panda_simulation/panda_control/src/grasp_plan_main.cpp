//
// Created by jieming on 28.12.20.
//
#include "common.h"
#include "Panda.h"
#include "visualize.h"
 #include "utils.h"

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

//    showTF(planner, nh);

    initialze_modules(searcher, planner, robot);

     vector<array<double,7>> waypoints;
    waypoints = mapSearchThread(&searcher);
    visual.pubWaypoints(waypoints, robot);

    planningThreadMap(planner, robot, visual, waypoints);
}




void planningThreadMap(MotionPlanner& planner,  Panda& robot, Visual& visual,   vector<array<double,7>>& guess){
    ros::Rate rate(100);

    planner.guess(guess, robot);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        auto rst=  planner.generateJointTrajectory(robot);

        if(rst.first){
            planner.pubTrajectory(rst.second);
            visual.pubPredictPath(rst.second, robot);
        }
//            visual.pubPath();

    }
}

vector<array<double,7>> mapSearchThread(SEARCHER* searcher_ptr){
    vector<array<double,7>>waypoints;

    optional< vector<array<double,7>>>  waypoints_opt = searcher_ptr->plan();
    if(waypoints_opt.has_value()){
        waypoints = waypoints_opt.value();
    }
    return waypoints;
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

    Goal_joints = {0,   0.97,    0, -0.98,  0.00, 3.49,  0.79}; // down wall  and orientation constraint
    //Goal_joints = {1.27, 0.48, 0.28, -1.37, 0.04, 1.91, 0.39};  // side wall

    searcher.setGoal(Goal_joints);
    searcher.setStart(Initial_joints);

    planner.setGoal(robot, Goal_joints);

    planner.buildMPC();
}





#if 0
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
#endif
