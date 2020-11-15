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

//    write_solver_data();
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
        visual.pubPath();

    }

}


#if 0
int main(int argc, char **argv){
    ros::init(argc, argv, "cartesian_impedance_controller_node");
    Panda robot;
    Controller controller;
    MotionPlanner planner;
    Visual visual;
    ros::Rate rate(1000);

    controller.setGoal(robot);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        if(!controller.dataReady()){
            continue;
        }

        controller.followTrajectoryOptim(robot, planner);
//        controller.followTrajectoryIK(robot);

//        controller.pubError();
        visual.pubPath();
    }
    write_solver_data();
    write_ee_data();
}
#endif

#if 0
int main(int argc, char **argv)
{
    ros::init(argc, argv, "INVDYN_controller_single_node");
    // Variables to regulate the flow (Force to read once every 1ms the sensors)
    int count = 0;
    int cycles = 0;
    int robot = 1;
    // Variable for desired position, set here the goal for the Panda for each joint
    std::vector<double> desiredPos1(7), desiredPos2(7), desiredPos3(7);
    desiredPos1[0] = 0;
    desiredPos1[1] = 0;
    desiredPos1[2] = 0.0;
    desiredPos1[3] = 0;
    desiredPos1[4] = 0.0;
    desiredPos1[5] = 0;
    desiredPos1[6] = 0.0;

//    desiredPos1[0] = 1;
//    desiredPos1[1] = 0.5;
//    desiredPos1[2] = 0.0;
//    desiredPos1[3] = -2;
//    desiredPos1[4] = 0.0;
//    desiredPos1[5] = 2.5;
//    desiredPos1[6] = 0.0;

    desiredPos2[0] = 0.0;
    desiredPos2[1] = 0.2;
    desiredPos2[2] = 0.0;
    desiredPos2[3] = -1.0;
    desiredPos2[4] = 0.0;
    desiredPos2[5] = 1.2;
    desiredPos2[6] = 0.0;

    desiredPos3[0] = -1;
    desiredPos3[1] = 0.5;
    desiredPos3[2] = 0.0;
    desiredPos3[3] = -1.2;
    desiredPos3[4] = 0.0;
    desiredPos3[5] = 1.6;
    desiredPos3[6] = 0;

    // Object of the class AIC which will take care of everything
    Invdyn invdyn_controller(robot);
    // Set desired position in the AIC class
    invdyn_controller.setGoal(desiredPos1);

    // Main loop
    ros::Rate rate(1000);
    while (ros::ok()){
        // Manage all the callbacks and so read sensors
        ros::spinOnce();

        // Skip only first cycle to allow reading the sensory input first
//        if (true){
        if ((count!=0)&&(invdyn_controller.dataReady()==1)){
            invdyn_controller.invdynMethod();
            cycles ++;
            if (cycles == 6000){
                invdyn_controller.setGoal(desiredPos1);
            }

            if (cycles == 12000){
                invdyn_controller.setGoal(desiredPos1);
            }

            if (cycles == 18000){
                invdyn_controller.setGoal(desiredPos1);
            }

            if (cycles == 24000){
                invdyn_controller.setGoal(desiredPos1);
                cycles = 0;
            }
        }
        else
            count ++;

        rate.sleep();
    }
    return 0;
}
#endif

#if 0
int main(int argc, char **argv)
{
    Eigen::AngleAxisd angle_axis(M_PI/4, Eigen::Vector3d ( 0,0,1 ) );
//    cout << angle_axis.angle() << endl;
//    cout << angle_axis.axis() << endl;
//    cout << angle_axis.axis()* angle_axis.angle() << endl;
    Eigen::Quaterniond b(angle_axis);
//    cout << b.x() << b.y() << b.z() << b.w() << endl;
//    cout << b.inverse().x() << b.inverse().y() << b.inverse().z() << b.inverse().w() << endl;
//    cout << b.conjugate().x() << b.conjugate().y() << b.conjugate().z() << b.conjugate().w() << endl;


}
#endif