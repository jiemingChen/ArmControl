//
// Created by jieming on 05.10.20.
//
#include "common.h"
#include "cartesian_impedance.h"
#include "Panda.h"
#include "motion_planner.h"
#include "visualize.h"
//#include "common_func.h"
//#include "Invdyn.h"


void torqueThread(Controller&, Panda&);

int main(int argc, char **argv){
    ros::init(argc, argv, "cartesian_impedance_controller_node");
    Panda robot;
    Controller controller;

//    controller.setGoal(robot);

    torqueThread(controller, robot);


}

void torqueThread(Controller& controller, Panda& robot){
    ros::Rate rate(100);
//    controller.setGoal(robot);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        if (!controller.dataReady())
            continue;
        controller.jointPDControl(robot);
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