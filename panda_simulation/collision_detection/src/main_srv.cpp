//
// Created by jieming on 06.12.20.
//

#include "ros/ros.h"
#include "CollisionCheck.h"
#include "msg_pkg/Collisioncheck.h"
#include <vector>
using std::cout;
using std::endl;

CollisionCheck* checker_ptr;

bool isCollision(msg_pkg::Collisioncheck::Request &req,
                msg_pkg::Collisioncheck::Response &res){
    res.collide =  false;
    return true;

    checker_ptr->callJointStateCB = false;
    if(!req.start){
	    res.collide = false;
	    return true;
    }
    
    std::vector<double> double_joint_pos(req.joint_pos.size());
    for(size_t i=0; i<req.joint_pos.size(); i++){
        double_joint_pos[i] = req.joint_pos[i];
    }

    res.collide = checker_ptr->checkCollision(double_joint_pos);

    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "collisionCheckNode");
    ros::NodeHandle n;
    ros::Rate rate(10);
    checker_ptr = new CollisionCheck();
    checker_ptr->callJointStateCB = false;

    ros::ServiceServer service = n.advertiseService("collision_check_srv",isCollision);
    ROS_INFO("Ready to check collison.");
    while(ros::ok()){
    	ros::spinOnce();
	    rate.sleep();
    }
    delete  checker_ptr;
    return 0;
}

