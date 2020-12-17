//
// Created by jieming on 06.12.20.
//


#include "CollisionCheck.h"
using std::cout;
using std::endl;



int main(int argc, char **argv){
    ros::init(argc, argv, "collisionCheckNode");

    CollisionCheck checker_srv, checker;
    ros::Rate rate(10);

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
        if(checker_srv.isFirstReceiv()){
            cout << "wait for initial" << endl;
            continue;
        }
        if(checker.isFirstReceiv()){
            cout << "wait for initial" << endl;
            continue;
        }

        checker.getCollisions();
        checker.getCollisionsInRange(0.2);
      }

}

