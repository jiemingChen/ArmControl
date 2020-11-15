//
// Created by jieming on 12.11.20.
//

#ifndef PANDA_CONTROL_VISUALIZE_H
#define PANDA_CONTROL_VISUALIZE_H

#include "common.h"
#include <nav_msgs/Path.h>
extern vector<array<double, 3>> ee_data;

class Visual {
private:
    ros::Publisher path_pub_;
    ros::NodeHandle nh_;
    nav_msgs::Path path_;
    geometry_msgs::PoseStamped this_pose_stamped;

public:
    Visual(ros::NodeHandle nh){
        nh_ = nh;
        path_pub_ = nh_.advertise<nav_msgs::Path>("rviz_trajectory",1, true);
        path_.header.frame_id="panda1/world";
    }
    void pubPath(){
        if(ee_data.size()==0 )
            return;

        path_.header.stamp=ros::Time().now();
        this_pose_stamped.pose.position.x = (*(ee_data.end()-1))[0];
        this_pose_stamped.pose.position.y = (*(ee_data.end()-1))[1];
        this_pose_stamped.pose.position.z = (*(ee_data.end()-1))[2];

        path_.poses.push_back(this_pose_stamped);
        path_pub_.publish(path_);
    }
};

#endif //PANDA_CONTROL_VISUALIZE_H
