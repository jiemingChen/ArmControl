//
// Created by jieming on 12.11.20.
//

#ifndef PANDA_CONTROL_VISUALIZE_H
#define PANDA_CONTROL_VISUALIZE_H

#include "common.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

extern vector<array<double, 3>> ee_data;

class Visual {
private:
    ros::Publisher path_pub_, obs_pub_;
    ros::NodeHandle nh_;
    nav_msgs::Path path_;

    vector<array<double,3>> obs_;
public:
    Visual(ros::NodeHandle& nh){
        nh_ = nh;
        path_pub_ = nh_.advertise<nav_msgs::Path>("actual_trajectory",100);
        obs_pub_ = nh_.advertise<visualization_msgs::Marker>( "obs_marker",  20 );

        path_.header.frame_id="panda1/world";
    }
    ~Visual(){
        path_.poses.clear();
    }
    void setObstacleInfo(const vector<array<double,3>>& obs){
        obs_ = obs;
    }

    void pubPath(){
        if(ee_data.size()==0 )
            return;
//        path_.header.seq = 0;
        path_.header.stamp=ros::Time().now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = (*(ee_data.end()-1))[0];
        this_pose_stamped.pose.position.y = (*(ee_data.end()-1))[1];
        this_pose_stamped.pose.position.z = (*(ee_data.end()-1))[2];

        path_.poses.push_back(this_pose_stamped);
        path_pub_.publish(path_);
    }

    void pubObs(){
        visualization_msgs::Marker marker;

        marker.header.frame_id = "panda1/world";
        marker.header.stamp = ros::Time();
//        marker.ns = "my_namespace";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        bool flag = true;
        ros::Rate rate(100);
        while (flag) {
            if (obs_pub_.getNumSubscribers() > 0) {
                for (auto i = 0; i < obs_.size(); i++) {
                    marker.id = i;
                    marker.pose.position.x = obs_[i][0];
                    marker.pose.position.y = obs_[i][1];
                    marker.pose.position.z = obs_[i][2];
                    obs_pub_.publish(marker);
                }
                flag = false;
            }
            else{
                rate.sleep();
            }
        }
    }

};

#endif //PANDA_CONTROL_VISUALIZE_H
