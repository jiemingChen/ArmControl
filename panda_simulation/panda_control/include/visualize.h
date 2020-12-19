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
    ros::Publisher path_pub_, obs_pub_, pred_pub_, waypoint_pub_;
    ros::NodeHandle nh_;
    nav_msgs::Path path_, actual_path_, waypoint_path;

    vector<array<double,3>> obs_;
public:
    Visual(ros::NodeHandle& nh){
        nh_ = nh;
        obs_pub_ = nh_.advertise<visualization_msgs::Marker>( "obs_marker",  20 );
        pred_pub_ = nh_.advertise<nav_msgs::Path>("pred_trajectory",100);
        path_pub_ = nh_.advertise<nav_msgs::Path>("actual_trajectory",100);
        waypoint_pub_ = nh_.advertise<nav_msgs::Path>("waypoint_trajectory",100);


        path_.header.frame_id="panda1/world";
        actual_path_.header.frame_id="panda1/world";
        waypoint_path.header.frame_id="panda1/world";
    }
    ~Visual(){
        path_.poses.clear();
        actual_path_.poses.clear();
    }

    void setObstacleInfo(const vector<array<double,3>>& obs){
        obs_ = obs;
    }

    void pubPath(){
        if(ee_data.empty())
            return;

        actual_path_.header.stamp=ros::Time().now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = (*(ee_data.end()-1))[0];
        this_pose_stamped.pose.position.y = (*(ee_data.end()-1))[1];
        this_pose_stamped.pose.position.z = (*(ee_data.end()-1))[2];

        actual_path_.poses.push_back(this_pose_stamped);
        path_pub_.publish(actual_path_);
    }

    void pubPredictPath(const vector<Eigen::Vector7d>& path, Panda& robot){
        path_.header.stamp=ros::Time().now();
        geometry_msgs::PoseStamped this_pose_stamped;

        Eigen::Affine3d desir_transform;
        Eigen::Vector3d desir_position;
        for(auto& joint_point: path){
            robot.setJoints(joint_point, Eigen::Vector7d::Zero());
            desir_transform = robot.fkEE();
            desir_position = desir_transform.translation();

            this_pose_stamped.pose.position.x = desir_position(0);
            this_pose_stamped.pose.position.y = desir_position(1);
            this_pose_stamped.pose.position.z = desir_position(2);

            path_.poses.push_back(this_pose_stamped);
        }

        pred_pub_.publish(path_);
        path_.poses.clear();
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

    void pubWaypoints(vector<array<double,7>>& points, Panda& robot){
        Eigen::Affine3d waypoint_transform;
        Eigen::Vector3d waypoint_position;
        Eigen::Vector7d joints_pos;
        geometry_msgs::PoseStamped this_pose_stamped;

        for(size_t i=0; i<points.size(); i++){
            joints_pos = Eigen::Map<Eigen::Vector7d>(points[i].data());
            robot.setJoints(joints_pos, Eigen::Vector7d::Zero());
            waypoint_transform = robot.fkEE();
            waypoint_position = waypoint_transform.translation();

            this_pose_stamped.pose.position.x = waypoint_position(0);
            this_pose_stamped.pose.position.y = waypoint_position(1);
            this_pose_stamped.pose.position.z = waypoint_position(2);

            waypoint_path.poses.push_back(this_pose_stamped);
        }
        waypoint_path.header.stamp=ros::Time().now();

        waypoint_pub_.publish(waypoint_path);
        waypoint_path.poses.clear();
    }

    void pubWaypoints(vector<array<double,3>>& points, Panda& robot){

         geometry_msgs::PoseStamped this_pose_stamped;

        for(size_t i=0; i<points.size(); i++){
            this_pose_stamped.pose.position.x = points[i][0];
            this_pose_stamped.pose.position.y = points[i][1];
            this_pose_stamped.pose.position.z = points[i][2];

            waypoint_path.poses.push_back(this_pose_stamped);
        }
        waypoint_path.header.stamp=ros::Time().now();

        waypoint_pub_.publish(waypoint_path);
        waypoint_path.poses.clear();
    }

};

#endif //PANDA_CONTROL_VISUALIZE_H
