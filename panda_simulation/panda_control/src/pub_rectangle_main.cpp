#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/range/irange.hpp>
int main(int argc, char** argv) {
    ros::init(argc, argv, "showline");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("window", 10);
    ros::Publisher obs_pub = n.advertise<std_msgs::Float32MultiArray>("near_boxes", 10);

    ros::Rate rate(10);
    visualization_msgs::Marker line_list;

    line_list.header.frame_id = "panda1/world";
    line_list.lifetime = ros::Duration(0.5);
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    double x_min = -0.3, x_max= 0.1, z_min= 0.3, z_max=0.9, y=0.6;
    geometry_msgs::Point p;
    p.x = x_min;
    p.y = y;
    p.z = z_min;
    geometry_msgs::Point p1;
    p1.x = x_max;
    p1.y = y;
    p1.z = z_min;
 
    geometry_msgs::Point p4;
    p4.x = x_min;
    p4.y = y;
    p4.z = z_max;
    geometry_msgs::Point p5;
    p5.x = x_max;
    p5.y = y;
    p5.z = z_max;

    line_list.points.push_back(p4);
    line_list.points.push_back(p);
    line_list.points.push_back(p5);
    line_list.points.push_back(p1);
    line_list.points.push_back(p);
    line_list.points.push_back(p1);
    line_list.points.push_back(p5);
    line_list.points.push_back(p4);


//    std::vector<double> xrange = {-0.15, -0.05, 0.05};
    std::vector<double> xrange = {-0.25,-0.2, -0.15, -0.1,  -0.05, 0, 0.05};
    std::vector<double> zrange = {0.35,0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85};


    std_msgs::Float32MultiArray msg;
    msg.data.reserve(2000);
    msg.data.push_back( (xrange.size()+zrange.size())*2 );

    for(auto x: xrange){
        msg.data.push_back(x);
        msg.data.push_back(y);
        msg.data.push_back(z_min);
        msg.data.push_back(0.05);
    }
    for(auto x: xrange){
        msg.data.push_back(x);
        msg.data.push_back(y);
        msg.data.push_back(z_max);
        msg.data.push_back(0.05);
    }
    for(auto z: zrange){
        msg.data.push_back(x_min);
        msg.data.push_back(y);
        msg.data.push_back(z);
        msg.data.push_back(0.05);
    }
    for(auto z: zrange){
        msg.data.push_back(x_max);
        msg.data.push_back(y);
        msg.data.push_back(z);
        msg.data.push_back(0.05);
    }
    msg.data.shrink_to_fit();


  while (ros::ok()) {
    marker_pub.publish(line_list);
    line_list.header.stamp = ros::Time::now();

//    if (obs_pub.getNumSubscribers() == 0)
//        continue;
//    else
      obs_pub.publish(msg);

    rate.sleep();
  }
  // ros::spinOnce();

  return 0;
}
