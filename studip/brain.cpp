//
//  brain.cpp
//  
//
//  Created by Tim G on 29.11.18.
//

#include "brain.hpp"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;

void trigger(nav_msgs::Odometry::Ptr msg)
{
    static double d = 0;
    static double a = 0;
    
    d += sqrt(msg->pose.pose.position.x * msg->pose.pose.position.x
              + msg->pose.pose.position.y * msg->pose.pose.position.y);
    
    auto q = msg->pose.pose.orientation;
    
    a += atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    
    if(d > 1 | a > M_PI / 2)
    {
        d = 0;
        a = 0;
        geometry_msgs::Twist msg2;
        pub.publish(msg2);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "default");
    ros::NodeHandle n;
    n.subscribe("odom", 10, trigger);
    n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::spin();
}
