#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <fstream>

#include <sstream>
#include <stdio.h>

std::ofstream ofs;

void callhandler(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO("Seq: [%d]", msg->header.seq);
    // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);


    if (ofs.is_open()){
           //store array contents to text file
           
        //    ofs << msg->header.stamp << " " << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << "\n";
        ofs << msg->header.stamp << " ";
        ofs << msg->pose.pose.position.x << " ";
        ofs << msg->pose.pose.position.y << " ";
        ofs << msg->pose.pose.position.z << " ";
        ofs << msg->pose.pose.orientation.x << " ";
        ofs << msg->pose.pose.orientation.y << " ";
        ofs << msg->pose.pose.orientation.z << " ";
        ofs << msg->pose.pose.orientation.w << "\n";
    }
    else 
        printf("Unable to open file \n");
}

// void write_to_file(fstream& ofs){
//     if (ofs.is_open()){
//         ofs
//     }

// }
int main(int argc, char **argv)
{   
    // std::ofstream ofs ("home/joaojorge/LOAM_ws/save_pose.txt", std::ofstream::app);
    ros::init(argc, argv, "save_pose");
    ros::NodeHandle nh;
    ros::Subscriber Listener = nh.subscribe<nav_msgs::Odometry>("integrated_to_init", 100, callhandler); //aft_mapped_to_init  e integrated_to_init
    
    // write_to_file(ofs);
    if(ofs.good()){
        remove("/home/joaojorge/POLO2_SEQ1_SC_LL.txt");
        ofs.open("/home/joaojorge/POLO2_SEQ1_SC_LL.txt", std::ofstream::app);
    }
    ros::spin();

    return 0;
}