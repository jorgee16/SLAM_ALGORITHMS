#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <fstream>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

#include <sstream>
#include <stdio.h>

std::ofstream ofs;

void callhandler(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO("Seq: [%d]", msg->header.seq);
    // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        // Extract translation and rotation information from the odometry message
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
    tf::Matrix3x3 rotation_matrix(quaternion);
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

    // Cast tf::Matrix3x3 to Eigen::Matrix3f
    Eigen::Matrix3f rotation_matrix_eigen;
    rotation_matrix_eigen << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
                            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
                            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2];


    transformation_matrix.block<3, 3>(0, 0) = rotation_matrix_eigen;
    transformation_matrix.block<3, 1>(0, 3) <<  msg->pose.pose.position.x,
                                                msg->pose.pose.position.y,
                                                msg->pose.pose.position.z;

    if (ofs.is_open()){

        // SAVE TRANSFORMATION MATRIX
        // ofs << transformation_matrix << "\n";

        ofs << rotation_matrix[0][0] << " ";
        ofs << rotation_matrix[0][1] << " ";
        ofs << rotation_matrix[0][2] << " ";
        ofs << msg->pose.pose.position.x << " ";
        ofs << rotation_matrix[1][0] << " ";
        ofs << rotation_matrix[1][1] << " ";
        ofs << rotation_matrix[1][2] << " ";
        ofs << msg->pose.pose.position.y << " ";
        ofs << rotation_matrix[2][0] << " ";
        ofs << rotation_matrix[2][1] << " ";
        ofs << rotation_matrix[2][2] << " ";
        ofs << msg->pose.pose.position.z << " ";
        ofs << "0" << " " << "0" << " " <<  "0" << " " <<  "1" << "\n";
        // ofs << "0" << " ";
        // ofs << "0" << " ";
        // ofs << "1" << "\n";
        // SAVE POSES (X,Y,Z,QX,QY,QZ,QW)

        // ofs << msg->header.stamp << " ";
        // ofs << msg->pose.pose.position.x << " ";
        // ofs << msg->pose.pose.position.y << " ";
        // ofs << msg->pose.pose.position.z << " ";
        // ofs << msg->pose.pose.orientation.x << " ";
        // ofs << msg->pose.pose.orientation.y << " ";
        // ofs << msg->pose.pose.orientation.z << " ";
        // ofs << msg->pose.pose.orientation.w << "\n";
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
    ros::Subscriber Listener = nh.subscribe<nav_msgs::Odometry>("aft_mapped_to_init", 100, callhandler); //aft_mapped_to_init  e integrated_to_init
    
    // write_to_file(ofs);
    if(ofs.good()){
        remove("/home/joaojorge/POLO2_SEQ02_SC_LL.txt");
        ofs.open("/home/joaojorge/POLO2_SEQ02_SC_LL.txt", std::ofstream::app);
    }
    ros::spin();

    return 0;
}