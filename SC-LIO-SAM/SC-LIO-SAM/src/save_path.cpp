#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
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

        // Save the transformation matrix to the file
        // ofs << msg->header.stamp << " ";
        // ofs << transformation_matrix << "\n";
        for (int i = 0; i < transformation_matrix.rows(); ++i) {
            for (int j = 0; j < transformation_matrix.cols(); ++j) {
                ofs << transformation_matrix(i, j) << " ";
            }
        }

        ofs << std::endl;
        
        // ofs << rotation_matrix[0][0] << " ";
        // ofs << rotation_matrix[0][1] << " ";
        // ofs << rotation_matrix[0][2] << " ";
        // ofs << rotation_matrix[1][0] << " ";
        // ofs << rotation_matrix[1][1] << " ";
        // ofs << rotation_matrix[1][2] << " ";
        // ofs << rotation_matrix[2][0] << " ";
        // ofs << rotation_matrix[2][1] << " ";
        // ofs << rotation_matrix[2][2] << " ";
        // ofs << msg->pose.pose.position.x << " ";
        // ofs << msg->pose.pose.position.y << " ";
        // ofs << msg->pose.pose.position.z << " ";
        // ofs << 1 << "\n";


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

int main(int argc, char **argv)
{   
    // ros::init(argc, argv, "save_path");
    // ros::NodeHandle nh;
    // ros::Subscriber Listener = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 200, callhandler); // //integrated_to_init
    // // ros::Subscriber Listener = nh.subscribe<nav_msgs::Odometry>("icp_odometry/odom", 100, callhandler); 
    
    // if(ofs.good()){
    //     remove("/home/joaojorge/Documents/LIO_SAM_E4.txt");
    //     ofs.open("/home/joaojorge/Documents/LIO_SAM_E4.txt", std::ofstream::app);
    // }
    // ros::spin();

    return 0;
}