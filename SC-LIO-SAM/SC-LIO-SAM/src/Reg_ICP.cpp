#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf/transform_datatypes.h>

#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <tf2_eigen/tf2_eigen.h>


struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

using PointXYZIRT = VelodynePointXYZIRT;

class ICPRegistration
{

private:

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;
    bool has_previous_cloud;
    bool has_initial_rotation;
    tf::Quaternion initial_rotation;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    Eigen::Matrix4f cumulative_transform = Eigen::Matrix4f::Identity();

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserOdometryGlobal;
    ros::Publisher  pubExtractedCloud;


    pcl::PointCloud<PointXYZIRT>::Ptr       laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr    extractedCloud;

    sensor_msgs::PointCloud2 currentCloudMsg;
    std_msgs::Header cloudHeader;
    tf::StampedTransform laserOdometryTrans;

    

public:
    ICPRegistration() : has_previous_cloud(false), has_initial_rotation(false)
    {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ICPRegistration::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("registered_cloud", 1);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("odometry", 1);
        allocateMemory();
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());

        fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        extractedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
    }

    void processBag(const std::string &bag_file)
    {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        for (const rosbag::MessageInstance &msg : rosbag::View(bag))
        {
            if (msg.isType<sensor_msgs::Imu>() && msg.getTopic() == "/your_imu_topic")
            {
                // Handle IMU data
                sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
                handleImuData(imu_msg);
            }
            else if (msg.isType<sensor_msgs::PointCloud2>() && msg.getTopic() == "/your_lidar_topic")
            {
                // Handle LiDAR data
                sensor_msgs::PointCloud2::ConstPtr pc_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                cloudHandler(pc_msg);
            }
        }

        bag.close();
    }

    // void handleImuData(const sensor_msgs::Imu::ConstPtr &imu_msg)
    // {
    //     // Extract orientation from IMU data
    //     tf::Quaternion imu_orientation;
    //     tf::quaternionMsgToTF(imu_msg->orientation, imu_orientation);

    //     // Use the IMU orientation as the initial rotation
    //     if (!has_initial_rotation)
    //     {
    //         initial_rotation = imu_orientation;
    //         has_initial_rotation = true;
    //     }
    // }
    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
        {
            // projectPointCloud();

            // cloudExtraction();

            // publishClouds();

            // resetParameters();

            pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl_conversions::toPCL(pc_msg->header, source_cloud->header);  // Set the header
            pcl::fromROSMsg(*pc_msg, *source_cloud);

            // Perform ICP
            performICP(source_cloud);
            publishClouds();
    }

    void publishOdometry(const Eigen::Matrix4f &transform){

        cloudHeader = previous_cloud->header;

        // Publish TF
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header = cloudHeader;
        transform_stamped.child_frame_id = "base_link";
        tf::transformEigenToMsg(transform.cast<double>(), transform_stamped.transform);

        // Broadcast the transform

        static tf::TransformBroadcaster tf_broadcaster;
        tf_broadcaster.sendTransform(transform_stamped);

        // Publish odometry for ROS (global) (nav_msgs Odometry)
        nav_msgs::Odometry laserOdometryROS;
        
        std::cout << "PUBLISHING ODOMETRY" << endl;
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = cloudHeader;
        laserOdometryROS.header.frame_id = "odom";
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transform(0, 3);
        laserOdometryROS.pose.pose.position.y = transform(1, 3);
        laserOdometryROS.pose.pose.position.z = transform(2, 3);
        tf::Quaternion q;

        tf::Matrix3x3(transform.block<3, 3>(0, 0).cast<double>()).getRotation(q);
        laserOdometryROS.pose.pose.orientation.x = q.x();
        laserOdometryROS.pose.pose.orientation.y = q.y();
        laserOdometryROS.pose.pose.orientation.z = q.z();
        laserOdometryROS.pose.pose.orientation.w = q.w();
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        
        // static tf::TransformBroadcaster br;
        // tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
        //                                             tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        // tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, cloudHeader, "odom", "lidar_link");
        // br.sendTransform(trans_odom_to_lidar);
    }
    void performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Downsample the source cloud to improve efficiency
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(source_cloud);
        voxel_grid.setLeafSize(0.1, 0.1, 0.1); 
        voxel_grid.filter(*source_cloud);

        // Perform ICP registration

        if (!has_previous_cloud)
        {
            // If there is no previous point cloud, set the current cloud as the target
            icp.setInputTarget(source_cloud);
            has_previous_cloud = true;
        }
        else
        {
            // Set the current source cloud
            icp.setInputSource(source_cloud);
            icp.setInputTarget(previous_cloud);
            // Align the current source cloud with the target (previous) cloud
            icp.align(*aligned_cloud);

            

            // Publish odometry
            if (icp.hasConverged())
            {
                std::cout << "ICP converged. Score: " << icp.getFitnessScore() << std::endl; 
                aligned_cloud  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");

                previous_cloud = source_cloud;
                Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
                cumulative_transform = cumulative_transform * transformation_matrix;
                publishOdometry(cumulative_transform);
            }
            else
            {
                std::cout << "ICP did not converge." << std::endl;
            }
        }   

    }

    sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<pcl::PointXYZI>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        if (thisPub->getNumSubscribers() != 0)
            thisPub->publish(tempCloud);
        return tempCloud;
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        currentCloudMsg  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            // int rowIdn = (i % 64) + 1 ; // for MulRan dataset, Ouster OS1-64 .bin file,  giseop 

            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            static float ang_res_x = 360.0/float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_registration_node");
    ICPRegistration icp_reg;
    icp_reg.processBag("your_bag_file.bag");

    return 0;
}