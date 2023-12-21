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

#include <opencv2/opencv.hpp>

#include "utility.h"

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

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)
using PointXYZIRT = VelodynePointXYZIRT;

class ICPRegistration
{

private:

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    Eigen::Matrix4f cumulative_transform = Eigen::Matrix4f::Identity();

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserOdometryGlobal;
    ros::Publisher  pubExtractedCloud;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    std::deque<sensor_msgs::Imu> imuQueue;


    pcl::PointCloud<PointXYZIRT>::Ptr       laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr    extractedCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr    fullCloud;

    sensor_msgs::PointCloud2 currentCloudMsg;
    std_msgs::Header         cloudHeader;
    tf::StampedTransform     laserOdometryTrans;
    ros::Time                timeLaserInfoStamp;

    double timeScanCur;
    double timeScanEnd;
    

    cv::Mat rangeMat;
    int deskewFlag;
    std::mutex imuLock;


public:
    ICPRegistration() : deskewFlag(0)
    {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("points_raw", 5, &ICPRegistration::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("registered_cloud", 1);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("odometry_ICP", 1);
        allocateMemory();
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());

        fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        extractedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extQRPY = Eigen::Quaterniond(extRPY);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
    }

    ~ICPRegistration(){}

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
                // handleImuData(imu_msg);
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

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
    {
            projectPointCloud();

            cloudExtraction();

            publishClouds();

            pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloudHeader = pc_msg->header;
            timeLaserInfoStamp = pc_msg->header.stamp;
            pcl::fromROSMsg(*pc_msg, *source_cloud);

            // Perform ICP
            performICP(source_cloud);

            resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());

        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();

        cloudQueue.pop_front();

        if (sensor == SensorType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // remove Nan
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    void publishOdometry(const Eigen::Matrix4f &transform){

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
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
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

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;

        
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
                
                Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
                cumulative_transform = cumulative_transform * transformation_matrix;
                publishOdometry(cumulative_transform);
            }
            else
            {
                std::cout << "ICP did not converge." << std::endl;
            }
            previous_cloud = source_cloud;
        }   

    }

    // sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<pcl::PointXYZI>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
    // {
    //     sensor_msgs::PointCloud2 tempCloud;
    //     pcl::toROSMsg(*thisCloud, tempCloud);
    //     tempCloud.header.stamp = thisStamp;
    //     tempCloud.header.frame_id = thisFrame;
    //     if (thisPub->getNumSubscribers() != 0)
    //         thisPub->publish(tempCloud);
    //     return tempCloud;
    // }

    void publishClouds()
    {
        // cloudInfo.header = cloudHeader;
        currentCloudMsg  = publishCloud(&pubExtractedCloud, extractedCloud, timeLaserInfoStamp, "base_link");
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

            // int rowIdn = laserCloudIn->points[i].ring;
            int rowIdn = (i % 64) + 1 ; // for MulRan dataset, Ouster OS1-64 .bin file,  giseop 

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

            // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

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
            // cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    // cloudInfo.pointColInd[count] = j;
                    // save range info
                    // cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            // cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }

    // float pointDistance(pcl::PointXYZI p1, pcl::PointXYZI p2)
    // {
    //     return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    // }

    // float pointDistance(pcl::PointXYZI p)
    // {
    //     return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    // }

    // sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    // {
    //     sensor_msgs::Imu imu_out = imu_in;
    //     // rotate acceleration
    //     Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    //     acc = extRot * acc;
    //     imu_out.linear_acceleration.x = acc.x();
    //     imu_out.linear_acceleration.y = acc.y();
    //     imu_out.linear_acceleration.z = acc.z();
    //     // rotate gyroscope
    //     Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    //     gyr = extRot * gyr;
    //     imu_out.angular_velocity.x = gyr.x();
    //     imu_out.angular_velocity.y = gyr.y();
    //     imu_out.angular_velocity.z = gyr.z();
    //     // rotate roll pitch yaw
    //     Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    //     Eigen::Quaterniond q_final = q_from * extQRPY;
    //     imu_out.orientation.x = q_final.x();
    //     imu_out.orientation.y = q_final.y();
    //     imu_out.orientation.z = q_final.z();
    //     imu_out.orientation.w = q_final.w();

    //     if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    //     {
    //         ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
    //         ros::shutdown();
    //     }

    //     return imu_out;
    // }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_registration_node");
    ICPRegistration icp_reg;
    icp_reg.processBag("your_bag_file.bag");

    return 0;
}