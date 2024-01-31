#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include "utility.h"
#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"

#include <pcl/registration/gicp.h>

// typedef pcl::PointXYZI PointType;

// struct OusterPointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity)
//     (std::uint8_t, ring, ring) (std::uint16_t, noise, noise) (std::uint32_t, range, range)
// )

// struct VelodynePointXYZIRT
// {
//     PCL_ADD_POINT4D
//     PCL_ADD_INTENSITY;
//     uint16_t ring;
//     float time;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (std::uint16_t, ring, ring) (float, time, time)
// )
// using PointXYZIRT = VelodynePointXYZIRT;


class ICPRegistration : public ParamServer
{

private:
    
    // Constants
    const double ICP_FITNESS_THRESH = 0.1;
    const double ICP_MAX_CORR_DIST = 1.0;
    const double ICP_EPSILON = 1e-06;
    const double ICP_MAX_ITERS = 10; // 50

    // tf2_ros::TransformBroadcaster tf_broadcaster;
    // Eigen::Matrix4f cumulative_transform = Eigen::Matrix4f::Identity();

   
    // ros::Publisher  pubLaserOdometryGlobal;
    // ros::Publisher  pubExtractedCloud;
    // ros::Publisher pubIcpKeyFrames;

    // std::deque<sensor_msgs::PointCloud2> cloudQueue;
    // std::deque<sensor_msgs::Imu>         imuQueue;


    // pcl::PointCloud<PointXYZIRT>::Ptr       laserCloudIn;
    // pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    // pcl::PointCloud<PointType>::Ptr    extractedCloud;
    // pcl::PointCloud<PointType>::Ptr    fullCloud;

    // sensor_msgs::PointCloud2 currentCloudMsg;
    // std_msgs::Header         cloudHeader;
    // tf::StampedTransform     laserOdometryTrans;
    // ros::Time                timeLaserInfoStamp;

    // double timeScanCur;
    // double timeScanEnd;

    bool has_previous_cloud;

    // ROS node handle, URDF frames, topics and publishers
    ros::Publisher prev_cloud_pub_;
    ros::Publisher aligned_cloud_pub_;
    ros::Publisher icp_odom_pub_;
    ros::Publisher icp_pose_pub_;
    ros::Publisher icp_odom_path_pub_;
    
    ros::Subscriber laser_cloud_sub_;

    // Odometry path containers
    nav_msgs::Odometry icp_odom_;
    nav_msgs::Path icp_odom_path_;

    // int verbosity_level_;
    bool initial_pose_set_;
    bool odom_inited_;

    // Translations and rotations estimated by ICP
    bool new_transform_;

    // Cloud skipping and filtering
    int clouds_skipped_;
    int num_clouds_skip_;
    double voxel_leaf_size_;

    // Transforms and intermediate poses
    ros::Time latest_stamp;
    Pose6DOF icp_latest_transform_;
    std::vector<Pose6DOF> icp_odom_poses_;
    
    // PCL clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud_;

    // cv::Mat rangeMat;
    // int deskewFlag;
    // std::mutex imuLock;


public:
    ICPRegistration()
    {
        // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("points_raw", 5, &ICPRegistration::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        // pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("registered_cloud", 1);
        // pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("odometry_ICP", 1);
        // pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("icp_corrected_cloud", 1);
        allocateMemory();
        init();
        has_previous_cloud = false;
    }

    void allocateMemory()
    {
        // laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        // tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());

        // fullCloud.reset(new pcl::PointCloud<PointType>());
        // extractedCloud.reset(new pcl::PointCloud<PointType>());

        // fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        // resetParameters();

        prev_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        curr_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    void init() {
        // Nao e necessario dar load de parametros porque o utility.h ja tem os parametros
        // loadParameters();
        advertisePublishers();
        registerSubscribers();

        ROS_INFO("ICP odometry initialized");
    }   

    void advertisePublishers(){

        icp_odom_pub_ = nh.advertise<nav_msgs::Odometry>("icp_odometer/odom", 1, true);
        icp_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("icp_odometer/pose", 1, true);
        icp_odom_path_pub_ = nh.advertise<nav_msgs::Path>("icp_odometer/path", 1, true);

        // ICP odometry debug topics

        prev_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometer/prev_cloud", 1);
        aligned_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometer/aligned_cloud", 1);
    }

    void registerSubscribers(){
        laser_cloud_sub_ = nh.subscribe("points_raw", 5, &ICPRegistration::laserCloudCallback, this);
    }

    bool isOdomReady() const {
        return odom_inited_;
    }

    void setInitialPose(const Pose6DOF& initial_pose) {
        icp_odom_poses_.push_back(initial_pose);
        initial_pose_set_ = true;
    }

    Pose6DOF getFirstPose() const {
        return icp_odom_poses_.front();
    }

    Pose6DOF getLatestPose() const {
        return icp_odom_poses_.back();
    }

    void getEstimates(ros::Time& stamp, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Pose6DOF& latest_icp_transform, Pose6DOF& icp_pose, bool& new_transform) {
        cloud = prev_cloud_;

        stamp = latest_stamp;
        latest_icp_transform = icp_latest_transform_;
        icp_pose = getLatestPose();
        new_transform = new_transform_;

        icp_latest_transform_.setIdentity();

        this->new_transform_ = false;
    }

    void voxelFilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr* input, pcl::PointCloud<pcl::PointXYZ>::Ptr* output) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(*input);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(**output);
    }

    void publishPath(const ros::Time& stamp) {
        icp_odom_path_.header.stamp = stamp;
        icp_odom_path_.header.frame_id = "map";
        icp_odom_path_pub_.publish(icp_odom_path_);
    }

    bool updateICPOdometry(const ros::Time& stamp, const Eigen::Matrix4d& T) {
        ROS_INFO("ICP odometry update!");
        Pose6DOF transform(T, stamp);
        Pose6DOF prev_pose = getLatestPose();
        Pose6DOF new_pose = prev_pose + transform;
        icp_latest_transform_ += transform;


        std::cout << "#####		ICP odometry		#####" << std::endl;
        std::cout << "Initial pose:\n" << getFirstPose().toStringQuat("   ");
        std::cout << "Prev odometry pose:\n" << prev_pose.toStringQuat("   ");
        std::cout << "Cloud transform = " << transform.toStringQuat("   ");
        std::cout << "ICP odometry pose = " << new_pose.toStringQuat("   ");
        std::cout << std::endl;

        if(transform.norm() < 0.00001)
        	return false;
        else
        {
            new_transform_ = true;
            icp_odom_poses_.push_back(new_pose);
            insertPoseInPath(new_pose.toROSPose(), "map", stamp, icp_odom_path_);

            if (icp_odom_pub_.getNumSubscribers() > 0) {
            publishOdometry(new_pose.pos, new_pose.rot, "map", "odom", stamp, &icp_odom_pub_);
            }
            if (icp_pose_pub_.getNumSubscribers() > 0) {
            publishPoseStamped(new_pose.pos, new_pose.rot, "map", stamp, &icp_pose_pub_);
            }
            if (icp_odom_path_pub_.getNumSubscribers() > 0) {
            publishPath(stamp);
            }

            return true;
        }
    }

    // void resetParameters()
    // {
    //     laserCloudIn->clear();
    //     extractedCloud->clear();
    //     rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    // }

    ~ICPRegistration(){}

    // pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    // {
    //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    //     PointType *pointFrom;

    //     int cloudSize = cloudIn->size();
    //     cloudOut->resize(cloudSize);

    //     Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
    //     #pragma omp parallel for num_threads(numberOfCores)
    //     for (int i = 0; i < cloudSize; ++i)
    //     {
    //         pointFrom = &cloudIn->points[i];
    //         cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
    //         cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
    //         cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
    //         cloudOut->points[i].intensity = pointFrom->intensity;
    //     }
    //     return cloudOut;
    // }

    // void processBag(const std::string &bag_file)
    // {
    //     rosbag::Bag bag;
    //     bag.open(bag_file, rosbag::bagmode::Read);

    //     for (const rosbag::MessageInstance &msg : rosbag::View(bag))
    //     {
    //         if (msg.isType<sensor_msgs::Imu>() && msg.getTopic() == "/your_imu_topic")
    //         {
    //             // Handle IMU data
    //             sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
    //             // handleImuData(imu_msg);
    //         }
    //         else if (msg.isType<sensor_msgs::PointCloud2>() && msg.getTopic() == "/your_lidar_topic")
    //         {
    //             // Handle LiDAR data
    //             sensor_msgs::PointCloud2::ConstPtr pc_msg = msg.instantiate<sensor_msgs::PointCloud2>();
    //             cloudHandler(pc_msg);
    //         }
    //     }

    //     bag.close();
    // }

    // void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    // {
    //     sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

    //     std::lock_guard<std::mutex> lock1(imuLock);
    //     imuQueue.push_back(thisImu);

    //     // debug IMU data
    //     // cout << std::setprecision(6);
    //     // cout << "IMU acc: " << endl;
    //     // cout << "x: " << thisImu.linear_acceleration.x << 
    //     //       ", y: " << thisImu.linear_acceleration.y << 
    //     //       ", z: " << thisImu.linear_acceleration.z << endl;
    //     // cout << "IMU gyro: " << endl;
    //     // cout << "x: " << thisImu.angular_velocity.x << 
    //     //       ", y: " << thisImu.angular_velocity.y << 
    //     //       ", z: " << thisImu.angular_velocity.z << endl;
    //     // double imuRoll, imuPitch, imuYaw;
    //     // tf::Quaternion orientation;
    //     // tf::quaternionMsgToTF(thisImu.orientation, orientation);
    //     // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    //     // cout << "IMU roll pitch yaw: " << endl;
    //     // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    // }

    // void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
    // {
    //     timeLaserInfoStamp = pc_msg->header.stamp;

    //     if (!cachePointCloud(pc_msg))
    //     return;

    //     projectPointCloud();

    //     cloudExtraction();

    //     publishClouds();

    //     // Perform ICP
    //     pcl::PointCloud<PointType>::Ptr source_cloud(new pcl::PointCloud<PointType>());
    //     source_cloud = fullCloud;

    //     performICP(source_cloud);
        

    //     resetParameters();
    // }
    void laserCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        ROS_INFO("Cloud callback!");
        if (!initial_pose_set_) {
            ROS_WARN("ICP odometry waiting for initial pose.");
            return;
        }

        if (clouds_skipped_ < num_clouds_skip_) {
            ROS_WARN("ICP odometry skips cloud");
            clouds_skipped_++;
            return;
        }
        clouds_skipped_ = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        // Filter cloud
        voxelFilterCloud(&input_cloud, &curr_cloud_);

        if (prev_cloud_->points.size() == 0) {
            *prev_cloud_ = *curr_cloud_;
            return;
        }

        latest_stamp = cloud_msg->header.stamp;

        // Registration
        // GICP is said to be better, but what about NICP from Serafin?
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations(ICP_MAX_ITERS);
        icp.setTransformationEpsilon(ICP_EPSILON);
        icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
        icp.setRANSACIterations(0);
        icp.setInputSource(curr_cloud_);
        icp.setInputTarget(prev_cloud_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_in_curr_frame(new pcl::PointCloud<pcl::PointXYZ>()),
            curr_cloud_in_prev_frame(new pcl::PointCloud<pcl::PointXYZ>()), joint_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        icp.align(*curr_cloud_in_prev_frame);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();

        if (icp.hasConverged() && icp.getFitnessScore() < 20) {
            // ROS_WARN("IcpSlam:    ICP odometer converged");
            // std::cout << "Estimated T:\n" << T << std::endl;
            Eigen::Matrix4d T_inv = T.inverse();
            pcl::transformPointCloud(*prev_cloud_, *prev_cloud_in_curr_frame, T_inv);
            bool success = updateICPOdometry(latest_stamp, T);
            if (success) {
            odom_inited_ = true;
            *prev_cloud_ = *curr_cloud_;
            }

            if (prev_cloud_pub_.getNumSubscribers() > 0) {
                publishPointCloud(prev_cloud_, cloud_msg->header.frame_id, latest_stamp, &prev_cloud_pub_);
            }
            if (aligned_cloud_pub_.getNumSubscribers() > 0) {
                publishPointCloud(curr_cloud_in_prev_frame, cloud_msg->header.frame_id, latest_stamp, &aligned_cloud_pub_);
            }
        }
    }

    // bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    // {

    //     // cache point cloud
    //     cloudQueue.push_back(*laserCloudMsg);
    //     if (cloudQueue.size() <= 2)
    //         return false;

    //     // convert cloud
    //     currentCloudMsg = std::move(cloudQueue.front());

    //     cloudHeader = currentCloudMsg.header;
    //     timeScanCur = cloudHeader.stamp.toSec();

    //     cloudQueue.pop_front();

    //     if (sensor == SensorType::VELODYNE)
    //     {
    //         pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
    //     }
    //     else if (sensor == SensorType::OUSTER)
    //     {
    //         // Convert to Velodyne format
    //         pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
    //         laserCloudIn->points.resize(tmpOusterCloudIn->size());
    //         laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
    //         for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
    //         {
    //             auto &src = tmpOusterCloudIn->points[i];
    //             auto &dst = laserCloudIn->points[i];
    //             dst.x = src.x;
    //             dst.y = src.y;
    //             dst.z = src.z;
    //             dst.intensity = src.intensity;
    //             dst.ring = src.ring;
    //             dst.time = src.t * 1e-9f;
    //         }
    //     }
    //     else
    //     {
    //         ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
    //         ros::shutdown();
    //     }

    //     // get timestamp
    //     timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

    //     // remove Nan
    //     std::vector<int> indices;
    //     pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

    //     // check dense flag
    //     if (laserCloudIn->is_dense == false)
    //     {
    //         ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
    //         ros::shutdown();
    //     }

    //     // check ring channel
    //     static int ringFlag = 0;
    //     if (ringFlag == 0)
    //     {
    //         ringFlag = -1;
    //         for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
    //         {
    //             if (currentCloudMsg.fields[i].name == "ring")
    //             {
    //                 ringFlag = 1;
    //                 break;
    //             }
    //         }
    //         if (ringFlag == -1)
    //         {
    //             ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
    //             ros::shutdown();
    //         }
    //     }

    //     // check point time
    //     if (deskewFlag == 0)
    //     {
    //         deskewFlag = -1;
    //         for (auto &field : currentCloudMsg.fields)
    //         {
    //             if (field.name == "time" || field.name == "t")
    //             {
    //                 deskewFlag = 1;
    //                 break;
    //             }
    //         }
    //         if (deskewFlag == -1)
    //             ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    //     }



    //     return true;
    // }

    // void publishOdometry(const Eigen::Matrix4f &transform){

    //     // Publish TF
    //     geometry_msgs::TransformStamped transform_stamped;
    //     transform_stamped.header = cloudHeader;
    //     transform_stamped.child_frame_id = "base_link";

    //     Eigen::Matrix4d transformDouble = transform.cast<double>();

    //     Eigen::Affine3d affineTransform(transformDouble);
    //     tf::transformEigenToMsg(affineTransform, transform_stamped.transform);

    //     // Broadcast the transform

    //     static tf::TransformBroadcaster tf_broadcaster;
    //     tf_broadcaster.sendTransform(transform_stamped);

    //     // Publish odometry for ROS (global) (nav_msgs Odometry)
    //     nav_msgs::Odometry laserOdometryROS;
        
    //     std::cout << "PUBLISHING ODOMETRY" << endl;
    //     laserOdometryROS.header.stamp = timeLaserInfoStamp;
    //     laserOdometryROS.header.frame_id = "odom";
    //     laserOdometryROS.child_frame_id = "odom_mapping";
    //     laserOdometryROS.pose.pose.position.x = transform(0, 3);
    //     laserOdometryROS.pose.pose.position.y = transform(1, 3);
    //     laserOdometryROS.pose.pose.position.z = transform(2, 3);
    //     // tf::Quaternion q;

    //     Eigen::Matrix3f rotationMatrix = transform.block<3, 3>(0, 0);
    //     Eigen::Quaternionf q(rotationMatrix);

    //     laserOdometryROS.pose.pose.orientation.x = q.x();
    //     laserOdometryROS.pose.pose.orientation.y = q.y();
    //     laserOdometryROS.pose.pose.orientation.z = q.z();
    //     laserOdometryROS.pose.pose.orientation.w = q.w();
    //     pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        
    //     // static tf::TransformBroadcaster br;
    //     // tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
    //     //                                             tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
    //     // tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, cloudHeader, "odom", "lidar_link");
    //     // br.sendTransform(trans_odom_to_lidar);
    // }
    // void performICP(const pcl::PointCloud<PointType>::Ptr& source_cloud)
    // {

    //      // Setup ICP
    //     static pcl::IterativeClosestPoint<PointType, PointType> icp;
    //     icp.setMaxCorrespondenceDistance(100);
    //     icp.setMaximumIterations(100);
    //     icp.setTransformationEpsilon(1e-6);
    //     icp.setEuclideanFitnessEpsilon(1e-6);
    //     icp.setRANSACIterations(0);

    //     if(!has_previous_cloud){
    //         previous_cloud = source_cloud;
    //         has_previous_cloud = true;
    //     }
    //     else{

    //         icp.setInputSource(source_cloud);
    //         icp.setInputTarget(previous_cloud);

    //         // Perform ICP registration
    //         pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
    //         icp.align(*aligned);
    //     }

    //     // Downsample the source cloud to improve efficiency
    //     // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    //     // voxel_grid.setInputCloud(source_cloud);
    //     // voxel_grid.setLeafSize(0.1, 0.1, 0.1); 
    //     // voxel_grid.filter(*source_cloud);


    //     // Publish odometry
    //     if (icp.hasConverged())
    //     {
    //         std::cout << "ICP converged. Score: " << icp.getFitnessScore() << std::endl; 
            
    //         // Get pose transformation
    //         // float x, y, z, roll, pitch, yaw;
    //         // Eigen::Affine3f correctionLidarFrame;
    //         // correctionLidarFrame = icp.getFinalTransformation();


    //         Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation().cast<double>();
    //         cumulative_transform = cumulative_transform * transformation_matrix;
    //         publishOdometry(cumulative_transform);
    //     }
    //     else
    //     {
    //         std::cout << "ICP did not converge." << std::endl;
    //         previous_cloud = source_cloud;
    //         return;
    //     }
           
        
    //     // publish corrected cloud
    //     // if (pubIcpKeyFrames.getNumSubscribers() != 0)
    //     // {
    //     //     pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    //     //     pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
    //     //     publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, "odom");
    //     // }
    // }

    //  void publishFrames()
    // {
    //     if (cloudKeyPoses3D->points.empty())
    //         return;
    //     // publish key poses
    //     std::cout << "PUBLISHING FRAMES" << endl;
    //     publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
    //     // Publish surrounding key frames
    //     publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
    //     // publish registered key frame
    //     if (pubRecentKeyFrame.getNumSubscribers() != 0)
    //     {
    //         pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    //         PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    //         *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
    //         *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
    //         publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
    //     }
    //     // publish registered high-res raw cloud
    //     if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
    //     {
    //         pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    //         pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
    //         PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    //         *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
    //         publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
    //     }
    //     // publish path
    //     std::cout << "PUBLISH PATH" << endl;
    //     if (pubPath.getNumSubscribers() != 0)
    //     {
    //         globalPath.header.stamp = timeLaserInfoStamp;
    //         globalPath.header.frame_id = odometryFrame;
    //         pubPath.publish(globalPath);
    //     }
    // }

    // void publishClouds()
    // {
    //     // cloudInfo.header = cloudHeader;
    //     currentCloudMsg  = publishCloud(&pubExtractedCloud, extractedCloud, timeLaserInfoStamp, "base_link");
    // }

    // void projectPointCloud()
    // {
    //     int cloudSize = laserCloudIn->points.size();
    //     // range image projection
    //     for (int i = 0; i < cloudSize; ++i)
    //     {
    //         pcl::PointXYZI thisPoint;
    //         thisPoint.x = laserCloudIn->points[i].x;
    //         thisPoint.y = laserCloudIn->points[i].y;
    //         thisPoint.z = laserCloudIn->points[i].z;
    //         thisPoint.intensity = laserCloudIn->points[i].intensity;

    //         float range = pointDistance(thisPoint);
    //         if (range < lidarMinRange || range > lidarMaxRange)
    //             continue;

    //         // int rowIdn = laserCloudIn->points[i].ring;
    //         int rowIdn = (i % 64) + 1 ; // for MulRan dataset, Ouster OS1-64 .bin file,  giseop 

    //         if (rowIdn < 0 || rowIdn >= N_SCAN)
    //             continue;

    //         if (rowIdn % downsampleRate != 0)
    //             continue;

    //         float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

    //         static float ang_res_x = 360.0/float(Horizon_SCAN);
    //         int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
    //         if (columnIdn >= Horizon_SCAN)
    //             columnIdn -= Horizon_SCAN;

    //         if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
    //             continue;

    //         if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
    //             continue;

    //         // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

    //         rangeMat.at<float>(rowIdn, columnIdn) = range;

    //         int index = columnIdn + rowIdn * Horizon_SCAN;
    //         fullCloud->points[index] = thisPoint;
    //     }
    // }

    // void cloudExtraction()
    // {
    //     int count = 0;
    //     // extract segmented cloud for lidar odometry
    //     for (int i = 0; i < N_SCAN; ++i)
    //     {
    //         // cloudInfo.startRingIndex[i] = count - 1 + 5;

    //         for (int j = 0; j < Horizon_SCAN; ++j)
    //         {
    //             if (rangeMat.at<float>(i,j) != FLT_MAX)
    //             {
    //                 // mark the points' column index for marking occlusion later
    //                 // cloudInfo.pointColInd[count] = j;
    //                 // save range info
    //                 // cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
    //                 // save extracted cloud
    //                 extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
    //                 // size of extracted cloud
    //                 ++count;
    //             }
    //         }
    //         // cloudInfo.endRingIndex[i] = count -1 - 5;
    //     }
    // }

    // void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    // {
    //     // extract near keyframes
    //     nearKeyframes->clear();
    //     int cloudSize = copy_cloudKeyPoses6D->size();
    //     for (int i = -searchNum; i <= searchNum; ++i)
    //     {
    //         int keyNear = key + i;
    //         if (keyNear < 0 || keyNear >= cloudSize )
    //             continue;
    //         *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
    //         *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
    //     }

    //     if (nearKeyframes->empty())
    //         return;

    //     // downsample near keyframes
    //     pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    //     downSizeFilterICP.setInputCloud(nearKeyframes);
    //     downSizeFilterICP.filter(*cloud_temp);
    //     *nearKeyframes = *cloud_temp;
    // }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_registration_node");

    // ICPRegistration icp_reg;
    // icp_reg.processBag("your_bag_file.bag");

    return 0;
}