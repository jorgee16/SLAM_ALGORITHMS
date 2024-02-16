#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/octree/octree_search.h>

#include "utility.h"
#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"

#include <pcl/registration/gicp.h>
#include <pcl_ros/impl/transforms.hpp>


/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
                                   
class ICPRegistration : public ParamServer
{

private:
    
    // Constants
    const double ICP_FITNESS_THRESH = 0.1;
    const double ICP_MAX_CORR_DIST = 1.0;
    const double ICP_EPSILON = 1e-06;
    const double ICP_MAX_ITERS = 50; // 10

    tf2_ros::TransformBroadcaster tf_broadcaster;
    Eigen::Isometry3d T_map_to_odom_;
    // bool publish_map_transform_;

    // Mutex
    std::mutex mtx;
    // ROS node handle, URDF frames, topics and publishers
    ros::Publisher prev_cloud_pub_;
    ros::Publisher aligned_cloud_pub_;
    ros::Publisher icp_odom_pub_;
    ros::Publisher icp_pose_pub_;
    ros::Publisher icp_odom_path_pub_;
    
    ros::Subscriber laser_cloud_sub_;

    // Publish map cloud and path refined

    ros::Publisher map_cloud_pub_;
    ros::Publisher nn_cloud_pub_;
    ros::Publisher registered_cloud_pub_;
    ros::Publisher refined_path_pub_;
    ros::Publisher register_cloud_pub2;

    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;

    // Odometry path containers
    nav_msgs::Odometry icp_odom_;
    nav_msgs::Path icp_odom_path_;
    nav_msgs::Path refined_path_;
    nav_msgs::Path globalPath;

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
    std::vector<Pose6DOF> refined_poses;
    
    // PCL clouds
    pcl::PointCloud<PointType>::Ptr prev_cloud_;
    pcl::PointCloud<PointType>::Ptr curr_cloud_;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    // pcl::PointCloud<PointType>::Ptr laserCloudLastDS;

    // PCL clouds for mapping
    pcl::PointCloud<PointType>::Ptr map_cloud_;
    // pcl::octree::OctreePointCloudSearch<PointType>::Ptr map_octree_;

    vector<pcl::PointCloud<PointType>::Ptr> CloudKeyFrames;

public:
    ICPRegistration(): 
        T_map_to_odom_(Eigen::Isometry3d::Identity()),
        odom_inited_(false),
        num_clouds_skip_(0),
        voxel_leaf_size_(0.4)
        // map_octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.5))
    {
        allocateMemory();
        init();
    }
    
    ~ICPRegistration(){}

    void allocateMemory()
    {   
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        prev_cloud_.reset(new pcl::PointCloud<PointType>());
        curr_cloud_.reset(new pcl::PointCloud<PointType>());
        map_cloud_.reset(new pcl::PointCloud<PointType>());
        
        // map_octree_.reset(new pcl::octree::OctreePointCloudSearch<PointType>(0.5));
        // map_octree_->setInputCloud(map_cloud_);
    }

    void init() {
        // Nao e necessario dar load de parametros porque o utility.h ja tem os parametros

        advertisePublishers();
        registerSubscribers();
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);


        ROS_INFO("ICP odometry initialized");
    }   

    void advertisePublishers(){
        // ICP topics
    
        icp_odom_pub_ = nh.advertise<nav_msgs::Odometry>("icp_odometry/odom", 1);
        icp_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("icp_odometry/pose", 1);
        icp_odom_path_pub_ = nh.advertise<nav_msgs::Path>("icp_odometry/path", 1);

        // ICP odometry debug topics

        prev_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/prev_cloud", 1);
        aligned_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/aligned_cloud", 1);

        // Map topics

        map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/map_cloud", 1);
        nn_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/nn_cloud", 1);
        // registered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/registered_cloud2", 1);

        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/cloud_registered_raw", 1);
        refined_path_pub_ = nh.advertise<nav_msgs::Path>("icp_odometry/refined_path", 1);
    }

    void registerSubscribers(){
        laser_cloud_sub_ = nh.subscribe("points_raw", 5, &ICPRegistration::laserCloudCallback, this);
    }

    bool isOdomReady() const {
        return odom_inited_;
    }

    void setInitialPose(const Pose6DOF& initial_pose) {
        icp_odom_poses_.push_back(initial_pose);
        refined_poses.push_back(initial_pose);
        initial_pose_set_ = true;
    }

    Pose6DOF getFirstPose() const {
        return icp_odom_poses_.front();
    }

    Pose6DOF getLatestPose() const {
        return icp_odom_poses_.back();
    }

    void getEstimates(ros::Time& stamp, pcl::PointCloud<PointType>::Ptr& cloud, Pose6DOF& latest_icp_transform, Pose6DOF& icp_pose, bool& new_transform) {
        cloud = prev_cloud_;

        stamp = latest_stamp;
        latest_icp_transform = icp_latest_transform_;
        icp_pose = getLatestPose();
        new_transform = new_transform_;

        icp_latest_transform_.setIdentity();

        this->new_transform_ = false;
    }

    void voxelFilterCloud(pcl::PointCloud<PointType>::Ptr* input, pcl::PointCloud<PointType>::Ptr* output) {
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(*input);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(**output);
    }

    void publishPath(const ros::Time& stamp) {
        icp_odom_path_.header.stamp = stamp;
        icp_odom_path_.header.frame_id = "odom";
        icp_odom_path_pub_.publish(icp_odom_path_);
    }

    void publishPath(const Pose6DOF& latest_pose) {
        insertPoseInPath(latest_pose.toROSPose(), "odom", latest_stamp, refined_path_);
        refined_path_.header.stamp = latest_stamp;
        refined_path_.header.frame_id = "odom";
        refined_path_pub_.publish(refined_path_);
    }

    float pointDistance(PointType p)
    {
        return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    }


    float pointDistance(PointType p1, PointType p2)
    {
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    }

    // void addPointsToMap(pcl::PointCloud<PointType>::Ptr input_cloud) {
        
    //     for (size_t i = 0; i < input_cloud->points.size(); i++) {
    //         PointType point  = input_cloud->points[i];
    //         if (!map_octree_->isVoxelOccupiedAtPoint(point)) {
    //             map_octree_->addPointToCloud(point, map_cloud_);
    //         }
    //     }
    // }

    PointTypePose PosetoPointTypePose(const Pose6DOF& pose)
    {
        PointTypePose thisPose6D;
        thisPose6D.x = pose.pos(0);
        thisPose6D.y = pose.pos(1);
        thisPose6D.z = pose.pos(2);

        Eigen::Vector3d RPY = pose.rot.toRotationMatrix().eulerAngles(0,1,2);
        thisPose6D.roll  = RPY(0);
        thisPose6D.pitch = RPY(1);
        thisPose6D.yaw   = RPY(2);
        return thisPose6D;
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }


    void publishGlobalMap()
    {

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > 1500)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(CloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(&map_cloud_pub_, globalMapKeyFramesDS, latest_stamp, odometryFrame);
    }

    // bool approxNearestNeighbors(const pcl::PointCloud<PointType>::Ptr& cloud, pcl::PointCloud<PointType>::Ptr& nearest_neighbors) {
    //     nearest_neighbors->points.clear();
    //     ROS_INFO("CHEGUEI AQUI");
    //     // Iterate over points in the input point cloud, finding the nearest neighbor
    //     // for every point and storing it in the output array.
    //     for (size_t ii = 0; ii < cloud->points.size(); ++ii) {
    //         // Search for nearest neighbor and store.
    //         float unused = 0.0f;
    //         int result_index = -1;

    //         map_octree_->approxNearestSearch(cloud->points[ii], result_index, unused);
    //         if (result_index >= 0)
    //         nearest_neighbors->push_back(map_cloud_->points[result_index]);
    //     }

    //     return (nearest_neighbors->points.size() > 0);
    // }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr in_cloud, const Pose6DOF& transform) {

        Eigen::Matrix4d cloud_in_pose = transform.toEigenMatrix();

        pcl::PointCloud<PointType>::Ptr out_cloud(new pcl::PointCloud<PointType>());

        PointType *pointFrom;

        int cloudSize = in_cloud->size();
        out_cloud->resize(cloudSize);

        // Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

        Eigen::Affine3f transCur;
        
        transCur = Eigen::Affine3f(cloud_in_pose.cast<float>());

        // #pragma omp parallel for num_threads(4)
        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &in_cloud->points[i];
            out_cloud->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            out_cloud->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            out_cloud->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            // cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return out_cloud;
        // tf::Transform tf_cloud_in_pose = pose.toTFTransform();
        // try {
        //     pcl_ros::transformPointCloud(*in_cloud, *out_cloud, tf_cloud_in_pose);
        // } catch (tf::TransformException e) {
        // }
    }   

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        // #pragma omp parallel for num_threads(4)
        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    

    bool estimateTransformICP(const pcl::PointCloud<PointType>::Ptr& curr_cloud, const pcl::PointCloud<PointType>::Ptr& nn_cloud, Pose6DOF& transform) {
        ROS_INFO("Estimation of transform via ICP");
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
        icp.setMaximumIterations(ICP_MAX_ITERS);
        icp.setTransformationEpsilon(ICP_EPSILON);
        icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
        icp.setRANSACIterations(0);
        icp.setInputSource(curr_cloud);
        icp.setInputTarget(nn_cloud);

        pcl::PointCloud<PointType>::Ptr prev_cloud_in_curr_frame(new pcl::PointCloud<PointType>()),
            curr_cloud_in_prev_frame(new pcl::PointCloud<PointType>()), joint_cloud(new pcl::PointCloud<PointType>());
        icp.align(*curr_cloud_in_prev_frame);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();

        if (icp.hasConverged()) {
            ROS_INFO("ICP converged");
            transform = Pose6DOF(T, latest_stamp);
            
            // pcl::PointCloud<PointType>::Ptr cloud_in_map(new pcl::PointCloud<PointType>());
            // pcl::transformPointCloud(*curr_cloud, *cloud_in_map, T);
            // publishPointCloud(cloud_in_map, "odom", latest_stamp, &register_cloud_pub2);

            return true;
        }

        return false;
    }

    bool updateICPOdometry(const ros::Time& stamp, const Eigen::Matrix4d& T) {
        ROS_INFO("ICP odometry update!");
        Pose6DOF transform(T, stamp);
        Pose6DOF prev_pose = getLatestPose();
        Pose6DOF new_pose = prev_pose + transform;
        icp_latest_transform_ += transform;


        // std::cout << "#####		ICP odometry		#####" << std::endl;
        // std::cout << "Initial pose:\n" << getFirstPose().toStringQuat("   ");
        // std::cout << "Prev odometry pose:\n" << prev_pose.toStringQuat("   ");
        // std::cout << "Cloud transform = " << transform.toStringQuat("   ");
        // std::cout << "ICP odometry pose = " << new_pose.toStringQuat("   ");
        // std::cout << std::endl;

        if(transform.norm() < 0.00001)
        	return false;
        else
        {
            new_transform_ = true;
            icp_odom_poses_.push_back(new_pose);
            insertPoseInPath(new_pose.toROSPose(), "odom", stamp, icp_odom_path_);
    
            publishOdometry(new_pose.pos, new_pose.rot, "odom", "base_link", stamp, &icp_odom_pub_);
            publishPoseStamped(new_pose.pos, new_pose.rot, "odom", stamp, &icp_pose_pub_);
            publishPath(stamp);

            // Publish TF
            static tf::TransformBroadcaster br;
            Eigen::Vector3d RPY = new_pose.rot.toRotationMatrix().eulerAngles(0,1,2);
            tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(RPY(0), RPY(1), RPY(2)),
                                                        tf::Vector3(new_pose.pos(0), new_pose.pos(1), new_pose.pos(2)));
            tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, latest_stamp, "odom", "base_link");
            br.sendTransform(trans_odom_to_lidar);

            return true;
        }
    }

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
        pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>());

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        // Filter cloud
        voxelFilterCloud(&input_cloud, &curr_cloud_);

        if (prev_cloud_->points.size() == 0) {
            *prev_cloud_ = *curr_cloud_;
            return;
        }

        latest_stamp = cloud_msg->header.stamp;

        std::lock_guard<std::mutex> lock(mtx);
        // Registration GICP
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
        icp.setMaximumIterations(ICP_MAX_ITERS);
        icp.setTransformationEpsilon(ICP_EPSILON);
        icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
        icp.setRANSACIterations(0);
        icp.setInputSource(curr_cloud_);
        icp.setInputTarget(prev_cloud_);

        pcl::PointCloud<PointType>::Ptr prev_cloud_in_curr_frame(new pcl::PointCloud<PointType>()),
            curr_cloud_in_prev_frame(new pcl::PointCloud<PointType>()), joint_cloud(new pcl::PointCloud<PointType>());
        icp.align(*curr_cloud_in_prev_frame);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();

        if (icp.hasConverged() && icp.getFitnessScore() < 4) {
            ROS_INFO("ICP odometry converged");
            std::cout << "Estimated T:\n" << T << std::endl;
            
            Eigen::Matrix4d T_inv = T.inverse();
            pcl::transformPointCloud(*prev_cloud_, *prev_cloud_in_curr_frame, T_inv);
            bool success = updateICPOdometry(latest_stamp, T);
            if (success) {
                odom_inited_ = true;
                *prev_cloud_ = *curr_cloud_;
            }

            publishPointCloud(prev_cloud_, "base_link", latest_stamp, &prev_cloud_pub_);
            publishPointCloud(curr_cloud_in_prev_frame, "base_link", latest_stamp, &aligned_cloud_pub_);
        }
    }

    // bool refineTransformAndGrowMap(const ros::Time& stamp, const pcl::PointCloud<PointType>::Ptr& cloud, const Pose6DOF& raw_pose, Pose6DOF& transform) {
       
    //     pcl::PointCloud<PointType>::Ptr cloud_in_map(new pcl::PointCloud<PointType>());
    //     *cloud_in_map = *transformPointCloud(cloud, transform);

    //     if (map_cloud_->points.size() == 0) {
    //         // ROS_INFO("CHEGUEI AQUI");
    //         addPointsToMap(cloud_in_map);
    //         return false;
    //     }

    //     pcl::PointCloud<PointType>::Ptr nn_cloud_in_map(new pcl::PointCloud<PointType>());
    //     pcl::PointCloud<PointType>::Ptr nn_cloud(new pcl::PointCloud<PointType>());

    //     // Get closest points in map to current cloud via nearest-neighbors search
    //     mtx.lock();
    //     approxNearestNeighbors(cloud_in_map, nn_cloud_in_map);
    //     mtx.unlock();

    //     *nn_cloud = *transformPointCloud(nn_cloud_in_map, transform.inverse());
        
    //     publishPointCloud(nn_cloud, "base_link", stamp, &nn_cloud_pub_);
        
    //     ROS_INFO("CHEGUEI AQUI");
    //     if (estimateTransformICP(cloud, nn_cloud, transform)) {
    //         Pose6DOF refined_pose = raw_pose + transform;

    //         std::cout << "REFINED POSE" << std::endl;
    //         std::cout << "ICP odometry refined pose = " << refined_pose.toStringQuat("   ");
    //         std::cout << "Cloud transform = " << transform.toStringQuat("   ");

    //         *cloud_in_map = *transformPointCloud(cloud, transform);
    //         // addPointsToMap(cloud_in_map);
            
            
    //         // publishPointCloud(map_cloud_, "odom", stamp, &map_cloud_pub_);
    //         // publishPointCloud(cloud_in_map, "odom", stamp, &registered_cloud_pub_);
    //         publishPath(refined_pose);

    //         refined_poses.push_back(refined_pose);

    //         return true;
    //     }
       

    //     return false;
    // }
    // bool saveFrame()
    // {
    //     if (cloudKeyPoses3D->points.empty())
    //         return true;

    //     Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    //     Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
    //                                                         transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    //     Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    //     float x, y, z, roll, pitch, yaw;
    //     pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    //     if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
    //         abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
    //         abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
    //         sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
    //         return false;

    //     return true;
    // }

    void saveKeyFrames()
    {
        // if (saveFrame() == false)
        //     return;

        Pose6DOF pose = getLatestPose();
        //save key poses
        PointType     thisPose3D;
        PointTypePose thisPose6D;

        thisPose3D.x = pose.pos(0);
        thisPose3D.y = pose.pos(1);
        thisPose3D.z = pose.pos(2);
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;

        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index

        Eigen::Vector3d RPY = pose.rot.toRotationMatrix().eulerAngles(0,1,2);
        thisPose6D.roll   = RPY(0);
        thisPose6D.pitch  = RPY(1);
        thisPose6D.yaw    = RPY(2);

        double h_time = latest_stamp.toSec();
        thisPose6D.time = h_time;
        cloudKeyPoses6D->push_back(thisPose6D);

        // save all the received points for each curr frame
        pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*curr_cloud_,  *thisKeyFrame);


        // save key frame cloud
        CloudKeyFrames.push_back(thisKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
    }

    // void publishMapToOdomTf(const ros::Time& stamp) {
    //     geometry_msgs::TransformStamped map_to_odom_tf =
    //         getTfStampedFromEigenMatrix(stamp, T_map_to_odom_.matrix().cast<float>(), "map", "odom");
    //     tf_broadcaster.sendTransform(map_to_odom_tf);
    // }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses

        Pose6DOF pose = getLatestPose();
        std::cout << "PUBLISHING FRAMES" << endl;

        if (pubRecentKeyFrame.getNumSubscribers() != 0){
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = PosetoPointTypePose(pose);
            *cloudOut += *transformPointCloud(curr_cloud_,  &thisPose6D);
            publishCloud(&pubRecentKeyFrame, cloudOut, latest_stamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0){
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = PosetoPointTypePose(pose);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(&pubCloudRegisteredRaw, cloudOut, latest_stamp, odometryFrame);
        }
    }

    void main_loop(){
        ROS_INFO("Main loop started");

        unsigned long iter = 0;

        Pose6DOF icp_transform, icp_odom_pose, prev_icp_odom_pose, prev_icp_pose, refined_pose;
        bool new_transform_icp = false;

        // We start at the origin
        
        prev_icp_odom_pose = Pose6DOF::getIdentity();
        // prev_icp_pose = prev_icp_odom_pose;

        // ros::Time latest_stamp = ros::Time::now();
        
        ROS_INFO("Initial pose");
        setInitialPose(Pose6DOF::getIdentity());


        while (ros::ok()) {
            // std::cout << "Entrei no ROS::OK" << std::endl;
            // publishMapToOdomTf(latest_stamp);
            
            if (isOdomReady()) {
                // std::cout << "Tenho ODOM" << std::endl;
                pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
                getEstimates(latest_stamp, cloud, icp_transform, icp_odom_pose, new_transform_icp);

            //     std::cout << "ICP Tranform: " << icp_transform << std::endl;
            
                if (new_transform_icp) {

                    // Pose6DOF refined_transform;
                    // ROS_INFO("Transform refinement using nearest neighbor search");

                    // bool registration_success = refineTransformAndGrowMap(latest_stamp, cloud, prev_icp_odom_pose, refined_transform);
                    // if (registration_success) {
                    //     ROS_INFO("Refinement successful");
                    //     icp_transform = refined_transform;
                    // }
                    // icp_odom_pose = prev_icp_odom_pose + icp_transform;

                    // refined_pose = prev_icp_odom_pose + refined_transform;

                    saveKeyFrames();
                    publishFrames();
                    publishGlobalMap();

                    new_transform_icp = false;
                }

                // prev_icp_odom_pose = icp_odom_pose;
            }

            iter++;
            ros::spinOnce();
        }
    }
    // ONLY USE THIS IF THERE IS a GLOBAL MAP OPTIMIZER LIKE POSE GRAPH 
    // void computeMapToOdomTransform() { 
    //     if (keyframes_list_.size() > 0) {
    //         const auto& keyframe = keyframes_list_.back();
    //         T_map_to_odom_ = keyframe->graph_node_->estimate() * keyframe->pose_in_odom_.toEigenIsometry3().inverse();
    //     } else {
    //         T_map_to_odom_ = Eigen::Isometry3d::Identity();
    //     }
    // }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_registration_node");
    ROS_INFO("Starting ICP");
    ICPRegistration icp;
    icp.main_loop();   
}