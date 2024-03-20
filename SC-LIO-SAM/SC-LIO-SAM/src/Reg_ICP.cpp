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
    const double ICP_MAX_CORR_DIST = 5.0;
    const double ICP_EPSILON = 1e-6; //1e-6
    const double ICP_MAX_ITERS = 10; // 10

    tf2_ros::TransformBroadcaster tf_broadcaster;
    Eigen::Isometry3d T_map_to_odom_;
    // bool publish_map_transform_;

    // Mutex
    std::mutex mtx;

    // ROS node handle, URDF frames, topics and publishers
    ros::Publisher map_cloud_pub;
    ros::Publisher aligned_cloud_pub_;
    ros::Publisher curr_cloud_pub_;
    ros::Publisher prev_cloud_pub_;
    ros::Publisher icp_odom_pub_;
    ros::Publisher icp_pose_pub_;
    ros::Publisher icp_odom_path_pub_;
    
    // Subscriber ouster pcd's
    ros::Subscriber laser_cloud_sub_;
    
    // Publishers for map cloud, registered clouds and path refined
    ros::Publisher map_cloud_pub_;
    ros::Publisher nn_cloud_pub_;
    ros::Publisher registered_cloud_pub_;
    // ros::Publisher refined_path_pub_;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubCloudRegisteredDS;
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
    bool initialize;

    // Cloud skipping and filtering
    int clouds_skipped_;
    int num_clouds_skip_;
    double voxel_leaf_size_;
    int iter;
    // Transforms and intermediate poses
    ros::Time latest_stamp;
    Pose6DOF icp_latest_transform_;
    Pose6DOF prev_icp_odom;
    std::vector<Pose6DOF> icp_odom_poses_;
    // std::vector<Pose6DOF> refined_poses;
    
    // PCL clouds for ICP
    pcl::PointCloud<PointType>::Ptr prev_cloud_;
    pcl::PointCloud<PointType>::Ptr curr_cloud_;
    pcl::PointCloud<PointType>::Ptr raw_cloud;

    // pcl::PointCloud<PointType>::Ptr curKeyframeCloud;
    // pcl::PointCloud<PointType>::Ptr nearKeyframeCloud;
    pcl::PointCloud<PointType>::Ptr curr_cloud_in_prev_frame;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    // Parameters for voxelization
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    // PCL clouds for mapping
    pcl::PointCloud<PointType>::Ptr map_cloud_;

    vector<pcl::PointCloud<PointType>::Ptr> CloudKeyFrames;
    

public:
    ICPRegistration(): 
        // T_map_to_odom_(Eigen::Isometry3d::Identity()),
        odom_inited_(false),
        initialize(false),
        num_clouds_skip_(0),
        voxel_leaf_size_(0.1),
        iter(0),
        icp_latest_transform_(Pose6DOF::getIdentity()),
        prev_icp_odom(Pose6DOF::getIdentity())
        
    {
        allocateMemory();
        init();
    }
    
    ~ICPRegistration(){}

    void allocateMemory()
    {   
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        // copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        // copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());

        prev_cloud_.reset(new pcl::PointCloud<PointType>());
        curr_cloud_.reset(new pcl::PointCloud<PointType>());
        map_cloud_.reset(new pcl::PointCloud<PointType>());
        raw_cloud.reset(new pcl::PointCloud<PointType>());
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

        map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/map_cloud_ICP", 1);
        aligned_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/aligned_cloud", 1);
        curr_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/curr_cloud", 1);
        prev_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/prev_cloud", 1);

        // Map topics

        map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/map_cloud", 1);
        // nn_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/nn_cloud", 1);
        registered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/registered_cloud_odom_frame", 1);

        pubCloudRegisteredDS     = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/cloud_registered_raw", 1);
        // refined_path_pub_     = nh.advertise<nav_msgs::Path>("icp_odometry/refined_path", 1);
        // pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("icp_odometry/history_cloud", 1);
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

    void getEstimates(ros::Time& stamp, pcl::PointCloud<PointType>::Ptr& cloud, Pose6DOF& latest_icp_transform, Pose6DOF& icp_pose, bool& new_transform) {
        cloud = curr_cloud_;
        
        // cloud = map_cloud_;

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

    // void publishPath(const Pose6DOF& latest_pose) {
    //     insertPoseInPath(latest_pose.toROSPose(), "odom", latest_stamp, refined_path_);
    //     refined_path_.header.stamp = latest_stamp;
    //     refined_path_.header.frame_id = "odom";
    //     refined_path_pub_.publish(refined_path_);
    // }

    float pointDistance(PointType p)
    {
        return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    }


    float pointDistance(PointType p1, PointType p2)
    {
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    }

    PointTypePose PosetoPointTypePose(const Pose6DOF& pose)
    {
        PointTypePose thisPose6D;
        thisPose6D.x = pose.pos(0);
        thisPose6D.y = pose.pos(1);
        thisPose6D.z = pose.pos(2);

        pose.rot.normalized();
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
        pose_stamped.header.frame_id = "odom";
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

    void addPointsToMap(pcl::PointCloud<PointType>::Ptr &input_cloud) {
        // Concatenate the input_cloud with the map_cloud
        *map_cloud_ += *input_cloud;

        // Remove duplicate points in the merged point cloud
        pcl::PointCloud<PointType>::Ptr merged_cloud(new pcl::PointCloud<PointType>);
        pcl::VoxelGrid<PointType> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(map_cloud_);
        voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2); // Adjust the leaf size as needed
        voxel_grid_filter.filter(*merged_cloud);

        // Update the map_cloud_ with the filtered merged cloud
        map_cloud_ = merged_cloud;
    }

    void performMapICP(const ros::Time& stamp, const pcl::PointCloud<PointType>::Ptr& cloud, Pose6DOF& prev_pose, Pose6DOF & transform) {
        
        pcl::PointCloud<PointType>::Ptr cloud_in_map(new pcl::PointCloud<PointType>());
        // Pose6DOF transform;
        // Pose6DOF prev_pose = getLatestPose();

        transformCloudToPoseFrame(cloud, prev_pose, cloud_in_map);
        
        if (map_cloud_->points.size() == 0) {
            // ROS_INFO("CHEGUEI AQUI");
            addPointsToMap(cloud_in_map);

            return;
        }
        
        std::lock_guard<std::mutex> lock(mtx);

        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(ICP_EPSILON);
        icp.setMaxCorrespondenceDistance(150);
        icp.setRANSACIterations(0);
        icp.setInputSource(cloud_in_map);
        icp.setInputTarget(map_cloud_);

        pcl::PointCloud<PointType>::Ptr prev_cloud_in_curr_frame(new pcl::PointCloud<PointType>()),
            curr_cloud_in_prev_frame(new pcl::PointCloud<PointType>()), joint_cloud(new pcl::PointCloud<PointType>());
        icp.align(*curr_cloud_in_prev_frame);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();

        std::cout << "MAP ICP FITNESS SCORE : " << icp.getFitnessScore() << std::endl;

        if (icp.hasConverged() && icp.getFitnessScore() < 0.1) {
            ROS_INFO("ICP converged");
        
            bool success = updateICPOdometry(latest_stamp, T);
            if (success) {
                odom_inited_ = true;
                *prev_cloud_ = *curr_cloud_;
            }
            
            transform = Pose6DOF(T, latest_stamp);

            if (registered_cloud_pub_.getNumSubscribers() > 0) {
                publishPointCloud(cloud_in_map, "odom", stamp, &registered_cloud_pub_);
            }

            if (map_cloud_pub.getNumSubscribers() != 0){
                publishPointCloud(map_cloud_, "odom", stamp, &map_cloud_pub);
            }
        }
        
        Pose6DOF new_pose = prev_pose + transform;
            
        std::cout << "ICP odometry refined pose = " << new_pose.toStringQuat("   ");
        std::cout << "Cloud transform = " << transform.toStringQuat("   ");

        transformCloudToPoseFrame(prev_cloud_, new_pose, cloud_in_map);
        addPointsToMap(cloud_in_map);
    }

    void performICP(const ros::Time& stamp, Pose6DOF& pose){

        std::cout << " Perform ICP" << std::endl;
        std::cout << "STAMP FOR COARSE ICP: " << latest_stamp << std::endl;

        std::lock_guard<std::mutex> lock(mtx);
        
        // Registration GICP
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaximumIterations(ICP_MAX_ITERS);
        icp.setTransformationEpsilon(ICP_EPSILON);
        icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
        icp.setRANSACIterations(0);
        icp.setInputSource(curr_cloud_);
        icp.setInputTarget(prev_cloud_);

        pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>()), cloud_in_map(new pcl::PointCloud<PointType>());

        icp.align(*aligned_cloud);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();
        

        std::cout << "ICP FITNESS SCORE: " << icp.getFitnessScore() << std::endl;

        if (icp.hasConverged() && icp.getFitnessScore() < 3) { // fitness score a 3 funciona bem (dentro do possivel) com bag rate 0.5
            ROS_INFO("ICP odometry converged");
            // std::cout << "ICP Fitness score: " << icp.getFitnessScore() << std::endl;
            // std::cout << "Estimated T:\n" << T << std::endl;v

            // odom_inited_ = true;
            *prev_cloud_ = *curr_cloud_;
            // Pose6DOF transform;


            icp_latest_transform_ = Pose6DOF(T,stamp);
            Pose6DOF prev_pose = getLatestPose();
            pose = prev_pose + icp_latest_transform_;

            // bool success = updateICPOdometry(latest_stamp, T);
            // if (success) {
            //     odom_inited_ = true;
            //     *prev_cloud_ = *curr_cloud_;
            // }


            transformCloudToPoseFrame(prev_cloud_, T, aligned_cloud);

            // if (prev_cloud_pub_.getNumSubscribers() != 0){
            //     publishPointCloud(prev_cloud_, "base_link", stamp, &map_cloud_pub);
            // }

            if (aligned_cloud_pub_.getNumSubscribers() != 0){
                publishPointCloud(aligned_cloud, "base_link", stamp, &aligned_cloud_pub_);
            }

            if (curr_cloud_pub_.getNumSubscribers() != 0){
                publishPointCloud(curr_cloud_, "base_link", stamp, &curr_cloud_pub_);
            }
        }
       
    }

    void publishGlobalMap() // const ros::Time& stamp
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

        publishCloud(&map_cloud_pub_, globalMapKeyFramesDS, latest_stamp, "odom");
    }

    void transformCloudToPoseFrame(const pcl::PointCloud<PointType>::Ptr& in_cloud, const Pose6DOF& pose, pcl::PointCloud<PointType>::Ptr& out_cloud) {
        tf::Transform tf_cloud_in_pose = pose.toTFTransform();
        try {
            pcl_ros::transformPointCloud(*in_cloud, *out_cloud, tf_cloud_in_pose);
        } catch (tf::TransformException& e) {
        }
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

    bool estimateTransformICP(const pcl::PointCloud<PointType>::Ptr& cloud_in_map, const pcl::PointCloud<PointType>::Ptr& map_cloud, Pose6DOF& transform) {
        // ROS_INFO("Estimation of transform via ICP");

        std::lock_guard<std::mutex> lock(mtx);
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
        icp.setMaximumIterations(30);
        icp.setTransformationEpsilon(ICP_EPSILON);
        icp.setMaxCorrespondenceDistance(150);
        icp.setRANSACIterations(0);
        icp.setInputSource(cloud_in_map);
        icp.setInputTarget(map_cloud);

        pcl::PointCloud<PointType>::Ptr prev_cloud_in_curr_frame(new pcl::PointCloud<PointType>()),
            curr_cloud_in_prev_frame(new pcl::PointCloud<PointType>()), joint_cloud(new pcl::PointCloud<PointType>());
        icp.align(*curr_cloud_in_prev_frame);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();

        std::cout << "MAP ICP FITNESS SCORE : " << icp.getFitnessScore() << std::endl;

        if (icp.hasConverged() && icp.getFitnessScore() < 0.1) {
            ROS_INFO("ICP converged");
        
            bool success = updateICPOdometry(latest_stamp, T);
            if (success) {
                odom_inited_ = true;
                *prev_cloud_ = *curr_cloud_;
            }
            transform = Pose6DOF(T, latest_stamp);

            if (registered_cloud_pub_.getNumSubscribers() > 0) {
                publishPointCloud(cloud_in_map, "odom", latest_stamp, &registered_cloud_pub_);
            }

            if (map_cloud_pub.getNumSubscribers() != 0){
                publishPointCloud(map_cloud_, "odom", latest_stamp, &map_cloud_pub);
            }
            return true;

            if (prev_cloud_pub_.getNumSubscribers() != 0){
                publishPointCloud(prev_cloud_, "base_link", latest_stamp, &map_cloud_pub);
            }
        }

        return false;
    }

    bool updateICPOdometry(const ros::Time& stamp, const Eigen::Matrix4d& T) {  // const Eigen::Matrix4d& T  Pose6DOF& transform
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

            tf::Transform t_odom_to_lidar = new_pose.toTFTransform();
            tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, stamp, "odom", "base_link");

            br.sendTransform(trans_odom_to_lidar);

            return true;
        }
    }

    void laserCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // ROS_INFO("Cloud callback!");
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
        *raw_cloud = *input_cloud;

        // Filter cloud
        voxelFilterCloud(&input_cloud, &curr_cloud_);

        latest_stamp = cloud_msg->header.stamp;
        // mtx.lock();
       
        // Pose6DOF pose;

        // pose = getLatestPose();
        Pose6DOF icp_odom_pose, transform, pose, pose_ICP;
        pose = getLatestPose();

        std::cout << "ITERAÇÃO: " << iter << std::endl;
        if(iter == 0){
            *prev_cloud_ = *curr_cloud_;

            pcl::PointCloud<PointType>::Ptr cloud_in_map(new pcl::PointCloud<PointType>());
            transformCloudToPoseFrame(curr_cloud_, pose, cloud_in_map);
            addPointsToMap(cloud_in_map);
            iter++;
            return;
        }
        if(iter >= 1){
            
            performICP(latest_stamp, pose);

            bool registration_success = refineTransformAndGrowMap(latest_stamp, curr_cloud_, prev_icp_odom, transform);

            if(registration_success){
                std::cout << "SUCESSO NA REFINE" << std::endl;
                icp_latest_transform_ = transform;
            }

            else{
                pcl::PointCloud<PointType>::Ptr cloud_in_map(new pcl::PointCloud<PointType>());
                transformCloudToPoseFrame(curr_cloud_, pose, cloud_in_map);    
                addPointsToMap(cloud_in_map);
            }

            // icp_latest_transform_ = transform;
            pose = prev_icp_odom + icp_latest_transform_;

            // pose = prev_icp_odom + icp_latest_transform_;
            saveKeyFrames(latest_stamp, pose);
            publishFrames(latest_stamp, pose);


            prev_icp_odom = pose;
            iter++;
        }

        // if (map_cloud_->points.size() == 0) {
        //     *prev_cloud_ = *curr_cloud_;

        //     pcl::PointCloud<PointType>::Ptr cloud_in_map(new pcl::PointCloud<PointType>());
        //     transformCloudToPoseFrame(prev_cloud_, pose, cloud_in_map);
        //     addPointsToMap(cloud_in_map);
        //     iter++;
        //     return;
        // }

        

        // performMapICP(latest_stamp, prev_cloud_, prev_icp_odom, transform);

        // refineTransformAndGrowMap(latest_stamp, prev_cloud_, prev_icp_odom, refined_transform);

       
        
    }

    bool refineTransformAndGrowMap(const ros::Time& stamp, const pcl::PointCloud<PointType>::Ptr& cloud, const Pose6DOF& prev_pose, Pose6DOF& transform) {
        
        pcl::PointCloud<PointType>::Ptr cloud_in_map(new pcl::PointCloud<PointType>());
        
        transformCloudToPoseFrame(cloud, prev_pose, cloud_in_map);
        
        // if (map_cloud_->points.size() == 0) {
        //     // ROS_INFO("CHEGUEI AQUI");
        //     addPointsToMap(cloud_in_map);
        //     return false;
        // }

        if (estimateTransformICP(cloud_in_map, map_cloud_, transform)) {
            Pose6DOF new_pose = prev_pose + transform;

            std::cout << "STAMP FOR FINE ICP: " << latest_stamp << std::endl;
            // std::cout << "REFINED POSE" << std::endl;
            std::cout << "ICP odometry refined pose = " << new_pose.toStringQuat("   ");
            std::cout << "Cloud transform = " << transform.toStringQuat("   ");

            // pcl::PointCloud<PointType>::Ptr aligned_cloud_in_map(new pcl::PointCloud<PointType>());

            transformCloudToPoseFrame(cloud, new_pose, cloud_in_map);
            addPointsToMap(cloud_in_map);
            
            if (registered_cloud_pub_.getNumSubscribers() > 0) {
                publishPointCloud(cloud_in_map, "odom", stamp, &registered_cloud_pub_);
            }

            if (map_cloud_pub.getNumSubscribers() != 0){
                publishPointCloud(map_cloud_, "odom", stamp, &map_cloud_pub);
            }
            return true;
        }
        return false;
    }
    
    void saveKeyFrames(const ros::Time& stamp, const Pose6DOF& pose) // const ros::Time& stamp, const Pose6DOF& pose
    {
        // if (saveFrame() == false)
        //     return;

        // Pose6DOF pose = getLatestPose();

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

        // std::cout << "POINTCLOUD PITCH: " << thisPose6D.pitch*180/M_PI << std::endl; 
        
        double h_time = stamp.toSec();
        thisPose6D.time = h_time;
        cloudKeyPoses6D->push_back(thisPose6D);

        // save all the received points for each curr frame
        pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*curr_cloud_,  *thisKeyFrame);


        // save key frame cloud
        CloudKeyFrames.push_back(thisKeyFrame);

        // Determine the starting index based on the last 30 keyframes
        // int startIndex = std::max(0, static_cast<int>(CloudKeyFrames.size()) - 10);
        // for (int i = startIndex; i < static_cast<int>(CloudKeyFrames.size()); ++i) {
        //     const auto& keyFrame = CloudKeyFrames[i];
        //     *map_cloud_ += *keyFrame;
        // }
        // save path for visualization
        updatePath(thisPose6D);
    }

    // void publishMapToOdomTf(const ros::Time& stamp) {
    //     geometry_msgs::TransformStamped map_to_odom_tf =
    //         getTfStampedFromEigenMatrix(stamp, T_map_to_odom_.matrix().cast<float>(), "map", "odom");
    //     tf_broadcaster.sendTransform(map_to_odom_tf);
    // }

    void publishFrames(const ros::Time& stamp, const Pose6DOF& pose) // const ros::Time& stamp, const Pose6DOF& pose
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses

        // Pose6DOF pose = getLatestPose();
        // std::cout << "PUBLISHING FRAMES" << endl;

        // publish registered cloud already voxelized

        if (pubCloudRegisteredDS.getNumSubscribers() != 0){
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = PosetoPointTypePose(pose);
            *cloudOut += *transformPointCloud(curr_cloud_,  &thisPose6D);
            publishCloud(&pubCloudRegisteredDS, cloudOut, stamp, "odom");
        }
        
        // publish registered high-res raw cloud

        if (pubCloudRegisteredRaw.getNumSubscribers() != 0){
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = PosetoPointTypePose(pose);
            *cloudOut = *transformPointCloud(raw_cloud,  &thisPose6D);
            publishCloud(&pubCloudRegisteredRaw, cloudOut, stamp, "odom");
        }
    }

    void savePointCloud()
    {
        // save final point cloud and trajectory

        std::cout << "SAVING PCDS" << endl;
        string mapString = "/home/joaojorge/Documents/map.pcd";
        string trajectoryString = "/home/joaojorge/Documents/trajectory.pcd";

        pcl::PointCloud<PointType>::Ptr mapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr mapCloudDS(new pcl::PointCloud<PointType>());
        
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
            *mapCloud  += *transformPointCloud(CloudKeyFrames[i],   &cloudKeyPoses6D->points[i]);
            std::cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }

        downSizeFilterMap.setInputCloud(mapCloud);
        downSizeFilterMap.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilterMap.filter(*mapCloudDS);


        pcl::io::savePCDFileASCII(mapString, *mapCloud);
        pcl::io::savePCDFileASCII(trajectoryString, *cloudKeyPoses3D);

        std::cout << "****************************************************" << std::endl;
        std::cout << "Saving map to pcd files completed" << std::endl;
    }

    //   void visualizeGlobalMapThread()
    // {
    //     //
    //     ros::Rate rate(0.2);
    //     while (ros::ok()){
    //         rate.sleep();
    //         publishGlobalMap(latest_stamp);
    //     }
    // }

    void main_loop(){
        ROS_INFO("Main loop started");

        // unsigned long iter = 0;

        Pose6DOF icp_transform, icp_odom_pose, prev_icp_odom_pose, prev_icp_pose, refined_pose;
        bool new_transform_icp = false;

        // We start at the origin
        
        prev_icp_odom_pose = Pose6DOF::getIdentity();
        // prev_icp_pose = prev_icp_odom_pose;

        // ros::Time latest_stamp = ros::Time::now();
        
        // ROS_INFO("Initial pose");
        setInitialPose(Pose6DOF::getIdentity());


        while (ros::ok()) {
            // publishMapToOdomTf(latest_stamp);
            if (isOdomReady()) {
               
                pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
                getEstimates(latest_stamp, cloud, icp_transform, icp_odom_pose, new_transform_icp);

                //     std::cout << "ICP Tranform: " << icp_transform << std::endl;
                
                if (new_transform_icp) {

               
                    // std::cout << "STAMP FOR FINE ICP: " << latest_stamp << std::endl;

                

                    // icp_transform = refined_transform;
                    // icp_odom_pose = prev_icp_odom_pose + icp_transform;

                    // saveKeyFrames(latest_stamp,icp_odom_pose);
                    // publishFrames(latest_stamp,icp_odom_pose);

                    // Pose6DOF refined_transform;
                    // bool registration_sucess = refineTransformAndGrowMap(latest_stamp, cloud, prev_icp_odom_pose, refined_transform);
                    // if (registration_sucess) {
                    //     ROS_INFO("Refinement successful");
                    //     icp_transform = refined_transform; 
                    // }
                    // // 
                    // icp_odom_pose = prev_icp_odom_pose + icp_transform;
                    // bool success = updateICPOdometry(latest_stamp, icp_transform);
                    // if (success){
                        
                    // }
                    

                    publishGlobalMap();

                    new_transform_icp = false;
                }
                // prev_icp_odom_pose = icp_odom_pose;
            }

            // iter++;
            ros::spinOnce();
        }
        // savePointCloud();
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