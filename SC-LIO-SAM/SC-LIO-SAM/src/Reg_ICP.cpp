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

class ICPRegistration : public ParamServer
{

private:
    
    // Constants
    const double ICP_FITNESS_THRESH = 0.1;
    const double ICP_MAX_CORR_DIST = 1.0;
    const double ICP_EPSILON = 1e-06;
    const double ICP_MAX_ITERS = 10; // 50

    tf2_ros::TransformBroadcaster tf_broadcaster;
    Eigen::Isometry3d T_map_to_odom_;
    // bool publish_map_transform_;

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

    // Odometry path containers
    nav_msgs::Odometry icp_odom_;
    nav_msgs::Path icp_odom_path_;
    nav_msgs::Path refined_path_;

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

    // PCL clouds for mapping
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map_octree_;

public:
    ICPRegistration(): 
        T_map_to_odom_(Eigen::Isometry3d::Identity()),
        odom_inited_(false),
        num_clouds_skip_(0),
        voxel_leaf_size_(0.2)
        // map_octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.5))
    {
        allocateMemory();
        init();
    }
    
    ~ICPRegistration(){}

    void allocateMemory()
    {   
        prev_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        curr_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        map_octree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.5));
        map_octree_->setInputCloud(map_cloud_);
    }

    void init() {
        // Nao e necessario dar load de parametros porque o utility.h ja tem os parametros

        advertisePublishers();
        registerSubscribers();

        ROS_INFO("ICP odometry initialized");
    }   

    void advertisePublishers(){
        // ICP topics
        icp_odom_pub_ = nh.advertise<nav_msgs::Odometry>("icp_odometer/odom", 1);
        icp_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("icp_odometer/pose", 1);
        icp_odom_path_pub_ = nh.advertise<nav_msgs::Path>("icp_odometer/path", 1);

        // ICP odometry debug topics

        prev_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometer/prev_cloud", 1);
        aligned_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("icp_odometer/aligned_cloud", 1);

        // Map topics
        map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("octree_mapper/map_cloud", 1);
        nn_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("octree_mapper/nn_cloud", 1);
        registered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("octree_mapper/registered_cloud", 1);
        refined_path_pub_ = nh.advertise<nav_msgs::Path>("octree_mapper/refined_path", 1);
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

    void publishPath(const Pose6DOF& latest_pose) {
        insertPoseInPath(latest_pose.toROSPose(), "map", ros::Time().now(), refined_path_);
        refined_path_.header.stamp = ros::Time().now();
        refined_path_.header.frame_id = "map";
        refined_path_pub_.publish(refined_path_);
    }

    void addPointsToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
        
        // *map_cloud_ += *input_cloud;
        // map_octree_->setInputCloud(map_cloud_);
        // map_octree_->addPointsFromInputCloud();
        for (size_t i = 0; i < input_cloud->points.size(); i++) {
            pcl::PointXYZ point  = input_cloud->points[i];
            if (!map_octree_->isVoxelOccupiedAtPoint(point)) {
                ROS_INFO("ANTES");
                map_octree_->addPointToCloud(point, map_cloud_);
                ROS_INFO("DEPOIS");
            }
        }
    }

    bool approxNearestNeighbors(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& nearest_neighbors) {
        nearest_neighbors->points.clear();
        ROS_INFO("CHEGUEI AQUI");
        // Iterate over points in the input point cloud, finding the nearest neighbor
        // for every point and storing it in the output array.
        for (size_t ii = 0; ii < cloud->points.size(); ++ii) {
            // Search for nearest neighbor and store.
            float unused = 0.0f;
            int result_index = -1;

            map_octree_->approxNearestSearch(cloud->points[ii], result_index, unused);
            if (result_index >= 0)
            nearest_neighbors->push_back(map_cloud_->points[result_index]);
        }

        return (nearest_neighbors->points.size() > 0);
    }

    void transformCloudToPoseFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud, const Pose6DOF& pose, pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud) {
        tf::Transform tf_cloud_in_pose = pose.toTFTransform();
        try {
            pcl_ros::transformPointCloud(*in_cloud, *out_cloud, tf_cloud_in_pose);
        } catch (tf::TransformException e) {
        }
    }   

    bool estimateTransformICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& curr_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& nn_cloud, Pose6DOF& transform) {
        // ROS_INFO("Estimation of transform via ICP");
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations(ICP_MAX_ITERS);
        icp.setTransformationEpsilon(ICP_EPSILON);
        icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DIST);
        icp.setRANSACIterations(0);
        icp.setInputSource(curr_cloud);
        icp.setInputTarget(nn_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_in_curr_frame(new pcl::PointCloud<pcl::PointXYZ>()),
            curr_cloud_in_prev_frame(new pcl::PointCloud<pcl::PointXYZ>()), joint_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        icp.align(*curr_cloud_in_prev_frame);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();

        if (icp.hasConverged()) {
            ROS_INFO("ICP converged");
            transform = Pose6DOF(T, latest_stamp);
            return true;
        }

        return false;
    }

    bool refineTransformAndGrowMap(const ros::Time& stamp, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Pose6DOF& raw_pose, Pose6DOF& transform) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_map(new pcl::PointCloud<pcl::PointXYZ>());
        transformCloudToPoseFrame(cloud, raw_pose, cloud_in_map);

        if (map_cloud_->points.size() == 0) {
            // ROS_WARN("IcpSlam: Octree map is empty!");
            ROS_INFO("CHEGUEI AQUI");
            addPointsToMap(cloud_in_map);
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr nn_cloud_in_map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr nn_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // Get closest points in map to current cloud via nearest-neighbors search
        approxNearestNeighbors(cloud_in_map, nn_cloud_in_map);
        transformCloudToPoseFrame(nn_cloud_in_map, raw_pose.inverse(), nn_cloud);

        publishPointCloud(nn_cloud, "base_link", stamp, &nn_cloud_pub_);
        // if (nn_cloud_pub_.getNumSubscribers() > 0) {
        //     publishPointCloud(nn_cloud, "base_link", stamp, &nn_cloud_pub_);
        // }
        publishPointCloud(map_cloud_, "map", stamp, &map_cloud_pub_);
        publishPointCloud(cloud_in_map, "map", stamp, &registered_cloud_pub_);
        
        if (estimateTransformICP(cloud, nn_cloud, transform)) {
            Pose6DOF refined_pose = raw_pose + transform;
            transformCloudToPoseFrame(cloud, refined_pose, cloud_in_map);
            addPointsToMap(cloud_in_map);
            
            // publishPointCloud(map_cloud_, "map", stamp, &map_cloud_pub_);
            publishPath(refined_pose);
            
            // if (map_cloud_pub_.getNumSubscribers() > 0) {
            // publishPointCloud(map_cloud_, "map", stamp, &map_cloud_pub_);
            // }
            // if (refined_path_pub_.getNumSubscribers() > 0) {
            // publishPath(refined_pose);
            // }
            // if (registered_cloud_pub_.getNumSubscribers() > 0) {
            // publishPointCloud(cloud_in_map, "map", stamp, &registered_cloud_pub_);
            // }
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
            insertPoseInPath(new_pose.toROSPose(), "odom", stamp, icp_odom_path_);

            publishOdometry(new_pose.pos, new_pose.rot, "odom", "base_link", stamp, &icp_odom_pub_);
            publishPoseStamped(new_pose.pos, new_pose.rot, "odom", stamp, &icp_pose_pub_);
            publishPath(stamp);
            // if (icp_odom_pub_.getNumSubscribers() > 0) {
            // publishOdometry(new_pose.pos, new_pose.rot, "odom", "base_link", stamp, &icp_odom_pub_);
            // }
            // if (icp_pose_pub_.getNumSubscribers() > 0) {
            // publishPoseStamped(new_pose.pos, new_pose.rot, "odom", stamp, &icp_pose_pub_);
            // }
            // if (icp_odom_path_pub_.getNumSubscribers() > 0) {
            // publishPath(stamp);
            // }

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        // Filter cloud
        voxelFilterCloud(&input_cloud, &curr_cloud_);

        if (prev_cloud_->points.size() == 0) {
            *prev_cloud_ = *curr_cloud_;
            return;
        }

        latest_stamp = cloud_msg->header.stamp;

        // Registration GICP
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

        if (icp.hasConverged() && icp.getFitnessScore() < 10) {
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
            // if (prev_cloud_pub_.getNumSubscribers() > 0) {
            //     publishPointCloud(prev_cloud_, "base_link", latest_stamp, &prev_cloud_pub_);
            // }
            // if (aligned_cloud_pub_.getNumSubscribers() > 0) {
            //     publishPointCloud(curr_cloud_in_prev_frame, "base_link", latest_stamp, &aligned_cloud_pub_);
            // }
        }
    }
    
    void main_loop(){
        ROS_INFO("Main loop started");

        unsigned long iter = 0;

        Pose6DOF icp_transform, icp_odom_pose, prev_icp_odom_pose, prev_icp_pose;
        bool new_transform_icp = false;

        // We start at the origin
        
        prev_icp_odom_pose = Pose6DOF::getIdentity();
        prev_icp_pose = prev_icp_odom_pose;

        // ros::Time latest_stamp = ros::Time::now();
        
        ROS_INFO("Initial pose");
        setInitialPose(Pose6DOF::getIdentity());



        while (ros::ok()) {
            // std::cout << "Entrei no ROS::OK" << std::endl;
            publishMapToOdomTf(latest_stamp);
            
            if (isOdomReady()) {
                std::cout << "Tenho ODOM" << std::endl;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                getEstimates(latest_stamp, cloud, icp_transform, icp_odom_pose, new_transform_icp);

            //     std::cout << "ICP Tranform: " << icp_transform << std::endl;

                if (new_transform_icp) {
                     std::cout << "Tenho ICP Transform" << std::endl;
                    Pose6DOF refined_transform;
                    ROS_INFO("Transform refinement using nearest neighbor search");
                    bool registration_success = refineTransformAndGrowMap(latest_stamp, cloud, prev_icp_odom_pose, refined_transform);
                    if (registration_success) {
                        ROS_INFO("Refinement successful");
                        icp_transform = refined_transform;
                    }
                    icp_odom_pose = prev_icp_odom_pose + icp_transform;
                    new_transform_icp = false;
                }

                prev_icp_odom_pose = icp_odom_pose;
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
    void publishMapToOdomTf(const ros::Time& stamp) {
        geometry_msgs::TransformStamped map_to_odom_tf =
            getTfStampedFromEigenMatrix(stamp, T_map_to_odom_.matrix().cast<float>(), "map", "odom");
        tf_broadcaster.sendTransform(map_to_odom_tf);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_registration_node");
    ROS_INFO("Starting ICP");
    ICPRegistration icp;
    icp.main_loop();   
}