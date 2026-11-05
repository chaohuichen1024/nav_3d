/*
 * @Descripttion: 
 * @FilePath: /src/robot_navigation/octo_planner_3d/src/octomap_builder_node.cpp
 * @Author: chaohui_chen1024
 * @Date: 2026-07-06 18:01:06
 * @LastEditors: chaohui_chen1024
 * @LastEditTime: 2026-07-08 11:49:57
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcd2octomap_converter.h>

class OctoMapBuilderNode
{
public:
    OctoMapBuilderNode()
        : nh_("~"), nh_global_(), converter_(), map_ready_(false), 
          realtime_mode_(false), downsample_leaf_size_(0.1), 
          update_counter_(0), update_interval_(5)
    {
        nh_.param<double>("resolution", resolution_, 0.2);
        nh_.param<int>("min_points_per_voxel", min_points_per_voxel_, 3);
        nh_.param<int>("min_cluster_voxels", min_cluster_voxels_, 4);
        nh_.param<std::string>("input_pcd_path", input_pcd_path_, "");
        nh_.param<bool>("load_pcd_on_start", load_pcd_on_start_, false);
        nh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "/cloud_registered");
        nh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap");
        nh_.param<std::string>("occupied_voxels_topic", occupied_voxels_topic_, "/occupied_voxels");
        nh_.param<double>("downsample_leaf_size", downsample_leaf_size_, 0.1);
        nh_.param<bool>("realtime_mode", realtime_mode_, true);

        converter_.setParams(resolution_, min_points_per_voxel_, min_cluster_voxels_);
        converter_.setRealtimeMode(realtime_mode_);

        pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 1,
            &OctoMapBuilderNode::pointcloudCallback, this);

        octomap_pub_ = nh_global_.advertise<octomap_msgs::Octomap>(octomap_topic_, 1, true);
        occupied_voxels_pub_ = nh_global_.advertise<sensor_msgs::PointCloud2>(occupied_voxels_topic_, 1, true);
        bt_ready_pub_ = nh_global_.advertise<std_msgs::String>("/octomap_bt_ready", 1, true);

        load_pcd_srv_ = nh_.advertiseService("load_pcd",
            &OctoMapBuilderNode::loadPcdService, this);

        ROS_INFO("OctoMapBuilderNode initialized");
        ROS_INFO("  resolution: %f", resolution_);
        ROS_INFO("  min_points_per_voxel: %d", min_points_per_voxel_);
        ROS_INFO("  min_cluster_voxels: %d", min_cluster_voxels_);
        ROS_INFO("  input_pcd_path: %s", input_pcd_path_.c_str());
        ROS_INFO("  load_pcd_on_start: %s", load_pcd_on_start_ ? "true" : "false");
        ROS_INFO("  occupied_voxels_topic: %s", occupied_voxels_topic_.c_str());
        ROS_INFO("  pointcloud_topic: %s", pointcloud_topic_.c_str());
        ROS_INFO("  downsample_leaf_size: %f", downsample_leaf_size_);
        ROS_INFO("  realtime_mode: %s", realtime_mode_ ? "true" : "false");

        if (load_pcd_on_start_ && !input_pcd_path_.empty()) {
            loadPcdFromPath(input_pcd_path_);
        }
    }

    void spin()
    {
        ros::spin();
    }

private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (downsample_leaf_size_ > 0.0) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud);
            vg.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
            vg.filter(*downsampled);
            cloud = downsampled;
            ROS_DEBUG("PointCloud downsampled from %zu to %zu points", msg->width * msg->height, cloud->size());
        }

        if (realtime_mode_) {
            ROS_DEBUG("Realtime mode: updating octomap incrementally with %zu points", cloud->size());
            converter_.updateFromPointCloud(cloud);
        } else {
            ROS_DEBUG("Offline mode: rebuilding octomap with %zu points", cloud->size());
            converter_.convertFromPointCloud(cloud);
        }
        
        map_ready_ = true;

        update_counter_++;
        if (update_counter_ >= update_interval_) {
            update_counter_ = 0;
            publishOctomap();
            publishOccupiedVoxels();
            ROS_INFO("Octomap updated (interval: %d, realtime_mode: %s)", update_interval_, realtime_mode_ ? "true" : "false");
        }
    }

    bool loadPcdService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        if (input_pcd_path_.empty()) {
            res.success = false;
            res.message = "Input PCD path is not set";
            return true;
        }

        if (loadPcdFromPath(input_pcd_path_)) {
            res.success = true;
            res.message = "PCD file loaded successfully";
        } else {
            res.success = false;
            res.message = "Failed to load PCD file";
        }
        return true;
    }

    bool loadPcdFromPath(const std::string& path)
    {
        converter_.setInputPcdPath(path);
        if (converter_.convert()) {
            map_ready_ = true;
            publishOctomap();
            publishOccupiedVoxels();
            ROS_INFO("PCD file loaded and OctoMap published");
            return true;
        } else {
            ROS_ERROR("Failed to convert PCD to OctoMap");
            return false;
        }
    }

    void publishOctomap()
    {
        auto octree = converter_.getOctomap();
        if (!octree) {
            ROS_WARN("OctoMap is null, cannot publish");
            return;
        }

        if (!realtime_mode_) {
            std::string bt_path = "/home/robot/robot_slam/src/robot_navigation/octo_planner_3d/result_cleaned.bt";
            octree->writeBinary(bt_path);
            ROS_INFO("OctoMap saved to BT file: %s", bt_path.c_str());

            std_msgs::String ready_msg;
            ready_msg.data = bt_path;
            bt_ready_pub_.publish(ready_msg);
            ROS_INFO("Published octomap_bt_ready notification");
        }

        octomap_msgs::Octomap msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.binary = true;
        msg.resolution = octree->getResolution();

        std::stringstream ss;
        octree->writeBinary(ss);
        std::string data = ss.str();
        msg.data.resize(data.size());
        std::copy(data.begin(), data.end(), msg.data.begin());

        octomap_pub_.publish(msg);
        ROS_DEBUG("OctoMap published with %zu bytes", msg.data.size());
    }

    void publishOccupiedVoxels()
    {
        auto octree = converter_.getOctomap();
        if (!octree) {
            ROS_WARN("OctoMap is null, cannot publish occupied voxels");
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->header.frame_id = "map";
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());

        for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
            if (octree->isNodeOccupied(*it)) {
                pcl::PointXYZRGB point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();
                point.r = 100;
                point.g = 100;
                point.b = 100;
                cloud->points.push_back(point);
            }
        }

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        occupied_voxels_pub_.publish(msg);
        ROS_INFO("Published %zu occupied voxels as PointCloud2", cloud->points.size());
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_global_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher octomap_pub_;
    ros::Publisher occupied_voxels_pub_;
    ros::Publisher bt_ready_pub_;
    ros::ServiceServer load_pcd_srv_;

    octo_planner_3d::Pcd2OctomapConverter converter_;

    double resolution_;
    int min_points_per_voxel_;
    int min_cluster_voxels_;
    std::string input_pcd_path_;
    bool load_pcd_on_start_;
    std::string pointcloud_topic_;
    std::string octomap_topic_;
    std::string occupied_voxels_topic_;
    bool map_ready_;
    double downsample_leaf_size_;
    bool realtime_mode_;
    int update_counter_;
    int update_interval_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_builder_node");
    
    OctoMapBuilderNode node;
    node.spin();

    return 0;
}
