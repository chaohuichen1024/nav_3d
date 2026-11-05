/*
 * @Descripttion: 
 * @FilePath: /src/robot_navigation/octo_planner_3d/src/local_planner_node.cpp
 * @Author: chaohui_chen1024
 * @Date: 2026-07-09 11:47:11
 * @LastEditors: chaohui_chen1024
 * @LastEditTime: 2026-09-19 10:31:43
 */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <local_planner.h>

namespace octo_planner_3d
{

class LocalPlannerNode
{
public:
    LocalPlannerNode()
        : nh_("~"), nh_global_(), planner_(), path_ready_(false), odom_ready_(false), 
          octomap_ready_(false), control_frequency_(20.0), first_path_received_(false),
          require_fresh_obstacle_data_(true), odom_timeout_(0.5), obstacle_timeout_(2.0),
          local_octomap_update_interval_(1.0), use_octomap_for_local_collision_(false),
          progress_timeout_(5.0), progress_distance_(0.15),
          commanded_motion_threshold_(0.08), replan_cooldown_(5.0),
          progress_initialized_(false), replan_pending_(false), stop_held_(false),
          local_cloud_processing_frequency_(5.0),
          max_replan_attempts_(3), replan_attempts_(0), navigation_failed_(false)
    {
        loadParams();
        initROS();
        ROS_INFO("LocalPlannerNode initialized");
    }

    void spin()
    {
        ros::Rate rate(control_frequency_);
        while (ros::ok())
        {
            ros::spinOnce();
            controlLoop();
            rate.sleep();
        }
        publishStop();
    }

private:
    void loadParams()
    {
        nh_.param<double>("max_speed", params_.max_speed, 1.0);
        nh_.param<double>("min_speed", params_.min_speed, -0.5);
        nh_.param<double>("max_angular_speed", params_.max_angular_speed, 1.5);
        nh_.param<double>("min_angular_speed", params_.min_angular_speed, -1.5);
        nh_.param<double>("acceleration", params_.acceleration, 0.5);
        nh_.param<double>("angular_acceleration", params_.angular_acceleration, 1.0);
        nh_.param<double>("dt", params_.dt, 0.1);
        nh_.param<double>("predict_time", params_.predict_time, 2.0);
        nh_.param<double>("goal_tolerance", params_.goal_tolerance, 0.3);
        nh_.param<double>("goal_z_tolerance", params_.goal_z_tolerance, 0.5);
        nh_.param<double>("goal_xy_hold_time", params_.goal_xy_hold_time, 2.0);
        nh_.param<double>("lookahead_distance", params_.lookahead_distance, 1.0);
        nh_.param<double>("robot_radius", params_.robot_radius, 0.2);
        nh_.param<double>("weight_heading", params_.weight_heading, 0.3);
        nh_.param<double>("weight_distance", params_.weight_distance, 0.5);
        nh_.param<double>("weight_velocity", params_.weight_velocity, 0.2);
        nh_.param<double>("weight_path", params_.weight_path, 0.2);
        nh_.param<int>("num_speed_samples", params_.num_speed_samples, 10);
        nh_.param<int>("num_angular_samples", params_.num_angular_samples, 10);
        nh_.param<bool>("allow_no_obstacle_info", params_.allow_no_obstacle_info, true);
        nh_.param<double>("decel_start", params_.decel_start, 3.5);
        nh_.param<double>("decel_end", params_.decel_end, 1.5);
        nh_.param<double>("min_approach_speed", params_.min_approach_speed, 0.1);
        nh_.param<double>("min_tracking_speed", params_.min_tracking_speed, 0.08);
        nh_.param<double>("path_equality_tolerance", params_.path_equality_tolerance, 0.05);
        nh_.param<double>("path_resample_step", params_.path_resample_step, 0.25);
        nh_.param<double>("odom_velocity_filter_alpha", params_.odom_velocity_filter_alpha, 0.35);
        nh_.param<double>("max_traversable_slope_deg", params_.max_traversable_slope_deg, 35.0);
        nh_.param<double>("surface_normal_radius", params_.surface_normal_radius, 0.35);
        nh_.param<double>("terrain_fit_radius", params_.terrain_fit_radius, 2.0);
        nh_.param<int>("min_terrain_points", params_.min_terrain_points, 20);
        nh_.param<double>("terrain_fit_max_error", params_.terrain_fit_max_error, 0.15);
        nh_.param<double>("slope_enter_angle_deg", params_.slope_enter_angle_deg, 5.0);
        nh_.param<double>("slope_exit_angle_deg", params_.slope_exit_angle_deg, 3.0);
        nh_.param<double>("path_elevation_change_threshold", params_.path_elevation_change_threshold, 0.25);
        nh_.param<double>("min_climb_speed", params_.min_climb_speed, 0.12);
        nh_.param<double>("max_climb_speed", params_.max_climb_speed, 0.30);
        nh_.param<double>("max_climb_angular_speed", params_.max_climb_angular_speed, 0.50);

        nh_.param<double>("control_frequency", control_frequency_, 20.0);
        nh_.param<double>("downsample_leaf_size", downsample_leaf_size_, 0.1);
        nh_.param<bool>("require_fresh_obstacle_data", require_fresh_obstacle_data_, true);
        nh_.param<double>("odom_timeout", odom_timeout_, 0.5);
        nh_.param<double>("obstacle_timeout", obstacle_timeout_, 2.0);
        nh_.param<double>("local_octomap_update_interval", local_octomap_update_interval_, 1.0);
        nh_.param<bool>("use_octomap_for_local_collision", use_octomap_for_local_collision_, false);
        nh_.param<double>("progress_timeout", progress_timeout_, 5.0);
        nh_.param<double>("progress_distance", progress_distance_, 0.15);
        nh_.param<double>("commanded_motion_threshold", commanded_motion_threshold_, 0.08);
        nh_.param<double>("replan_cooldown", replan_cooldown_, 5.0);
        nh_.param<int>("max_replan_attempts", max_replan_attempts_, 3);
        nh_.param<double>("local_cloud_processing_frequency", local_cloud_processing_frequency_, 5.0);
        
        nh_.param<std::string>("path_topic", path_topic_, "/global_planner/path");
        nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
        nh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
        nh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap");
        nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
        nh_.param<std::string>("replan_topic", replan_topic_, "/navigation/replan_request");
        nh_.param<std::string>("stop_topic", stop_topic_, "/navigation/stop");
        nh_.param<std::string>("map_frame", map_frame_, "map");
        
        planner_.setParams(params_);
        
        ROS_INFO("Local Planner Parameters:");
        ROS_INFO("  max_speed: %f", params_.max_speed);
        ROS_INFO("  max_angular_speed: %f", params_.max_angular_speed);
        ROS_INFO("  goal_tolerance: %f", params_.goal_tolerance);
        ROS_INFO("  goal_z_tolerance: %f", params_.goal_z_tolerance);
        ROS_INFO("  lookahead_distance: %f", params_.lookahead_distance);
        ROS_INFO("  weight_heading: %f", params_.weight_heading);
        ROS_INFO("  weight_distance: %f", params_.weight_distance);
        ROS_INFO("  weight_velocity: %f", params_.weight_velocity);
        ROS_INFO("  weight_path: %f", params_.weight_path);
        ROS_INFO("  control_frequency: %f", control_frequency_);
        ROS_INFO("  allow_no_obstacle_info: %s", params_.allow_no_obstacle_info ? "true" : "false");
        ROS_INFO("  min_tracking_speed: %f", params_.min_tracking_speed);
        ROS_INFO("  require_fresh_obstacle_data: %s", require_fresh_obstacle_data_ ? "true" : "false");
        ROS_INFO("  max_traversable_slope_deg: %.1f", params_.max_traversable_slope_deg);
        ROS_INFO("  Subscribing to path topic: %s", path_topic_.c_str());
        ROS_INFO("  Subscribing to odom topic: %s", odom_topic_.c_str());
    }

    void initROS()
    {
        path_sub_ = nh_global_.subscribe(path_topic_, 1,
            &LocalPlannerNode::pathCallback, this);
        
        odom_sub_ = nh_global_.subscribe(odom_topic_, 100,
            &LocalPlannerNode::odomCallback, this);
        
        cloud_sub_ = nh_global_.subscribe(cloud_topic_, 10,
            &LocalPlannerNode::cloudCallback, this);
        
        if (use_octomap_for_local_collision_)
            octomap_sub_ = nh_global_.subscribe(octomap_topic_, 1,
                &LocalPlannerNode::octomapCallback, this);

        // 外部（手柄/遥控程序）请求取消导航
        stop_sub_ = nh_global_.subscribe(stop_topic_, 1,
            &LocalPlannerNode::stopCallback, this);
        
        cmd_vel_pub_ = nh_global_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
        
        local_path_pub_ = nh_global_.advertise<nav_msgs::Path>("local_path", 10);
        
        target_marker_pub_ = nh_global_.advertise<visualization_msgs::Marker>("local_target_marker", 10);
        
        trajectory_marker_pub_ = nh_global_.advertise<visualization_msgs::Marker>("local_trajectory_marker", 10);
        replan_pub_ = nh_global_.advertise<std_msgs::Empty>(replan_topic_, 1);
        
        ROS_INFO("[LocalPlannerNode] Subscribing to: path=%s, odom=%s, cloud=%s, octomap=%s",
                 path_topic_.c_str(), odom_topic_.c_str(), cloud_topic_.c_str(), octomap_topic_.c_str());
        ROS_INFO("[LocalPlannerNode] Publishing to: cmd_vel=%s, local_path=/local_path, markers=/local_target_marker, /local_trajectory_marker", cmd_vel_topic_.c_str());
    }

    // 取消当前导航：清掉正在跟踪的路径，之后控制循环处于空闲（不会再发速度指令），
    // 手柄/遥控就可以正常接管 /cmd_vel。收到新的全局路径即自动恢复导航。
    void stopCallback(const std_msgs::Empty::ConstPtr&)
    {
        planner_.reset();
        path_ready_ = false;
        replan_pending_ = false;
        navigation_failed_ = false;
        replan_attempts_ = 0;
        stop_held_ = false;
        publishStop();
        ROS_WARN("[LocalPlannerNode] Navigation cancelled on %s; waiting for a new goal",
                 stop_topic_.c_str());
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        // 本节点自己请求的重规划（进度看门狗触发）不能清零 replan_attempts_，
        // 否则“卡住 → 重规划 → 再卡住”会无限循环、永远到不了放弃条件；
        // 只有用户重新发布导航点（此时没有待处理的重规划）才算新的导航任务。
        const bool from_watchdog_replan = replan_pending_;
        replan_pending_ = false;

        if (msg->poses.empty())
        {
            planner_.reset();
            path_ready_ = false;
            if (!from_watchdog_replan)
            {
                replan_attempts_ = 0;
                navigation_failed_ = false;
            }
            publishStop();
            ROS_WARN("[LocalPlannerNode] Global planner reported no valid path; stopping");
            return;
        }

        if (!from_watchdog_replan)
        {
            replan_attempts_ = 0;
            navigation_failed_ = false;
        }

        planner_.setGlobalPath(msg);
        path_ready_ = true;
        stop_held_ = false;

        last_path_time_ = ros::Time::now();
        last_progress_time_ = last_path_time_;
        progress_x_ = current_x_;
        progress_y_ = current_y_;
        progress_z_ = current_z_;
        progress_initialized_ = odom_ready_;
        
        if (!first_path_received_)
        {
            first_path_received_ = true;
            ROS_INFO("[LocalPlannerNode] First global path received with %zu waypoints", msg->poses.size());
        }
        
        ROS_DEBUG("[LocalPlannerNode] Path received: %zu waypoints, frame: %s", 
                  msg->poses.size(), msg->header.frame_id.c_str());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        planner_.updateRobotState(msg);
        odom_ready_ = true;
        last_odom_time_ = ros::Time::now();
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        if (!progress_initialized_)
        {
            progress_x_ = current_x_;
            progress_y_ = current_y_;
            progress_z_ = current_z_;
            last_progress_time_ = last_odom_time_;
            progress_initialized_ = true;
        }
        const double pdx = current_x_ - progress_x_;
        const double pdy = current_y_ - progress_y_;
        const double pdz = current_z_ - progress_z_;
        if (sqrt(pdx * pdx + pdy * pdy + pdz * pdz) >= progress_distance_)
        {
            progress_x_ = current_x_;
            progress_y_ = current_y_;
            progress_z_ = current_z_;
            last_progress_time_ = last_odom_time_;
        }
        
        static ros::Time last_log_time;
        if ((ros::Time::now() - last_log_time).toSec() > 5.0)
        {
            ROS_DEBUG("[LocalPlannerNode] Odometry received: (%.2f, %.2f, %.2f), yaw: %.2f",
                      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                      atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                                  msg->pose.pose.orientation.x * msg->pose.pose.orientation.y), 
                            1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + 
                                         msg->pose.pose.orientation.z * msg->pose.pose.orientation.z)));
            last_log_time = ros::Time::now();
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::string cloud_frame = msg->header.frame_id;
        std::string expected_frame = map_frame_;
        while (!cloud_frame.empty() && cloud_frame.front() == '/') cloud_frame.erase(cloud_frame.begin());
        while (!expected_frame.empty() && expected_frame.front() == '/') expected_frame.erase(expected_frame.begin());
        if (cloud_frame != expected_frame)
        {
            ROS_ERROR_THROTTLE(2.0,
                "[LocalPlannerNode] Cloud frame '%s' does not match planning frame '%s'; dropping cloud",
                msg->header.frame_id.c_str(), map_frame_.c_str());
            return;
        }

        const ros::Time now = ros::Time::now();
        last_cloud_time_ = now;
        const double processing_period = local_cloud_processing_frequency_ > 0.0 ?
                                         1.0 / local_cloud_processing_frequency_ : 0.0;
        if (processing_period > 0.0 && !last_cloud_processing_time_.isZero() &&
            (now - last_cloud_processing_time_).toSec() < processing_period)
            return;
        last_cloud_processing_time_ = now;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (downsample_leaf_size_ > 0.0 && cloud->size() > 0)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud);
            vg.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
            vg.filter(*downsampled);
            planner_.updateLocalCloud(downsampled);
        }
        else
        {
            planner_.updateLocalCloud(cloud);
        }
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        const ros::Time now = ros::Time::now();
        if (!last_octomap_deserialize_time_.isZero() &&
            (now - last_octomap_deserialize_time_).toSec() < local_octomap_update_interval_)
            return;
        last_octomap_deserialize_time_ = now;

        std::stringstream ss;
        ss.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
        
        octree_ = std::make_shared<octomap::OcTree>(0.1);
        if (octree_ && octree_->readBinary(ss))
        {
            planner_.updateOctomap(octree_);
            octomap_ready_ = true;
            last_octomap_time_ = now;
            ROS_DEBUG("[LocalPlannerNode] OctoMap updated for local planner");
        }
        else
        {
            ROS_ERROR("[LocalPlannerNode] Failed to deserialize OctoMap");
        }
    }

    void controlLoop()
    {
        geometry_msgs::Twist cmd_vel;

        const ros::Time now = ros::Time::now();
        const bool odom_fresh = odom_ready_ && !last_odom_time_.isZero() &&
                                (now - last_odom_time_).toSec() <= odom_timeout_;
        const bool cloud_fresh = !last_cloud_time_.isZero() &&
                                 (now - last_cloud_time_).toSec() <= obstacle_timeout_;
        if (!odom_fresh)
        {
            publishStop();
            ROS_WARN_THROTTLE(1.0, "[LocalPlannerNode] Odometry is missing or stale; commanding stop");
            return;
        }
        if (require_fresh_obstacle_data_ && !cloud_fresh)
        {
            publishStop();
            ROS_WARN_THROTTLE(1.0, "[LocalPlannerNode] Obstacle data is missing or stale; commanding stop");
            return;
        }
        if (navigation_failed_)
        {
            // 反复重规划仍无法推进，说明当前目标点不可达。停下来等新的导航点，
            // 避免“卡住 8s → 重规划 → 再卡住”的无休止循环。
            publishStopHeld();
            ROS_ERROR_THROTTLE(2.0,
                "[LocalPlannerNode] Goal unreachable (no progress after %d replans); "
                "stopped, waiting for a new global path", replan_attempts_ - 1);
            return;
        }
        if (planner_.isGoalReached())
        {
            // 到点即结束本次导航：停止规划、保持零速度，等待下一次发布导航点。
            publishStopHeld();
            ROS_INFO_THROTTLE(2.0,
                "[LocalPlannerNode] Goal reached; navigation finished, waiting for a new goal");
            return;
        }
        if (replan_pending_)
        {
            // A watchdog replan replaces the current route.  Continuing to
            // track the stale route while A* is running can drive the robot
            // away from the stair entrance, especially on a large 3-D map.
            publishStopHeld();
            ROS_INFO_THROTTLE(1.0, "[LocalPlannerNode] Waiting for replanned global path; commanding stop");
            return;
        }
        
        bool success = planner_.computeVelocity(cmd_vel);
        
        if (success)
        {
            const bool motion_commanded = fabs(cmd_vel.linear.x) >= commanded_motion_threshold_;
            const bool stalled = motion_commanded && progress_initialized_ &&
                                 !last_progress_time_.isZero() &&
                                 (now - last_progress_time_).toSec() >= progress_timeout_;
            const bool replan_allowed = last_replan_request_time_.isZero() ||
                                        (now - last_replan_request_time_).toSec() >= replan_cooldown_;
            if (stalled && replan_allowed && !planner_.isGoalReached())
            {
                ++replan_attempts_;
                last_replan_request_time_ = now;
                last_progress_time_ = now;
                publishStop();

                if (max_replan_attempts_ > 0 && replan_attempts_ > max_replan_attempts_)
                {
                    // 连续多次重规划都无法让机器人前进，说明当前目标点在现有地图上
                    // 不可达（例如终点被吸附到机器人上不去的平台）。放弃本次导航并
                    // 停下，等待用户发布新的初始点/目标点，不再无休止地重规划。
                    navigation_failed_ = true;
                    replan_pending_ = false;
                    ROS_ERROR("[LocalPlannerNode] Giving up after %d replan attempts; goal is unreachable",
                              replan_attempts_ - 1);
                    return;
                }

                replan_pending_ = true;
                replan_pub_.publish(std_msgs::Empty());
                ROS_WARN("[LocalPlannerNode] No %.2fm progress for %.1fs; requested replan %d/%d and stopped",
                         progress_distance_, progress_timeout_,
                         replan_attempts_, max_replan_attempts_);
                return;
            }

            // 正在跟踪路径：正常发布速度，并允许之后重新发送零速
            stop_held_ = false;
            cmd_vel_pub_.publish(cmd_vel);
            
            publishLocalPath();
            publishTargetMarker();
            publishTrajectoryMarker();
            
            static ros::Time last_info_time;
            if ((ros::Time::now() - last_info_time).toSec() > 1.0)
            {
                ROS_INFO("[LocalPlannerNode] Published cmd_vel: vx=%.2f, vy=%.2f, wz=%.2f",
                         cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
                const char* mode = planner_.getTerrainMode() == LocalPlanner::TerrainMode::UPHILL ? "UPHILL" :
                                   planner_.getTerrainMode() == LocalPlanner::TerrainMode::DOWNHILL ? "DOWNHILL" : "FLAT";
                ROS_INFO("[LocalPlannerNode] Terrain mode=%s, slope=%.1f deg",
                         mode, planner_.getTerrainSlope() * 180.0 / M_PI);
                last_info_time = ros::Time::now();
            }
            
            if (planner_.isGoalReached())
            {
                ROS_INFO_THROTTLE(2.0, "[LocalPlannerNode] Goal reached, stopping robot");
            }
        }
        else
        {
            // 本次算不出有效速度（没有路径 / 等待新目标等）：只发一次零速把之前
            // 可能残留的指令覆盖掉，之后保持安静。若继续以控制频率反复发零速，
            // 会与手柄遥控抢 /cmd_vel，手动开车会被反复打断。
            publishStopHeld();
            if (!path_ready_)
            {
                ROS_INFO_THROTTLE(2.0, "[LocalPlannerNode] Waiting for global path on topic: %s", path_topic_.c_str());
            }
            else if (!odom_ready_)
            {
                ROS_INFO_THROTTLE(2.0, "[LocalPlannerNode] Waiting for odometry on topic: %s", odom_topic_.c_str());
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "[LocalPlannerNode] Failed to compute velocity");
            }
        }
    }

    void publishStop()
    {
        geometry_msgs::Twist stop;
        cmd_vel_pub_.publish(stop);
        stop_held_ = true;
    }

    // 空闲/终态下只发一次零速：把可能残留的指令覆盖掉即可，之后保持安静。
    // 持续反复发零速会和手柄遥控抢 /cmd_vel。
    void publishStopHeld()
    {
        if (!stop_held_)
            publishStop();
    }

    void publishLocalPath()
    {
        nav_msgs::Path local_path = planner_.getLocalPath();
        if (!local_path.poses.empty())
        {
            local_path_pub_.publish(local_path);
        }
    }

    void publishTargetMarker()
    {
        geometry_msgs::PointStamped target;
        if (!planner_.getCurrentTarget(target))
            return;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "local_target";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = target.point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.5);
        
        target_marker_pub_.publish(marker);
    }

    void publishTrajectoryMarker()
    {
        nav_msgs::Path local_path = planner_.getLocalPath();
        if (local_path.poses.empty())
            return;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.08;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.2);
        
        for (const auto& pose : local_path.poses)
        {
            marker.points.push_back(pose.pose.position);
        }
        
        trajectory_marker_pub_.publish(marker);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_global_;
    
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber stop_sub_;
    
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher target_marker_pub_;
    ros::Publisher trajectory_marker_pub_;
    ros::Publisher replan_pub_;
    
    LocalPlanner planner_;
    DWAParams params_;
    std::shared_ptr<octomap::OcTree> octree_;
    
    std::string path_topic_;
    std::string odom_topic_;
    std::string cloud_topic_;
    std::string octomap_topic_;
    std::string cmd_vel_topic_;
    std::string replan_topic_;
    std::string stop_topic_;
    std::string map_frame_;
    
    double control_frequency_;
    double downsample_leaf_size_;
    bool require_fresh_obstacle_data_;
    double odom_timeout_;
    double obstacle_timeout_;
    double local_octomap_update_interval_;
    bool use_octomap_for_local_collision_;
    double progress_timeout_;
    double progress_distance_;
    double commanded_motion_threshold_;
    double replan_cooldown_;
    double local_cloud_processing_frequency_;
    int max_replan_attempts_;
    int replan_attempts_;
    bool navigation_failed_;
    bool progress_initialized_;
    bool replan_pending_;
    // 空闲/终态下是否已经发过零速（避免反复发零速与手柄遥控抢 /cmd_vel）
    bool stop_held_;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_z_ = 0.0;
    double progress_x_ = 0.0;
    double progress_y_ = 0.0;
    double progress_z_ = 0.0;

    ros::Time last_path_time_;
    ros::Time last_odom_time_;
    ros::Time last_cloud_time_;
    ros::Time last_octomap_time_;
    ros::Time last_octomap_deserialize_time_;
    ros::Time last_progress_time_;
    ros::Time last_replan_request_time_;
    ros::Time last_cloud_processing_time_;
    
    bool path_ready_;
    bool odom_ready_;
    bool octomap_ready_;
    bool first_path_received_;
};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_planner_node");
    
    ROS_INFO("[LocalPlannerNode] Starting local planner node...");
    
    octo_planner_3d::LocalPlannerNode node;
    
    ROS_INFO("[LocalPlannerNode] Local planner node initialized successfully, starting spin loop");
    
    node.spin();
    
    return 0;
}
