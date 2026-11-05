/*
 * @Descripttion: 
 * @FilePath: /src/robot_navigation/octo_planner_3d/src/global_planner_node.cpp
 * @Author: chaohui_chen1024
 * @Date: 2026-07-06 10:34:26
 * @LastEditors: chaohui_chen1024
 * @LastEditTime: 2026-07-06 20:18:17
 */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <octomap_msgs/Octomap.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <limits>

#include <global_planner.h>


class GlobalPlannerNode
{
public:
    GlobalPlannerNode()
        : nh_("~"), nh_global_(), planner_(), map_ready_(false), has_goal_(false),
          odom_ready_(false), plan_pending_(false),
          path_available_(false), validate_path_on_map_update_(false)
    {
        nh_.param<double>("robot_radius", robot_radius_, 0.20);
        nh_.param<int>("max_iterations", max_iterations_, 250000);
        nh_.param<int>("snap_search_radius_cells", snap_search_radius_cells_, 8);
        nh_.param<bool>("require_ground_support", require_ground_support_, true);
        nh_.param<bool>("strict_direct_ground_support", strict_direct_ground_support_, true);
        nh_.param<int>("ground_support_xy_radius_cells", ground_support_xy_radius_cells_, 1);
        nh_.param<int>("ground_support_depth_cells", ground_support_depth_cells_, 2);
        nh_.param<bool>("enable_preblocked_costmap", enable_preblocked_costmap_, true);
        nh_.param<int>("preblocked_costmap_radius_cells", preblocked_costmap_radius_cells_, 3);
        nh_.param<double>("preblocked_costmap_weight", preblocked_costmap_weight_, 1.5);
        nh_.param<bool>("lowest_traversable_only", lowest_traversable_only_, false);
        nh_.param<double>("z_cost_penalty_factor", z_cost_penalty_factor_, 5.0);
        nh_.param<bool>("require_ground_support_for_upward", require_ground_support_for_upward_, true);
        nh_.param<double>("robot_height", robot_height_, 0.55);
        nh_.param<double>("max_traversable_slope_deg", max_traversable_slope_deg_, 35.0);
        nh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap");
        nh_.param<std::string>("path_topic", path_topic_, "path");
        nh_.param<std::string>("path_marker_topic", path_marker_topic_, "path_marker");
        nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
        nh_.param<double>("map_update_interval", map_update_interval_, 1.0);
        nh_.param<double>("min_replan_interval", min_replan_interval_, 3.0);
        nh_.param<bool>("replan_on_map_update", replan_on_map_update_, false);
        nh_.param<std::string>("replan_topic", replan_topic_, "/navigation/replan_request");
        nh_.param<std::string>("goal_pose_topic", goal_pose_topic_, "/global_planner/goal_pose");
        nh_.param<bool>("validate_path_on_map_update", validate_path_on_map_update_, false);
        planner_.setParams(robot_radius_, max_iterations_, snap_search_radius_cells_,
                          require_ground_support_, strict_direct_ground_support_,
                          ground_support_xy_radius_cells_, ground_support_depth_cells_,
                          enable_preblocked_costmap_, preblocked_costmap_radius_cells_,
                          preblocked_costmap_weight_, lowest_traversable_only_,
                          z_cost_penalty_factor_, require_ground_support_for_upward_,
                          robot_height_, max_traversable_slope_deg_);

        bt_ready_sub_ = nh_global_.subscribe("/octomap_bt_ready", 1,
            &GlobalPlannerNode::btReadyCallback, this);

        octomap_sub_ = nh_global_.subscribe(octomap_topic_, 1,
            &GlobalPlannerNode::octomapCallback, this);

        point_goal_sub_ = nh_global_.subscribe("/clicked_point", 1,
            &GlobalPlannerNode::pointGoalCallback, this);

        // 直接给定目标点（起点自动取当前里程计位姿）。手柄/外部程序用它发布导航点，
        // 与 /clicked_point 一样：每次点击/发布都只是一个目标点，起点永远是机器人当前位置。
        goal_pose_sub_ = nh_global_.subscribe(goal_pose_topic_, 1,
            &GlobalPlannerNode::goalPoseCallback, this);

        odom_sub_ = nh_global_.subscribe(odom_topic_, 10,
            &GlobalPlannerNode::odomCallback, this);

        replan_sub_ = nh_global_.subscribe(replan_topic_, 1,
            &GlobalPlannerNode::replanCallback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(path_marker_topic_, 1, true);

        ROS_INFO("GlobalPlannerNode initialized");
        ROS_INFO("  robot_radius: %f", robot_radius_);
        ROS_INFO("  max_iterations: %d", max_iterations_);
        ROS_INFO("  require_ground_support: %s", require_ground_support_ ? "true" : "false");
        ROS_INFO("  z_cost_penalty_factor: %.1f", z_cost_penalty_factor_);
        ROS_INFO("  require_ground_support_for_upward: %s", require_ground_support_for_upward_ ? "true" : "false");
        ROS_INFO("  octomap_topic: %s", octomap_topic_.c_str());
        ROS_INFO("  Waiting for OctoMap on topic: %s", octomap_topic_.c_str());
        ROS_INFO("  odom_topic: %s", odom_topic_.c_str());
        ROS_INFO("  replan_on_map_update: %s", replan_on_map_update_ ? "true" : "false");
    }

    void spin()
    {
        ros::spin();
    }

private:
    void btReadyCallback(const std_msgs::String::ConstPtr& msg)
    {
        const std::string& bt_path = msg->data;
        ROS_INFO("Received octomap_bt_ready notification, loading from: %s", bt_path.c_str());

        octree_ = std::make_shared<octomap::OcTree>(bt_path);
        if (!octree_) {
            ROS_ERROR("Failed to load OctoMap from BT file: %s", bt_path.c_str());
            return;
        }

        int occupied_count = 0;
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (octree_->isNodeOccupied(*it)) {
                occupied_count++;
            }
        }

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

        ROS_INFO("OctoMap loaded from BT file: %s", bt_path.c_str());
        ROS_INFO("  resolution: %f", octree_->getResolution());
        ROS_INFO("  occupied nodes: %d", occupied_count);
        ROS_INFO("  bounds: [%f, %f, %f] to [%f, %f, %f]",
                 min_x, min_y, min_z, max_x, max_y, max_z);

        map_min_z_ = min_z;
        map_max_z_ = max_z;
        map_resolution_ = octree_->getResolution();

        planner_.setOctomap(octree_);
        map_ready_ = true;

        ROS_INFO("OctoMap setup complete, planner ready for path requests");

        if (plan_pending_ && has_goal_ && odom_ready_) {
            plan();
        }
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        const ros::Time now = ros::Time::now();
        if (!last_map_update_time_.isZero() &&
            (now - last_map_update_time_).toSec() < map_update_interval_) {
            return;
        }
        last_map_update_time_ = now;

        ROS_INFO("Received OctoMap message with %zu bytes", msg->data.size());

        std::stringstream ss;
        ss.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());

        octree_ = std::make_shared<octomap::OcTree>(0.1);
        if (!octree_ || !octree_->readBinary(ss)) {
            ROS_ERROR("Failed to deserialize OctoMap from message");
            octree_.reset();
            return;
        }

        int occupied_count = 0;
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (octree_->isNodeOccupied(*it)) {
                occupied_count++;
            }
        }

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

        ROS_INFO("OctoMap received from topic: %s", octomap_topic_.c_str());
        ROS_INFO("  resolution: %f", octree_->getResolution());
        ROS_INFO("  occupied nodes: %d", occupied_count);
        ROS_INFO("  bounds: [%f, %f, %f] to [%f, %f, %f]",
                 min_x, min_y, min_z, max_x, max_y, max_z);

        map_min_z_ = min_z;
        map_max_z_ = max_z;
        map_resolution_ = octree_->getResolution();

        planner_.setOctomap(octree_);
        map_ready_ = true;

        ROS_INFO("OctoMap updated from topic, planner ready");

        if (plan_pending_ && has_goal_ && odom_ready_) {
            plan();
        } else if (shouldReplanInvalidPath(now)) {
            plan();
        } else if (replan_on_map_update_ && path_available_ && odom_ready_ &&
                   (last_plan_time_.isZero() ||
                    (now - last_plan_time_).toSec() >= min_replan_interval_)) {
            plan();
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_point_.x = msg->pose.pose.position.x;
        current_point_.y = msg->pose.pose.position.y;
        current_point_.z = msg->pose.pose.position.z;
        odom_ready_ = true;
    }

    void replanCallback(const std_msgs::Empty::ConstPtr&)
    {
        const ros::Time now = ros::Time::now();
        if (!map_ready_ || !has_goal_ || !odom_ready_) {
            ROS_WARN_THROTTLE(1.0, "Replan requested before map, goal, or odometry is ready");
            return;
        }
        if (!last_plan_time_.isZero() &&
            (now - last_plan_time_).toSec() < min_replan_interval_) {
            ROS_WARN_THROTTLE(1.0, "Replan request throttled");
            return;
        }
        ROS_INFO("Replanning from current odometry after progress watchdog request");
        plan();
    }

    bool shouldReplanInvalidPath(const ros::Time& now)
    {
        if (!validate_path_on_map_update_ || !path_available_ || !odom_ready_ ||
            active_path_.empty())
            return false;
        if (!last_plan_time_.isZero() &&
            (now - last_plan_time_).toSec() < min_replan_interval_)
            return false;

        std::size_t nearest_index = 0;
        double best_distance_sq = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < active_path_.size(); ++i) {
            const double dx = active_path_[i].x - current_point_.x;
            const double dy = active_path_[i].y - current_point_.y;
            const double dz = active_path_[i].z - current_point_.z;
            const double distance_sq = dx * dx + dy * dy + dz * dz;
            if (distance_sq < best_distance_sq) {
                best_distance_sq = distance_sq;
                nearest_index = i;
            }
        }

        if (planner_.isPathValid(active_path_, nearest_index))
            return false;

        ROS_WARN("Active path became invalid after map update; replanning from current odometry");
        return true;
    }

    // RViz 里点一下就是目标点：起点不需要设置，永远是机器人当前位姿。
    void pointGoalCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        double x = msg->point.x;
        double y = msg->point.y;
        double z = msg->point.z;
        if (map_ready_) {
            z = snapToValidHeight(x, y, z);
        }

        goal_point_.x = x;
        goal_point_.y = y;
        goal_point_.z = z;
        has_goal_ = true;
        plan_pending_ = true;
        path_available_ = false;

        ROS_INFO("3D Goal pose set (from clicked_point): (%f, %f, %f)", x, y, z);

        publishGoalMarker();

        if (map_ready_) {
            plan();
        }
    }

    // 直接指定目标点：起点取机器人当前里程计位姿，立即规划。
    // 供手柄/外部脚本发送导航点使用（等价于“到那儿去”，不需要先给起点）。
    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!map_ready_) {
            ROS_WARN("Goal pose received before the map is ready; ignored");
            return;
        }
        if (!odom_ready_) {
            ROS_WARN("Goal pose received before odometry is ready; ignored");
            return;
        }

        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;
        z = snapToValidHeight(x, y, z);

        goal_point_.x = x;
        goal_point_.y = y;
        goal_point_.z = z;
        has_goal_ = true;

        publishGoalMarker();

        ROS_INFO("3D Goal pose set (from %s): (%f, %f, %f)",
                 goal_pose_topic_.c_str(), x, y, z);
        plan();
    }

    double snapToValidHeight(double x, double y, double z)
    {
        if (!octree_) {
            return z;
        }

        if (z < map_min_z_ || z > map_max_z_ || std::abs(z) < 0.01) {
            double first_occupied_z = map_max_z_;
            bool found_occupied = false;

            for (double search_z = map_min_z_; search_z <= map_max_z_; search_z += map_resolution_) {
                octomap::OcTreeNode* node = octree_->search(x, y, search_z);
                if (node && octree_->isNodeOccupied(node)) {
                    if (!found_occupied || search_z < first_occupied_z) {
                        first_occupied_z = search_z;
                        found_occupied = true;
                    }
                }
            }

            if (found_occupied) {
                z = first_occupied_z + map_resolution_;
                ROS_INFO("Auto-snapped Z from %f to %f (just above first occupied at ground level)", z - map_resolution_, z);
            } else {
                z = (map_min_z_ + map_max_z_) / 2.0;
                ROS_WARN("No occupied cells found, setting Z to center: %f", z);
            }
        }

        return z;
    }

    // 起点固定取机器人当前里程计位姿，不支持手工指定起点。
    void plan()
    {
        if (!map_ready_) {
            ROS_WARN("Map not ready, cannot plan");
            return;
        }
        if (!odom_ready_) {
            ROS_WARN("Cannot plan before odometry is ready");
            return;
        }

        // 起点标记同步成当前位姿，方便在 RViz 里核对这一次规划的起点
        start_point_ = current_point_;
        publishStartMarker();

        last_plan_time_ = ros::Time::now();
        plan_pending_ = false;
        planner_.makePlan(start_point_, goal_point_);

        std::vector<octo_planner_3d::PointPose> path;
        planner_.getPlannerResults(path);

        if (path.empty()) {
            ROS_WARN("No path found");
            path_available_ = false;
            active_path_.clear();
            nav_msgs::Path empty_path;
            empty_path.header.frame_id = "map";
            empty_path.header.stamp = ros::Time::now();
            path_pub_.publish(empty_path);
            return;
        }

        publishPath(path);
        publishPathMarker(path);

        plan_pending_ = false;
        path_available_ = true;
        active_path_ = path;

        ROS_INFO("Path published with %zu waypoints (start: current odometry)", path.size());
    }

    void publishPath(const std::vector<octo_planner_3d::PointPose>& path)
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        for (size_t i = 0; i < path.size(); ++i) {
            const auto& point = path[i];
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = point.z;
            const auto& direction_from = i + 1 < path.size() ? path[i] : path[i - (i > 0 ? 1 : 0)];
            const auto& direction_to = i + 1 < path.size() ? path[i + 1] : path[i];
            const double dx = direction_to.x - direction_from.x;
            const double dy = direction_to.y - direction_from.y;
            const double dz = direction_to.z - direction_from.z;
            const double horizontal = hypot(dx, dy);
            const double yaw = horizontal > 1e-6 ? atan2(dy, dx) : 0.0;
            const double pitch = horizontal > 1e-6 ? -atan2(dz, horizontal) : 0.0;
            pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, pitch, yaw);
            path_msg.poses.push_back(pose);
        }

        path_pub_.publish(path_msg);
    }

    void publishPathMarker(const std::vector<octo_planner_3d::PointPose>& path)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "path";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        for (const auto& point : path) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            marker.points.push_back(p);
        }

        marker_pub_.publish(marker);
    }

    void publishStartMarker()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "start_point";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = start_point_.x;
        marker.pose.position.y = start_point_.y;
        marker.pose.position.z = start_point_.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker_pub_.publish(marker);
    }

    void publishGoalMarker()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal_point";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goal_point_.x;
        marker.pose.position.y = goal_point_.y;
        marker.pose.position.z = goal_point_.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker_pub_.publish(marker);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_global_;
    ros::Subscriber bt_ready_sub_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber point_goal_sub_;
    ros::Subscriber goal_pose_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber replan_sub_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;

    octo_planner_3d::GlobalPlanner planner_;
    std::shared_ptr<octomap::OcTree> octree_;

    double robot_radius_;
    int max_iterations_;
    int snap_search_radius_cells_;
    bool require_ground_support_;
    bool strict_direct_ground_support_;
    int ground_support_xy_radius_cells_;
    int ground_support_depth_cells_;
    bool enable_preblocked_costmap_;
    int preblocked_costmap_radius_cells_;
    double preblocked_costmap_weight_;
    bool lowest_traversable_only_;
    double z_cost_penalty_factor_;
    bool require_ground_support_for_upward_;
    double robot_height_;
    double max_traversable_slope_deg_;
    std::string octomap_topic_;
    std::string path_topic_;
    std::string path_marker_topic_;
    std::string odom_topic_;
    std::string replan_topic_;
    std::string goal_pose_topic_;

    double map_update_interval_;
    double min_replan_interval_;
    bool replan_on_map_update_;

    double map_min_z_;
    double map_max_z_;
    double map_resolution_;

    bool map_ready_;
    bool has_goal_;
    bool odom_ready_;
    bool plan_pending_;
    bool path_available_;
    bool validate_path_on_map_update_;

    ros::Time last_map_update_time_;
    ros::Time last_plan_time_;

    octo_planner_3d::PointPose start_point_;
    octo_planner_3d::PointPose goal_point_;
    octo_planner_3d::PointPose current_point_;
    std::vector<octo_planner_3d::PointPose> active_path_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planner_node");

    GlobalPlannerNode node;
    node.spin();

    return 0;
}
