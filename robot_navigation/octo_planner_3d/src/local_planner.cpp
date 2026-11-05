/*
 * @Descripttion: 
 * @FilePath: /src/robot_navigation/octo_planner_3d/src/local_planner.cpp
 * @Author: chaohui_chen1024
 * @Date: 2026-07-09 11:45:54
 * @LastEditors: chaohui_chen1024
 * @LastEditTime: 2026-07-09 22:10:56
 */
#include <local_planner.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

namespace octo_planner_3d
{

LocalPlanner::LocalPlanner()
    : current_path_index_(0), path_z_offset_(0.0), has_path_z_offset_(false),
      path_ready_(false), odom_ready_(false), goal_reached_(false),
      has_last_odom_pose_(false), last_odom_x_(0.0), last_odom_y_(0.0),
      last_odom_z_(0.0), last_odom_yaw_(0.0), terrain_model_ready_(false),
      terrain_a_(0.0), terrain_b_(0.0), terrain_c_(0.0), terrain_slope_(0.0),
      terrain_mode_(TerrainMode::FLAT)
{
    local_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    obstacle_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    surface_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    obstacle_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    
    params_.max_speed = 1.0;
    params_.min_speed = -0.5;
    params_.max_angular_speed = 1.5;
    params_.min_angular_speed = -1.5;
    params_.acceleration = 0.5;
    params_.angular_acceleration = 1.0;
    params_.dt = 0.1;
    params_.predict_time = 2.0;
    params_.goal_tolerance = 0.3;
    params_.goal_z_tolerance = 0.5;
    params_.goal_xy_hold_time = 2.0;
    params_.lookahead_distance = 1.0;
    params_.robot_radius = 0.2;
    params_.weight_heading = 0.3;
    params_.weight_distance = 0.5;
    params_.weight_velocity = 0.2;
    params_.weight_path = 0.2;
    params_.num_speed_samples = 10;
    params_.num_angular_samples = 10;
    params_.allow_no_obstacle_info = true;
    params_.decel_start = 3.5;
    params_.decel_end = 1.5;
    params_.min_approach_speed = 0.1;
    params_.min_tracking_speed = 0.08;
    params_.path_equality_tolerance = 0.05;
    params_.path_resample_step = 0.25;
    params_.odom_velocity_filter_alpha = 0.35;
    params_.max_traversable_slope_deg = 35.0;
    params_.surface_normal_radius = 0.35;
    params_.terrain_fit_radius = 2.0;
    params_.min_terrain_points = 20;
    params_.terrain_fit_max_error = 0.15;
    params_.slope_enter_angle_deg = 5.0;
    params_.slope_exit_angle_deg = 3.0;
    params_.path_elevation_change_threshold = 0.25;
    params_.min_climb_speed = 0.12;
    params_.max_climb_speed = 0.30;
    params_.max_climb_angular_speed = 0.50;

    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.z = 0.0;
    robot_state_.roll = 0.0;
    robot_state_.pitch = 0.0;
    robot_state_.yaw = 0.0;
    robot_state_.vx = 0.0;
    robot_state_.vy = 0.0;
    robot_state_.vz = 0.0;
    robot_state_.wz = 0.0;

    last_cmd_vx_ = 0.0;
    last_cmd_wz_ = 0.0;
}

LocalPlanner::~LocalPlanner()
{
}

void LocalPlanner::setParams(const DWAParams& params)
{
    params_ = params;
}

void LocalPlanner::setGlobalPath(const nav_msgs::Path::ConstPtr& path)
{
    std::lock_guard<std::mutex> lock(path_mutex_);

    if (pathsEquivalentLocked(*path))
    {
        // Only refresh the header. Keeping the old path and progress prevents
        // latched/repeated global path messages from restarting tracking.
        global_path_.header = path->header;
        ROS_DEBUG_THROTTLE(2.0, "[LocalPlanner] Equivalent global path ignored; preserving index %d",
                           current_path_index_);
        return;
    }

    global_path_ = *path;
    tracked_path_ = resamplePath(*path, params_.path_resample_step);
    path_ready_ = !tracked_path_.poses.empty();
    goal_reached_ = false;
    has_path_z_offset_ = false;
    goal_xy_hold_start_ = ros::Time();

    // A replanned path normally starts at the current robot position. Find the
    // closest 3D waypoint instead of blindly returning to index zero.
    current_path_index_ = 0;
    if (odom_ready_ && path_ready_)
    {
        double best_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < tracked_path_.poses.size(); ++i)
        {
            const auto& p = tracked_path_.poses[i].pose.position;
            const double dx = p.x - robot_state_.x;
            const double dy = p.y - robot_state_.y;
            const double dz = p.z - robot_state_.z;
            const double dist_sq = dx * dx + dy * dy + dz * dz;
            if (dist_sq < best_dist_sq)
            {
                best_dist_sq = dist_sq;
                current_path_index_ = static_cast<int>(i);
            }
        }
    }

    // 用机器人附近这条路径点，量出 “路径 z − odom z” 的系统性偏置。
    // 同一个楼层内该偏置近似恒定，因此可以用它把路径终点的 z 换算到 odom 系，
    // 从而既能判断“真的到了终点”，又能在机器人还停在另一层时拒绝误判。
    if (odom_ready_ && path_ready_ &&
        current_path_index_ >= 0 &&
        current_path_index_ < static_cast<int>(tracked_path_.poses.size()))
    {
        path_z_offset_ = tracked_path_.poses[current_path_index_].pose.position.z - robot_state_.z;
        has_path_z_offset_ = true;
    }

    ROS_INFO("[LocalPlanner] New global path received with %zu waypoints (%zu after resampling); tracking from index %d",
             global_path_.poses.size(), tracked_path_.poses.size(), current_path_index_);
}

// 把稀疏的全局路径按 step 等间隔线性插值加密。
// 剪枝后的路径相邻点间距可达数米，若直接用它做前视与横向偏差评估，
// 前视点会直接跳到段末端（前视失效），偏差评估也会在段内失去分辨率。
nav_msgs::Path LocalPlanner::resamplePath(const nav_msgs::Path& path, double step)
{
    nav_msgs::Path out;
    out.header = path.header;
    if (path.poses.empty())
        return out;

    if (step <= 1e-3)
    {
        out = path;
        return out;
    }

    out.poses.push_back(path.poses.front());
    for (size_t i = 0; i + 1 < path.poses.size(); ++i)
    {
        const auto& a = path.poses[i].pose.position;
        const auto& b = path.poses[i + 1].pose.position;
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dz = b.z - a.z;
        const double seg = sqrt(dx * dx + dy * dy + dz * dz);
        const int n = std::max(1, static_cast<int>(ceil(seg / step)));
        for (int k = 1; k <= n; ++k)
        {
            const double t = static_cast<double>(k) / static_cast<double>(n);
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.orientation.w = 1.0;
            pose.pose.position.x = a.x + t * dx;
            pose.pose.position.y = a.y + t * dy;
            pose.pose.position.z = a.z + t * dz;
            out.poses.push_back(pose);
        }
    }
    return out;
}

bool LocalPlanner::pathsEquivalentLocked(const nav_msgs::Path& path) const
{
    if (global_path_.poses.size() != path.poses.size() || path.poses.empty())
        return false;

    const double tol_sq = params_.path_equality_tolerance * params_.path_equality_tolerance;
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        const auto& a = global_path_.poses[i].pose.position;
        const auto& b = path.poses[i].pose.position;
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        if (dx * dx + dy * dy + dz * dz > tol_sq)
            return false;
    }
    return true;
}

void LocalPlanner::updateRobotState(const nav_msgs::Odometry::ConstPtr& odom)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    robot_state_.x = odom->pose.pose.position.x;
    robot_state_.y = odom->pose.pose.position.y;
    robot_state_.z = odom->pose.pose.position.z;
    
    double qx = odom->pose.pose.orientation.x;
    double qy = odom->pose.pose.orientation.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    robot_state_.roll = atan2(sinr_cosp, cosr_cosp);
    const double sinp = 2.0 * (qw * qy - qz * qx);
    robot_state_.pitch = fabs(sinp) >= 1.0 ? copysign(M_PI / 2.0, sinp) : asin(sinp);
    robot_state_.yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    const ros::Time stamp = odom->header.stamp.isZero() ? ros::Time::now() : odom->header.stamp;
    if (has_last_odom_pose_)
    {
        const double sample_dt = (stamp - last_odom_stamp_).toSec();
        if (sample_dt > 0.001 && sample_dt < 1.0)
        {
            const double world_vx = (robot_state_.x - last_odom_x_) / sample_dt;
            const double world_vy = (robot_state_.y - last_odom_y_) / sample_dt;
            const double measured_vx = cos(robot_state_.yaw) * world_vx +
                                       sin(robot_state_.yaw) * world_vy;
            double delta_yaw = robot_state_.yaw - last_odom_yaw_;
            while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
            while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
            const double measured_wz = delta_yaw / sample_dt;
            const double alpha = std::max(0.0, std::min(1.0, params_.odom_velocity_filter_alpha));

            robot_state_.vx = alpha * measured_vx + (1.0 - alpha) * robot_state_.vx;
            robot_state_.vy = alpha * (-sin(robot_state_.yaw) * world_vx +
                                        cos(robot_state_.yaw) * world_vy) +
                              (1.0 - alpha) * robot_state_.vy;
            robot_state_.vz = alpha * ((robot_state_.z - last_odom_z_) / sample_dt) +
                              (1.0 - alpha) * robot_state_.vz;
            robot_state_.wz = alpha * measured_wz + (1.0 - alpha) * robot_state_.wz;
        }
    }

    last_odom_stamp_ = stamp;
    last_odom_x_ = robot_state_.x;
    last_odom_y_ = robot_state_.y;
    last_odom_z_ = robot_state_.z;
    last_odom_yaw_ = robot_state_.yaw;
    has_last_odom_pose_ = true;
    
    odom_ready_ = true;
}

void LocalPlanner::updateLocalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    local_cloud_ = cloud;
    if (kdtree_ && local_cloud_ && !local_cloud_->empty())
    {
        kdtree_->setInputCloud(local_cloud_);
    }
    classifyCloudAndFitTerrain(local_cloud_);
}

void LocalPlanner::classifyCloudAndFitTerrain(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    obstacle_cloud_->clear();
    surface_cloud_->clear();
    terrain_model_ready_ = false;
    terrain_slope_ = 0.0;

    if (!cloud || cloud->empty())
        return;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setSearchMethod(search_tree);
    normal_estimator.setRadiusSearch(std::max(0.1, params_.surface_normal_radius));
    normal_estimator.compute(*normals);

    const double min_ground_normal_z =
        cos(params_.max_traversable_slope_deg * M_PI / 180.0);
    obstacle_cloud_->reserve(cloud->size());
    surface_cloud_->reserve(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        const auto& point = cloud->points[i];
        const auto& normal = normals->points[i];
        const bool valid_normal = std::isfinite(normal.normal_x) &&
                                  std::isfinite(normal.normal_y) &&
                                  std::isfinite(normal.normal_z);
        const bool traversable_surface = valid_normal &&
                                         fabs(normal.normal_z) >= min_ground_normal_z;
        if (traversable_surface)
            surface_cloud_->push_back(point);
        else
            obstacle_cloud_->push_back(point);
    }

    if (!obstacle_cloud_->empty())
        obstacle_kdtree_->setInputCloud(obstacle_cloud_);

    Eigen::Matrix3d normal_matrix = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    int fit_count = 0;
    const double fit_radius_sq = params_.terrain_fit_radius * params_.terrain_fit_radius;
    const double max_height_delta = std::max(
        0.5, params_.terrain_fit_radius * tan(params_.max_traversable_slope_deg * M_PI / 180.0));

    for (const auto& point : surface_cloud_->points)
    {
        const double dx = point.x - robot_state_.x;
        const double dy = point.y - robot_state_.y;
        if (dx * dx + dy * dy > fit_radius_sq ||
            fabs(point.z - robot_state_.z) > max_height_delta)
            continue;
        const Eigen::Vector3d row(dx, dy, 1.0);
        normal_matrix += row * row.transpose();
        rhs += row * (point.z - robot_state_.z);
        ++fit_count;
    }

    if (fit_count < params_.min_terrain_points ||
        fabs(normal_matrix.determinant()) < 1e-9)
        return;

    const Eigen::Vector3d solution = normal_matrix.ldlt().solve(rhs);
    double squared_error = 0.0;
    for (const auto& point : surface_cloud_->points)
    {
        const double dx = point.x - robot_state_.x;
        const double dy = point.y - robot_state_.y;
        if (dx * dx + dy * dy > fit_radius_sq ||
            fabs(point.z - robot_state_.z) > max_height_delta)
            continue;
        const double predicted = solution.x() * dx + solution.y() * dy + solution.z();
        const double error = (point.z - robot_state_.z) - predicted;
        squared_error += error * error;
    }
    const double rms_error = sqrt(squared_error / static_cast<double>(fit_count));
    if (!std::isfinite(rms_error) || rms_error > params_.terrain_fit_max_error)
        return;

    terrain_a_ = solution.x();
    terrain_b_ = solution.y();
    terrain_c_ = robot_state_.z + solution.z() -
                 terrain_a_ * robot_state_.x - terrain_b_ * robot_state_.y;
    terrain_slope_ = atan(sqrt(terrain_a_ * terrain_a_ + terrain_b_ * terrain_b_));
    terrain_model_ready_ = true;

    ROS_DEBUG_THROTTLE(1.0,
        "[LocalPlanner] Cloud classified: %zu surface, %zu obstacle; terrain slope %.1f deg, rms %.3f",
        surface_cloud_->size(), obstacle_cloud_->size(),
        terrain_slope_ * 180.0 / M_PI, rms_error);
}

double LocalPlanner::terrainHeight(double x, double y) const
{
    if (!terrain_model_ready_)
        return robot_state_.z;
    return terrain_a_ * x + terrain_b_ * y + terrain_c_;
}

void LocalPlanner::updateTerrainMode()
{
    const double enter_angle = params_.slope_enter_angle_deg * M_PI / 180.0;
    const double exit_angle = params_.slope_exit_angle_deg * M_PI / 180.0;
    const double observed_slope = std::max(fabs(robot_state_.pitch), terrain_slope_);

    // Compare two points on the global path, rather than comparing a path
    // point directly with odometry Z.  The planner path represents a free
    // cell above the floor while /odom is base_footprint; their constant
    // vertical offset otherwise makes a level upper floor look uphill.
    double path_height_delta = 0.0;
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (current_path_index_ >= 0 &&
            current_path_index_ < static_cast<int>(tracked_path_.poses.size()))
        {
            path_height_delta = target_point_.point.z -
                                tracked_path_.poses[current_path_index_].pose.position.z;
        }
    }

    const double dx = target_point_.point.x - robot_state_.x;
    const double dy = target_point_.point.y - robot_state_.y;
    const double plane_height_delta = terrain_model_ready_ ? terrain_a_ * dx + terrain_b_ * dy : 0.0;
    const double path_height_threshold = std::max(0.0, params_.path_elevation_change_threshold);

    // 0.20 m is the OctoMap resolution.  Requiring the configured 0.25 m
    // path elevation change ignores a single snapped voxel on a flat floor,
    // while the 1.2 m look-ahead still enters UPHILL before a smooth stair.
    if (path_height_delta > path_height_threshold)
    {
        terrain_mode_ = TerrainMode::UPHILL;
        return;
    }
    else if (path_height_delta < -path_height_threshold)
    {
        terrain_mode_ = TerrainMode::DOWNHILL;
        return;
    }

    // Keep the current non-flat mode while the chassis or fitted local plane
    // still confirms an incline.  This avoids mode flicker between adjacent
    // staircase waypoints whose height happens to be almost equal.
    if (terrain_mode_ != TerrainMode::FLAT && observed_slope >= exit_angle)
        return;

    // When only the local terrain model sees a new incline, require the
    // normal enter threshold before selecting a direction from its height.
    if (observed_slope >= enter_angle)
    {
        if (plane_height_delta > 0.03)
        {
            terrain_mode_ = TerrainMode::UPHILL;
            return;
        }
        if (plane_height_delta < -0.03)
        {
            terrain_mode_ = TerrainMode::DOWNHILL;
            return;
        }
    }

    terrain_mode_ = TerrainMode::FLAT;
}

LocalPlanner::TerrainMode LocalPlanner::getTerrainMode() const
{
    return terrain_mode_;
}

double LocalPlanner::getTerrainSlope() const
{
    return terrain_slope_;
}

void LocalPlanner::updateOctomap(std::shared_ptr<octomap::OcTree> octree)
{
    octree_ = octree;
}

bool LocalPlanner::computeVelocity(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    if (!path_ready_)
    {
        ROS_DEBUG_THROTTLE(1.0, "[LocalPlanner] Path not ready");
        return false;
    }
    
    if (!odom_ready_)
    {
        ROS_DEBUG_THROTTLE(1.0, "[LocalPlanner] Odometry not ready");
        return false;
    }
    
    if (goal_reached_)
    {
        ROS_INFO_ONCE("[LocalPlanner] Goal already reached");
        return true;
    }
    
    if (!updateTargetPoint())
    {
        ROS_WARN("[LocalPlanner] Failed to update target point");
        return false;
    }

    if (goal_reached_)
        return true;

    updateTerrainMode();
    
    VelocitySample best;
    if (!findBestVelocity(best))
    {
        ROS_WARN_THROTTLE(1.0, "[LocalPlanner] No valid velocity found in DWA window");
        
        if (hasObstacleInfo())
        {
            ROS_WARN_THROTTLE(2.0, "[LocalPlanner] Obstacles blocking all trajectories, stopping");
        }
        
        if (params_.allow_no_obstacle_info && !hasObstacleInfo())
        {
            double dx = target_point_.point.x - robot_state_.x;
            double dy = target_point_.point.y - robot_state_.y;
            double dist = sqrt(dx * dx + dy * dy);
            
            if (dist > params_.goal_tolerance)
            {
                double target_yaw = atan2(dy, dx);
                double angle_diff = target_yaw - robot_state_.yaw;
                while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
                
                double turn_speed = std::max(params_.min_angular_speed, std::min(params_.max_angular_speed, angle_diff * 2.0));
                double forward_speed = (fabs(angle_diff) < 0.5) ? params_.max_speed * 0.5 : 0.0;
                
                cmd_vel.linear.x = forward_speed;
                cmd_vel.angular.z = turn_speed;
                
                best_trajectory_ = predictTrajectory(forward_speed, 0.0, turn_speed, params_.predict_time);
                
                last_cmd_vx_ = forward_speed;
                last_cmd_wz_ = turn_speed;

                ROS_INFO("[LocalPlanner] No obstacle info, using simple tracking: vx=%.2f, wz=%.2f",
                         forward_speed, turn_speed);
                return true;
            }
        }
        
        best_trajectory_.clear();
        geometry_msgs::Point pt;
        pt.x = robot_state_.x;
        pt.y = robot_state_.y;
        pt.z = robot_state_.z;
        best_trajectory_.push_back(pt);

        last_cmd_vx_ = 0.0;
        last_cmd_wz_ = 0.0;
        return true;
    }
    
    cmd_vel.linear.x = best.vx;
    cmd_vel.linear.y = best.vy;
    cmd_vel.angular.z = best.wz;
    best_trajectory_ = best.trajectory;

    last_cmd_vx_ = best.vx;
    last_cmd_wz_ = best.wz;

    ROS_DEBUG("[LocalPlanner] Computed velocity: vx=%.2f, vy=%.2f, wz=%.2f (score=%.4f)",
              best.vx, best.vy, best.wz, best.score);

    return true;
}

bool LocalPlanner::getCurrentTarget(geometry_msgs::PointStamped& target)
{
    if (!path_ready_)
        return false;
    
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (current_path_index_ >= static_cast<int>(tracked_path_.poses.size()))
        return false;

    target = target_point_;
    return true;
}

bool LocalPlanner::isGoalReached() const
{
    return goal_reached_;
}

void LocalPlanner::reset()
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    current_path_index_ = 0;
    path_ready_ = false;
    goal_reached_ = false;
    goal_xy_hold_start_ = ros::Time();
    best_trajectory_.clear();
}

nav_msgs::Path LocalPlanner::getLocalPath()
{
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    
    for (const auto& pt : best_trajectory_)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position = pt;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    
    return path;
}

bool LocalPlanner::generateDynamicWindow(std::vector<VelocitySample>& window)
{
    window.clear();

    // 计算到终点的距离，距离 < 2m 时线性降速到 0.1 m/s
    double dist_to_goal = std::numeric_limits<double>::max();
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!tracked_path_.poses.empty())
        {
            const auto& goal = tracked_path_.poses.back().pose.position;
            const double dx = goal.x - robot_state_.x;
            const double dy = goal.y - robot_state_.y;
            const double dz = goal.z - robot_state_.z;
            dist_to_goal = sqrt(dx * dx + dy * dy + dz * dz);
        }
    }

    // 距终点 3.5m~1.5m 线性降速: 0.5 → 0.4 → 0.3 → 0.2 → 0.1 m/s
    const double decel_start = params_.decel_start;
    const double decel_end   = params_.decel_end;
    const double min_approach_speed = params_.min_approach_speed;
    double effective_max_speed = params_.max_speed;
    if (dist_to_goal < decel_start)
    {
        if (dist_to_goal <= decel_end)
            effective_max_speed = min_approach_speed;
        else
            effective_max_speed = min_approach_speed +
                (params_.max_speed - min_approach_speed) * (dist_to_goal - decel_end) / (decel_start - decel_end);
    }

    if (terrain_mode_ == TerrainMode::UPHILL)
        effective_max_speed = std::min(effective_max_speed, params_.max_climb_speed);

    // 角速度按同比例缩放，避免靠近终点时原地自转
    double speed_ratio = params_.max_speed > 1e-6 ?
                         effective_max_speed / params_.max_speed : 0.0;
    double effective_max_angular = params_.max_angular_speed * speed_ratio;
    double effective_min_angular = params_.min_angular_speed * speed_ratio;
    if (terrain_mode_ != TerrainMode::FLAT)
    {
        effective_max_angular = std::min(effective_max_angular, params_.max_climb_angular_speed);
        effective_min_angular = std::max(effective_min_angular, -params_.max_climb_angular_speed);
    }

    // Acceleration constraints must be centred on measured motion. Using the
    // previous command hides wheel slip and lets the internal state diverge.
    double v_min = std::max(params_.min_speed, robot_state_.vx - params_.acceleration * params_.dt);
    double v_max = std::min(effective_max_speed, robot_state_.vx + params_.acceleration * params_.dt);
    double w_min = std::max(effective_min_angular, robot_state_.wz - params_.angular_acceleration * params_.dt);
    double w_max = std::min(effective_max_angular, robot_state_.wz + params_.angular_acceleration * params_.dt);

    // When the measured speed is temporarily above a newly reduced limit,
    // keep the strongest acceleration-limited deceleration sample instead of
    // returning an empty window.
    if (v_min > v_max)
        v_max = v_min;
    if (w_min > w_max)
        w_max = w_min;

    if (!std::isfinite(v_min) || !std::isfinite(v_max) ||
        !std::isfinite(w_min) || !std::isfinite(w_max))
    {
        ROS_WARN("[LocalPlanner] Dynamic window contains non-finite bounds: v=[%.2f,%.2f], w=[%.2f,%.2f]",
                  v_min, v_max, w_min, w_max);
        return false;
    }

    const int speed_samples = std::max(2, params_.num_speed_samples);
    const int angular_samples = std::max(2, params_.num_angular_samples);
    double v_step = (v_max - v_min) / (speed_samples - 1);
    double w_step = (w_max - w_min) / (angular_samples - 1);
    
    for (int i = 0; i < speed_samples; ++i)
    {
        double vx = v_min + i * v_step;
        const double reachable_min_climb = std::min(params_.min_climb_speed, v_max);
        const double reachable_min_tracking = std::min(params_.min_tracking_speed, v_max);
        if (terrain_mode_ == TerrainMode::UPHILL && vx < 0.0)
            continue;
        if (terrain_mode_ == TerrainMode::UPHILL &&
            dist_to_goal > params_.goal_tolerance && vx > 0.0 &&
            vx + 1e-6 < reachable_min_climb)
            continue;
        if (dist_to_goal > params_.decel_end && vx > 0.0 &&
            vx + 1e-6 < reachable_min_tracking)
            continue;

        for (int j = 0; j < angular_samples; ++j)
        {
            double wz = w_min + j * w_step;
            VelocitySample sample;
            sample.vx = vx;
            sample.vy = 0.0;
            sample.wz = wz;
            sample.score = 0.0;
            window.push_back(sample);
        }
    }

    // Always provide an exact stop candidate. Uniform sampling otherwise
    // commonly misses zero and produces persistent 0.01/0.03 m/s crawling.
    VelocitySample stop;
    stop.vx = 0.0;
    stop.vy = 0.0;
    stop.wz = 0.0;
    stop.score = 0.0;
    window.push_back(stop);
    
    ROS_DEBUG("[LocalPlanner] Generated %zu velocity samples", window.size());
    return !window.empty();
}

std::vector<geometry_msgs::Point> LocalPlanner::predictTrajectory(double vx, double vy, double wz, double time)
{
    std::vector<geometry_msgs::Point> trajectory;
    
    double x = robot_state_.x;
    double y = robot_state_.y;
    double yaw = robot_state_.yaw;
    
    int steps = static_cast<int>(time / params_.dt);
    
    for (int i = 0; i <= steps; ++i)
    {
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = terrainHeight(x, y);
        trajectory.push_back(pt);
        
        yaw += wz * params_.dt;
        x += vx * cos(yaw) * params_.dt;
        y += vx * sin(yaw) * params_.dt;
    }
    
    return trajectory;
}

double LocalPlanner::evaluateHeading(const std::vector<geometry_msgs::Point>& trajectory,
                                     double angular_speed)
{
    if (trajectory.empty())
        return 0.0;
    
    const auto& end_pt = trajectory.back();
    
    double dx = target_point_.point.x - end_pt.x;
    double dy = target_point_.point.y - end_pt.y;
    double target_angle = atan2(dy, dx);
    
    // Position-only trajectories cannot reveal the future heading of an
    // in-place rotation (or the chassis heading while reversing).
    double final_yaw = robot_state_.yaw + angular_speed * params_.predict_time;
    
    double angle_diff = target_angle - final_yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
    
    return 1.0 - fabs(angle_diff) / M_PI;
}

double LocalPlanner::evaluateDistance(const std::vector<geometry_msgs::Point>& trajectory, double speed)
{
    if (trajectory.empty())
        return 0.0;

    if (!hasObstacleInfo())
        return 1.0;

    double min_dist = std::numeric_limits<double>::max();

    for (const auto& pt : trajectory)
    {
        double dist = getDistanceToObstacle(pt.x, pt.y, pt.z);
        if (dist < min_dist)
            min_dist = dist;
    }

    // 安全距离随速度动态缩放
    double safety_margin = params_.robot_radius + speed * 0.5;
    double safe_dist = safety_margin + 0.6 + speed * 0.5;

    if (min_dist < safety_margin)
        return 0.0;
    else if (min_dist < safe_dist)
        return (min_dist - safety_margin) / (safe_dist - safety_margin);
    else
        return 1.0;
}

double LocalPlanner::evaluateVelocity(double vx, double vy)
{
    (void)vy;
    if (vx <= 0.0 || params_.max_speed <= 0.0)
        return 0.0;
    return std::min(1.0, vx / params_.max_speed);
}

double LocalPlanner::evaluatePathAlignment(const std::vector<geometry_msgs::Point>& trajectory)
{
    if (trajectory.empty())
        return 1.0;

    // 用轨迹中点评估，代表这条轨迹的整体位置偏离路径多远
    const auto& pt = trajectory[trajectory.size() / 2];

    double min_dist = std::numeric_limits<double>::max();
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (tracked_path_.poses.empty())
            return 1.0;

        // 偏差只在“当前位置及其前方”的路径段上评估，避免把轨迹拉向身后的起点吸附点
        int search_start = std::max(0, current_path_index_);
        int search_end = std::min(static_cast<int>(tracked_path_.poses.size()),
                                  current_path_index_ + 50);

        for (int i = search_start; i < search_end; ++i)
        {
            const auto& pose = tracked_path_.poses[i];
            // 只比水平偏差。预测轨迹的 z 来自局部拟合平面（拟合失败时直接取
            // 机器人当前高度），它永远表示不了前方的台阶/陡坡；而路径 z 是台阶
            // 上方的空闲体素。若把 dz 计入，上坡时所有前进样本都会被判成“偏离
            // 路径”，DWA 于是稳定选中 vx=0 的停车样本，机器人就卡在坡底不动。
            // 竖直方向由 terrain 模式、避障和终点 z 校验分别负责。
            const double dx = pose.pose.position.x - pt.x;
            const double dy = pose.pose.position.y - pt.y;
            const double dist = sqrt(dx * dx + dy * dy);
            if (dist < min_dist)
                min_dist = dist;
        }
    }

    // 偏离超过 lookahead_distance 得 0 分，在路径上得 1 分
    double max_deviation = params_.lookahead_distance;
    if (min_dist >= max_deviation)
        return 0.0;
    return 1.0 - min_dist / max_deviation;
}

bool LocalPlanner::findBestVelocity(VelocitySample& best)
{
    std::vector<VelocitySample> window;
    if (!generateDynamicWindow(window))
        return false;
    
    double best_score = -std::numeric_limits<double>::max();
    bool found_valid = false;

    // 诊断用：记录最优样本的四个分项得分与有效样本数，便于判断“有路径却不走”的原因
    int valid_samples = 0;
    double best_heading_score = 0.0;
    double best_distance_score = 0.0;
    double best_velocity_score = 0.0;
    double best_path_score = 0.0;
    
    for (auto& sample : window)
    {
        sample.trajectory = predictTrajectory(sample.vx, sample.vy, sample.wz, params_.predict_time);

        double sample_speed = sqrt(sample.vx * sample.vx + sample.vy * sample.vy);

        bool valid = true;
        if (hasObstacleInfo())
        {
            for (const auto& pt : sample.trajectory)
            {
                if (!isPointSafe(pt.x, pt.y, pt.z, sample_speed))
                {
                    valid = false;
                    break;
                }
            }
        }

        if (!valid)
            continue;

        ++valid_samples;
        double heading_score = evaluateHeading(sample.trajectory, sample.wz);
        double distance_score = evaluateDistance(sample.trajectory, sample_speed);
        double velocity_score = evaluateVelocity(sample.vx, sample.vy);
        double path_score = evaluatePathAlignment(sample.trajectory);

        sample.score = params_.weight_heading * heading_score +
                       params_.weight_distance * distance_score +
                       params_.weight_velocity * velocity_score +
                       params_.weight_path * path_score;
        
        if (sample.score > best_score)
        {
            best_score = sample.score;
            best = sample;
            found_valid = true;
            best_heading_score = heading_score;
            best_distance_score = distance_score;
            best_velocity_score = velocity_score;
            best_path_score = path_score;
        }
    }

    logDwaDiagnostics(window.size(), valid_samples, best, found_valid,
                      best_heading_score, best_distance_score,
                      best_velocity_score, best_path_score);
    
    if (!found_valid)
    {
        ROS_WARN("[LocalPlanner] No valid velocity found after evaluating all samples");
    }
    
    return found_valid;
}

// 1 Hz 诊断输出：解释“全局路径已就绪但机器人不前进”的具体原因
void LocalPlanner::logDwaDiagnostics(size_t total_samples, int valid_samples,
                                     const VelocitySample& best, bool found_valid,
                                     double heading_score, double distance_score,
                                     double velocity_score, double path_score)
{
    static ros::Time last_debug_time;
    const ros::Time now = ros::Time::now();
    if (!last_debug_time.isZero() && (now - last_debug_time).toSec() < 1.0)
        return;
    last_debug_time = now;

    // 机器人正前方 0.5 m（贴合局部地形高度）处的障碍距离
    const double probe_x = robot_state_.x + 0.5 * cos(robot_state_.yaw);
    const double probe_y = robot_state_.y + 0.5 * sin(robot_state_.yaw);
    const double front_obstacle = getDistanceToObstacle(probe_x, probe_y, terrainHeight(probe_x, probe_y));

    // 最优样本轨迹上的最近障碍距离
    double traj_obstacle = std::numeric_limits<double>::max();
    if (found_valid && hasObstacleInfo())
    {
        for (const auto& pt : best.trajectory)
            traj_obstacle = std::min(traj_obstacle, getDistanceToObstacle(pt.x, pt.y, pt.z));
    }

    const double dx = target_point_.point.x - robot_state_.x;
    const double dy = target_point_.point.y - robot_state_.y;
    const double dist_target = sqrt(dx * dx + dy * dy);
    double bearing_err = atan2(dy, dx) - robot_state_.yaw;
    while (bearing_err > M_PI) bearing_err -= 2.0 * M_PI;
    while (bearing_err < -M_PI) bearing_err += 2.0 * M_PI;

    ROS_INFO("[DWADebug] v_meas=%.2f w_meas=%.2f samples=%zu valid=%d | "
             "best(vx=%.2f wz=%.2f score=%.3f h=%.2f d=%.2f v=%.2f p=%.2f) | "
             "target idx=%d dist=%.2f bearing_err=%.0fdeg | obstacle front=%.2f traj=%.2f | "
             "surface=%zu obstacle=%zu terrain_ready=%d slope=%.1fdeg",
             robot_state_.vx, robot_state_.wz, total_samples, valid_samples,
             found_valid ? best.vx : 0.0, found_valid ? best.wz : 0.0,
             found_valid ? best.score : 0.0,
             heading_score, distance_score, velocity_score, path_score,
             current_path_index_, dist_target, bearing_err * 180.0 / M_PI,
             front_obstacle, traj_obstacle,
             surface_cloud_ ? surface_cloud_->size() : 0,
             obstacle_cloud_ ? obstacle_cloud_->size() : 0,
             terrain_model_ready_ ? 1 : 0, terrain_slope_ * 180.0 / M_PI);
}

int LocalPlanner::findNearestPathPointLocked() const
{
    if (tracked_path_.poses.empty())
        return -1;
    
    double min_dist = std::numeric_limits<double>::max();
    int nearest_idx = current_path_index_;

    // 只从当前索引往前找：路径起点是 A* 对起点做吸附后的格子，可能落在机器人身后，
    // 若允许向前回搜，最近点会长期锁在这个身后点上，把“路径贴合”项变成倒车奖励。
    int search_start = std::max(0, current_path_index_);
    int search_end = std::min(static_cast<int>(tracked_path_.poses.size()), current_path_index_ + 50);
    
    for (int i = search_start; i < search_end; ++i)
    {
        const auto& pose = tracked_path_.poses[i];
        const double dx = pose.pose.position.x - robot_state_.x;
        const double dy = pose.pose.position.y - robot_state_.y;
        const double dz = pose.pose.position.z - robot_state_.z;
        const double dist = sqrt(dx * dx + dy * dy + dz * dz);
        
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    return nearest_idx;
}

int LocalPlanner::findNearestPathPoint()
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    return findNearestPathPointLocked();
}

bool LocalPlanner::updateTargetPoint()
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    
    if (tracked_path_.poses.empty())
        return false;
    
    int nearest_idx = findNearestPathPointLocked();
    if (nearest_idx < 0)
        return false;
    
    current_path_index_ = nearest_idx;

    // 路径先于里程计到达时，上面的偏置还没能测到；等机器人已经落在路径中段
    // （最近点不是终点）时补测一次。终点附近不能补测，否则偏置会被自己抵消。
    if (!has_path_z_offset_ &&
        nearest_idx < static_cast<int>(tracked_path_.poses.size()) - 1)
    {
        path_z_offset_ = tracked_path_.poses[nearest_idx].pose.position.z - robot_state_.z;
        has_path_z_offset_ = true;
    }

    // 到点判定必须每个周期都做，不能只挂在“最近点已是最后一个 waypoint”上：
    // 最近点由三维距离选出，路径 z 与 odom 之间存在偏置，一旦最近点长期不是
    // 终点，判定就永远不会执行（日志里 "Goal reached" 0 次就是这个原因）。
    {
        const auto& goal = tracked_path_.poses.back().pose.position;
        const double dx = goal.x - robot_state_.x;
        const double dy = goal.y - robot_state_.y;
        const double dist_goal_xy = sqrt(dx * dx + dy * dy);
        // goal.z 是地面之上的空闲体素中心，先按偏置换算成 odom 系里“到达终点时
        // 机器人应有的高度”，剩下的 dz_error 才是真实的相对高差，用于跨层保护：
        // 机器人停在低一层平台上时 dz_error 会明显变大，从而拒绝误判为到点。
        const double dz_error = robot_state_.z - (goal.z - path_z_offset_);

        // 诊断（1 Hz，仅当已经接近终点时输出）：说明“为什么还没判定到点”。
        // 到点不停的全部信息就是这一行里的 dist_xy / dz_error / 两个容差。
        static ros::Time last_goal_check_log;
        const ros::Time now_check = ros::Time::now();
        if (dist_goal_xy < 2.0 &&
            (last_goal_check_log.isZero() || (now_check - last_goal_check_log).toSec() >= 1.0))
        {
            last_goal_check_log = now_check;
            ROS_INFO("[GoalCheck] robot=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f) waypoints=%zu idx=%d "
                     "dist_xy=%.2f (tol %.2f) dz_error=%.2f (tol %.2f) offset=%.2f",
                     robot_state_.x, robot_state_.y, robot_state_.z,
                     goal.x, goal.y, goal.z, tracked_path_.poses.size(), current_path_index_,
                     dist_goal_xy, params_.goal_tolerance,
                     dz_error, params_.goal_z_tolerance, path_z_offset_);
        }

        if (dist_goal_xy < params_.goal_tolerance &&
            fabs(dz_error) < params_.goal_z_tolerance)
        {
            goal_reached_ = true;
            ROS_INFO("[LocalPlanner] Goal reached! Horizontal: %.2f, dz_error: %.2f (path z offset %.2f)",
                     dist_goal_xy, dz_error, path_z_offset_);
            return true;
        }

        // 水平已经站在终点容差圈内，但高差不合格：终点比机器人所在面高/低一大截
        // （终点在另一层，或落在只有 0.2 m 厚的悬空薄板顶上）。此时机器人再怎么
        // 调整也到不了，若继续跟踪就会原地蹭到进度看门狗超时。只要机器人已经基本
        // 停住并持续 goal_xy_hold_time，就结束本次导航，停车等待新的导航点。
        if (dist_goal_xy < params_.goal_tolerance)
        {
            const ros::Time now_hold = ros::Time::now();
            if (goal_xy_hold_start_.isZero())
            {
                goal_xy_hold_start_ = now_hold;
            }
            else if (fabs(robot_state_.vx) < params_.min_tracking_speed &&
                     (now_hold - goal_xy_hold_start_).toSec() >= params_.goal_xy_hold_time)
            {
                goal_reached_ = true;
                ROS_WARN("[LocalPlanner] Stopped at goal height mismatch: dist_xy=%.2f, "
                         "dz_error=%.2f (tolerance %.2f). The goal is likely on another "
                         "level or on a non-walkable slab; finishing this navigation.",
                         dist_goal_xy, dz_error, params_.goal_z_tolerance);
                return true;
            }
        }
        else
        {
            goal_xy_hold_start_ = ros::Time();
        }
    }

    int target_idx = current_path_index_;
    double accumulated_dist = 0.0;
    
    for (int i = current_path_index_; i < static_cast<int>(tracked_path_.poses.size()) - 1; ++i)
    {
        const auto& curr = tracked_path_.poses[i];
        const auto& next = tracked_path_.poses[i + 1];
        const double dx = next.pose.position.x - curr.pose.position.x;
        const double dy = next.pose.position.y - curr.pose.position.y;
        const double dz = next.pose.position.z - curr.pose.position.z;
        accumulated_dist += sqrt(dx * dx + dy * dy + dz * dz);

        // 前视点每走一段就跟着前进。若剩余路径总长不足 lookahead_distance，
        // 循环结束时 target_idx 必须停在最后一个 waypoint（终点）上；否则前视点
        // 会退化成机器人脚下那个点，机器人只会绕着它转、永远到不了终点。
        target_idx = i + 1;
        if (accumulated_dist >= params_.lookahead_distance)
            break;
    }
    
    if (target_idx >= static_cast<int>(tracked_path_.poses.size()))
        target_idx = tracked_path_.poses.size() - 1;
    
    target_point_.header = tracked_path_.header;
    target_point_.point = tracked_path_.poses[target_idx].pose.position;
    
    ROS_DEBUG("[LocalPlanner] Target point updated: (%.2f, %.2f, %.2f) at index %d", 
              target_point_.point.x, target_point_.point.y, target_point_.point.z, target_idx);
    
    return true;
}

double LocalPlanner::getDistanceToObstacle(double x, double y, double z) const
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (obstacle_kdtree_ && obstacle_cloud_ && !obstacle_cloud_->empty())
    {
        pcl::PointXYZ search_pt;
        search_pt.x = x;
        search_pt.y = y;
        search_pt.z = z;

        std::vector<int> indices;
        std::vector<float> distances;
        if (obstacle_kdtree_->nearestKSearch(search_pt, 1, indices, distances) > 0)
            return sqrt(distances.front());
    }

    return std::numeric_limits<double>::max();
}

bool LocalPlanner::isPointSafe(double x, double y, double z, double speed) const
{
    if (!hasObstacleInfo())
        return true;

    double dist = getDistanceToObstacle(x, y, z);
    // 安全距离随速度动态缩放: 速度越快，要求离障碍物越远
    double safety_margin = params_.robot_radius + speed * 0.5;
    return dist >= safety_margin;
}

bool LocalPlanner::hasObstacleInfo() const
{
    return local_cloud_ != nullptr && !local_cloud_->empty();
}

}
