#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace octo_planner_3d
{

struct RobotState
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double vx;
    double vy;
    double vz;
    double wx;
    double wy;
    double wz;
};

struct DWAParams
{
    double max_speed;
    double min_speed;
    double max_angular_speed;
    double min_angular_speed;
    double acceleration;
    double angular_acceleration;
    double dt;
    double predict_time;
    double goal_tolerance;
    double goal_z_tolerance;
    // 水平方向已经进入终点容差、但高差超限（终点在另一层或落在悬空薄板上）时，
    // 机器人若已静止并持续这么久，就结束本次导航，不再原地反复蹭。
    double goal_xy_hold_time;
    double lookahead_distance;
    double robot_radius;
    double weight_heading;
    double weight_distance;
    double weight_velocity;
    double weight_path;
    int num_speed_samples;
    int num_angular_samples;
    bool allow_no_obstacle_info;
    double decel_start;
    double decel_end;
    double min_approach_speed;
    double min_tracking_speed;
    double path_equality_tolerance;
    double path_resample_step;
    double odom_velocity_filter_alpha;
    double max_traversable_slope_deg;
    double surface_normal_radius;
    double terrain_fit_radius;
    int min_terrain_points;
    double terrain_fit_max_error;
    double slope_enter_angle_deg;
    double slope_exit_angle_deg;
    double path_elevation_change_threshold;
    double min_climb_speed;
    double max_climb_speed;
    double max_climb_angular_speed;
};

struct VelocitySample
{
    double vx;
    double vy;
    double wz;
    double score;
    std::vector<geometry_msgs::Point> trajectory;
};

class LocalPlanner
{
public:
    enum class TerrainMode { FLAT, UPHILL, DOWNHILL };

    LocalPlanner();
    ~LocalPlanner();

    void setParams(const DWAParams& params);
    void setGlobalPath(const nav_msgs::Path::ConstPtr& path);
    void updateRobotState(const nav_msgs::Odometry::ConstPtr& odom);
    void updateLocalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void updateOctomap(std::shared_ptr<octomap::OcTree> octree);

    bool computeVelocity(geometry_msgs::Twist& cmd_vel);
    bool getCurrentTarget(geometry_msgs::PointStamped& target);
    bool isGoalReached() const;
    void reset();
    nav_msgs::Path getLocalPath();
    TerrainMode getTerrainMode() const;
    double getTerrainSlope() const;

private:
    bool generateDynamicWindow(std::vector<VelocitySample>& window);
    std::vector<geometry_msgs::Point> predictTrajectory(double vx, double vy, double wz, double time);
    double evaluateHeading(const std::vector<geometry_msgs::Point>& trajectory, double angular_speed);
    double evaluateDistance(const std::vector<geometry_msgs::Point>& trajectory, double speed);
    double evaluateVelocity(double vx, double vy);
    double evaluatePathAlignment(const std::vector<geometry_msgs::Point>& trajectory);
    bool findBestVelocity(VelocitySample& best);
    void logDwaDiagnostics(size_t total_samples, int valid_samples,
                           const VelocitySample& best, bool found_valid,
                           double heading_score, double distance_score,
                           double velocity_score, double path_score);
    int findNearestPathPoint();
    int findNearestPathPointLocked() const;
    bool updateTargetPoint();
    double getDistanceToObstacle(double x, double y, double z) const;
    bool isPointSafe(double x, double y, double z, double speed) const;
    bool hasObstacleInfo() const;
    bool pathsEquivalentLocked(const nav_msgs::Path& path) const;
    static nav_msgs::Path resamplePath(const nav_msgs::Path& path, double step);
    void classifyCloudAndFitTerrain(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void updateTerrainMode();
    double terrainHeight(double x, double y) const;

private:
    DWAParams params_;
    RobotState robot_state_;
    nav_msgs::Path global_path_;
    // 全局规划器剪枝后的路径相邻点间距可达数米，直接用于“最近点/前视点/横向偏差”
    // 会严重失真（前视点会一下跳到几米外的拐点）。因此把收到的路径按固定步长
    // 重采样后放在这里，跟踪一律使用这一份；global_path_ 只保留原始路径做等价比较。
    nav_msgs::Path tracked_path_;
    int current_path_index_;
    // 接管新路径时测得的 “路径 z − odom z” 系统性偏置。路径 z 是地面之上的
    // 空闲体素中心，/odom 是 base_footprint，两者相差一个近似常量。
    double path_z_offset_;
    bool has_path_z_offset_;
    geometry_msgs::PointStamped target_point_;
    bool path_ready_;
    bool odom_ready_;
    bool goal_reached_;
    // 水平已到终点容差但高差不合格时，开始计时的时刻；离开容差即清零。
    ros::Time goal_xy_hold_start_;
    
    std::shared_ptr<octomap::OcTree> octree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr obstacle_kdtree_;
    
    std::mutex path_mutex_;
    std::mutex odom_mutex_;
    mutable std::mutex cloud_mutex_;
    
    std::vector<geometry_msgs::Point> best_trajectory_;

    double last_cmd_vx_;
    double last_cmd_wz_;

    bool has_last_odom_pose_;
    ros::Time last_odom_stamp_;
    double last_odom_x_;
    double last_odom_y_;
    double last_odom_z_;
    double last_odom_yaw_;

    bool terrain_model_ready_;
    double terrain_a_;
    double terrain_b_;
    double terrain_c_;
    double terrain_slope_;
    TerrainMode terrain_mode_;
};

}
