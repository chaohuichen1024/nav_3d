/*
 * @Descripttion: 3D Global Planner using OctoMap with A* algorithm
 * @FilePath: /src/robot_navigation/octo_planner_3d/include/global_planner.h
 * @Author: chaohui_chen1024 (optimized)
 * @Date: 2026-07-06 10:30:01
 * @LastEditors: chaohui_chen1024
 * @LastEditTime: 2026-07-08 17:15:26
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>

#include <octomap/OcTree.h>

namespace octo_planner_3d
{

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & k) const
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct QueueNode
{
  GridIndex idx;
  double f;
  double g;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    return a.f > b.f;
  }
};

struct PointPose
{
    double x;
    double y;
    double z;
};

class GlobalPlanner
{
public:
  GlobalPlanner();
  
  ~GlobalPlanner();

  void setOctomap(std::shared_ptr<octomap::OcTree> map);

  void makePlan(const PointPose start, const PointPose goal);

  void getPlannerResults(std::vector<PointPose>& plannerResults);

  bool isPathValid(const std::vector<PointPose>& path, std::size_t start_index = 0) const;
  
  void setParams(double robot_radius, int max_iterations, int snap_search_radius_cells,
                 bool require_ground_support, bool strict_direct_ground_support,
                 int ground_support_xy_radius_cells, int ground_support_depth_cells,
                 bool enable_preblocked_costmap, int preblocked_costmap_radius_cells,
                 double preblocked_costmap_weight, bool lowest_traversable_only,
                 double z_cost_penalty_factor, bool require_ground_support_for_upward,
                 double robot_height, double max_traversable_slope_deg);

private:

  void onGoalPose(const PointPose goal);  // 保留声明（可能被外部调用，先保留，但实现为空或移除）

  void tryPlan();

  GridIndex worldToGrid(double x, double y, double z) const;

  octomap::point3d gridToWorld(const GridIndex & idx) const;

  bool isInsideMetricBounds(const GridIndex & idx) const;

  bool hasGroundSupport(
    const GridIndex & idx,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool isOccupiedCell(const GridIndex & idx) const;

  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const;

  void rebuildPreblockedCells();

  void rebuildPreblockedCostmap();

  double getPreblockedCost(const GridIndex & idx) const;

  void rebuildDerivedLayers();

  bool isCellTraversable(
    const GridIndex & idx,
    double robot_radius,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool findNearestFreeCell(
    const GridIndex & seed,
    double robot_radius,
    int radius_cells,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells,
    GridIndex & out) const;

  std::vector<GridIndex> make26Directions() const;

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const;

  bool startPlan();

  // 新增：路径剪枝函数
  std::vector<GridIndex> prunePath(const std::vector<GridIndex> & path) const;

  double euclidean(const GridIndex & a, const GridIndex & b)
  {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    const double dz = static_cast<double>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }


private:
 
  std::string source_world_file_;

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

  bool map_ready_;
  bool has_start_;
  bool has_goal_;
  bool has_goal_pose_;
  bool planning_in_progress_;

  std::uint64_t plan_seq_;
  std::uint64_t last_success_seq_;
  std::uint64_t last_octomap_hash_;

  PointPose start_point_;
  PointPose goal_point_;
  PointPose goal_pose_;

  std::vector<PointPose> planner_results_;

  std::shared_ptr<octomap::OcTree> octree_;

  // 预计算的可通行栅格集合
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;

  // 新增：分辨率倒数，用于加速坐标转换
  double inv_resolution_;
};

}
