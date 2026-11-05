/*
 * @Descripttion: 
 * @FilePath: /src/robot_navigation/octo_planner_3d/src/global_planner.cpp
 * @Author: chaohui_chen1024 (optimized)
 * @Date: 2026-07-06 10:31:50
 * @LastEditors: chaohui_chen1024
 * @LastEditTime: 2026-07-08 17:15:47
 */
#include <global_planner.h>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>  // for ROS_WARN

namespace octo_planner_3d
{

    GlobalPlanner::GlobalPlanner():
        robot_radius_(0.20),
        max_iterations_(250000),
        snap_search_radius_cells_(8),
        require_ground_support_(true),
        strict_direct_ground_support_(true),
        ground_support_xy_radius_cells_(1),
        ground_support_depth_cells_(2),
        enable_preblocked_costmap_(true),
        preblocked_costmap_radius_cells_(3),
        preblocked_costmap_weight_(1.5),
        lowest_traversable_only_(false),
        z_cost_penalty_factor_(5.0),
        require_ground_support_for_upward_(true),
        robot_height_(0.55),
        max_traversable_slope_deg_(35.0),
        map_ready_(false),
        has_start_(false),
        has_goal_(false),
        planning_in_progress_(false),
        plan_seq_(0),
        last_success_seq_(0),
        inv_resolution_(1.0)  // 默认值，后续更新
    {
        printf("GlobalPlanner Constructor!!! \n");
    }

    GlobalPlanner::~GlobalPlanner()
    {
        printf("GlobalPlanner Destructor!!! \n");
    }

    void GlobalPlanner::setParams(double robot_radius, int max_iterations, int snap_search_radius_cells,
                                  bool require_ground_support, bool strict_direct_ground_support,
                                  int ground_support_xy_radius_cells, int ground_support_depth_cells,
                                  bool enable_preblocked_costmap, int preblocked_costmap_radius_cells,
                                  double preblocked_costmap_weight, bool lowest_traversable_only,
                                  double z_cost_penalty_factor, bool require_ground_support_for_upward,
                                  double robot_height, double max_traversable_slope_deg)
    {
        robot_radius_ = robot_radius;
        max_iterations_ = max_iterations;
        snap_search_radius_cells_ = snap_search_radius_cells;
        require_ground_support_ = require_ground_support;
        strict_direct_ground_support_ = strict_direct_ground_support;
        ground_support_xy_radius_cells_ = ground_support_xy_radius_cells;
        ground_support_depth_cells_ = ground_support_depth_cells;
        enable_preblocked_costmap_ = enable_preblocked_costmap;
        preblocked_costmap_radius_cells_ = preblocked_costmap_radius_cells;
        preblocked_costmap_weight_ = preblocked_costmap_weight;
        lowest_traversable_only_ = lowest_traversable_only;
        z_cost_penalty_factor_ = z_cost_penalty_factor;
        require_ground_support_for_upward_ = require_ground_support_for_upward;
        robot_height_ = robot_height;
        max_traversable_slope_deg_ = max_traversable_slope_deg;
    }

    void GlobalPlanner::setOctomap(std::shared_ptr<octomap::OcTree> map)
    {
        if (!map)
        {
            printf("Octomap is Null!!! return.\n");
            return;
        }

        if (octree_ == map)
        {
            printf("Octomap is No Update!!! return.\n");
            return;
        }

        octree_ = map;
        map_ready_ = true;
        inv_resolution_ = 1.0 / octree_->getResolution();

        // 计算地图总栅格数，决定是否构建预计算集合
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        double r = octree_->getResolution();
        
        int grid_x = static_cast<int>((max_x - min_x) / r) + 1;
        int grid_y = static_cast<int>((max_y - min_y) / r) + 1;
        int grid_z = static_cast<int>((max_z - min_z) / r) + 1;
        long long total_cells = static_cast<long long>(grid_x) * grid_y * grid_z;
        
        const long long MAX_CELLS = 5000000;  // 可调，500万栅格以内启用预计算
        if (total_cells <= MAX_CELLS) {
            if (enable_preblocked_costmap_)
                rebuildPreblockedCells();
            else
                preblocked_cells_.clear();
            rebuildDerivedLayers();          // 填充 traversable_cells_
            rebuildPreblockedCostmap();      // 填充 preblocked_costmap_
            printf("GlobalPlanner::setOctomap() Precomputed traversable cells (%zu) and costmap (%zu) for %lld cells.\n",
                   traversable_cells_.size(), preblocked_costmap_.size(), total_cells);
        } else {
            // 地图过大，清空预计算集合，A* 将回退到实时检查
            traversable_cells_.clear();
            preblocked_costmap_.clear();
            ROS_WARN("GlobalPlanner::setOctomap() Map too large (%lld cells), skipping precomputation. A* will use real-time collision checking.", total_cells);
        }
    }

    void GlobalPlanner::makePlan(const PointPose start, const PointPose goal)
    {
        start_point_ = start;
        has_start_ = true;

        goal_point_ = goal;
        has_goal_ = true;

        printf("start = (%f,%f,%f),goal = (%f,%f,%f) \n",
               start_point_.x, start_point_.y, start_point_.z,
               goal_point_.x, goal_point_.y, goal_point_.z);
         
        tryPlan();
    }

    void GlobalPlanner::tryPlan()
    {
        printf("GlobalPlanner::tryPlan planning...\n");
        if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_) 
        {
            printf("GlobalPlanner::tryPlan 异常退出规划器\n");
            return;
        }
        planning_in_progress_ = true;
        ++plan_seq_;
        const bool ok = startPlan();
        planning_in_progress_ = false;
        if (!ok) 
        {
            printf("GlobalPlanner::tryPlan() A* planning failed. \n");
        } 
        else 
        {
            last_success_seq_ = plan_seq_;
        }
    }

    void GlobalPlanner::getPlannerResults(std::vector<PointPose>& plannerResults)
    {
       plannerResults = planner_results_;
    }

    bool GlobalPlanner::isPathValid(const std::vector<PointPose>& path,
                                    std::size_t start_index) const
    {
        if (!octree_ || path.empty() || start_index >= path.size())
            return false;

        for (std::size_t i = start_index; i < path.size(); ++i)
        {
            const GridIndex from = worldToGrid(path[i].x, path[i].y, path[i].z);
            const GridIndex to = i + 1 < path.size() ?
                worldToGrid(path[i + 1].x, path[i + 1].y, path[i + 1].z) : from;
            const int dx = to.x - from.x;
            const int dy = to.y - from.y;
            const int dz = to.z - from.z;
            const int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz), 1});
            for (int step = 0; step <= steps; ++step)
            {
                const double ratio = static_cast<double>(step) / static_cast<double>(steps);
                const GridIndex sample{
                    static_cast<int>(std::lround(from.x + ratio * dx)),
                    static_cast<int>(std::lround(from.y + ratio * dy)),
                    static_cast<int>(std::lround(from.z + ratio * dz))};
                if (!isCellTraversable(
                        sample, robot_radius_, require_ground_support_,
                        strict_direct_ground_support_, ground_support_xy_radius_cells_,
                        ground_support_depth_cells_))
                    return false;
            }
        }
        return true;
    }

    // ------------------------------------------------------------
    //  A* 主循环（已优化）
    // ------------------------------------------------------------
    bool GlobalPlanner::startPlan()
    {
        // Never expose a path produced for an older map/start/goal when the
        // current planning attempt fails.
        planner_results_.clear();

        const double robot_radius = robot_radius_;
        const int max_iterations = max_iterations_;
        const int snap_radius = snap_search_radius_cells_;
        const bool require_ground_support = require_ground_support_; 
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells = ground_support_xy_radius_cells_;
        const int support_depth_cells = ground_support_depth_cells_;
        const bool enable_preblocked_costmap = enable_preblocked_costmap_;
        const double preblocked_costmap_weight = preblocked_costmap_weight_;
        const double z_cost_penalty_factor = z_cost_penalty_factor_;
        const bool require_ground_support_for_upward = require_ground_support_for_upward_;

        // 判断是否使用预计算集合
        const bool use_precomputed = !traversable_cells_.empty();

        const GridIndex start_raw = worldToGrid(
            start_point_.x, start_point_.y, start_point_.z);
        const GridIndex goal_raw = worldToGrid(
            goal_point_.x, goal_point_.y, goal_point_.z);

        GridIndex start = start_raw;
        GridIndex goal = goal_raw;
        const bool start_ok = findNearestFreeCell(
            start_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells, start);
        const bool goal_ok = findNearestFreeCell(
            goal_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells, goal);

        if (!start_ok) 
        {
            printf("GlobalPlanner::startPlan() Start is occupied/out of map and no nearby free cell.\n");
            return false;
        }

        if (!goal_ok) 
        {
            printf("GlobalPlanner::startPlan() Goal is occupied/out of map and no nearby free cell.\n");
            return false;
        }

        if (!(start == start_raw)) 
        {
            const auto p = gridToWorld(start);
            printf("GlobalPlanner::startPlan() Start snapped to free cell: [%.2f, %.2f, %.2f] \n",
                   p.x(), p.y(), p.z());
        }

        if (!(goal == goal_raw))
        {
            const auto p = gridToWorld(goal);
            printf("GlobalPlanner::startPlan() Goal snapped to free cell: [%.2f, %.2f, %.2f] \n",
                   p.x(), p.y(), p.z());
        }

        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
        std::unordered_map<GridIndex, double, GridIndexHash> g_score;
        std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
        std::unordered_set<GridIndex, GridIndexHash> closed_set;

        g_score[start] = 0.0;
        open_set.push(QueueNode{start, euclidean(start, goal), 0.0});

        const std::vector<GridIndex> directions = make26Directions();
        int iters = 0;
        bool found = false;

        while (!open_set.empty() && iters < max_iterations) 
        {
            const QueueNode current = open_set.top();
            open_set.pop();
            ++iters;

            if (closed_set.find(current.idx) != closed_set.end()) {
                continue;
            }
            closed_set.insert(current.idx);

            if (current.idx == goal) {
                found = true;
                break;
            }

            for (const auto & d : directions) 
            {
                GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
                if (closed_set.find(nbr) != closed_set.end()) {
                    continue;
                }

                const auto traversable_cell = [&](const GridIndex& cell) {
                    if (use_precomputed)
                        return traversable_cells_.find(cell) != traversable_cells_.end();
                    return isCellTraversable(
                        cell, robot_radius, require_ground_support, strict_direct_ground_support,
                        support_xy_radius_cells, support_depth_cells);
                };

                // Prevent a horizontal diagonal from cutting through the
                // corner of two occupied cells.
                if (d.z == 0 && d.x != 0 && d.y != 0 &&
                    (!traversable_cell(GridIndex{current.idx.x + d.x, current.idx.y, current.idx.z}) ||
                     !traversable_cell(GridIndex{current.idx.x, current.idx.y + d.y, current.idx.z})))
                    continue;

                const bool traversable = traversable_cell(nbr);
                if (!traversable) {
                    continue;
                }

                // 上升移动需要地面支撑：防止凭空爬升到二楼
                // 使用宽松模式（strict=false），在下方3×3区域搜索支撑体素，容忍楼梯体素间隙
                if (d.z > 0 && require_ground_support_for_upward_) {
                    if (!hasGroundSupport(nbr, false,
                                          support_xy_radius_cells, support_depth_cells)) {
                        continue;
                    }
                }

                double step_cost = euclidean(current.idx, nbr);
                // 高度变化惩罚：让A*倾向于同层移动，只在楼梯处爬升
                if (d.z != 0) {
                    step_cost *= z_cost_penalty_factor_;
                }
                double tentative_g = current.g + step_cost;
                if (enable_preblocked_costmap) {
                    tentative_g += preblocked_costmap_weight * getPreblockedCost(nbr);
                }

                auto g_it = g_score.find(nbr);
                if (g_it == g_score.end() || tentative_g < g_it->second) {
                    came_from[nbr] = current.idx;
                    g_score[nbr] = tentative_g;
                    const double f = tentative_g + euclidean(nbr, goal);
                    open_set.push(QueueNode{nbr, f, tentative_g});
                }
            }
        }

        if (!found)
            return false;

        // 重建原始路径（栅格序列）
        auto raw_path = reconstructPath(came_from, goal);
        // 剪枝
        auto pruned_path = prunePath(raw_path);
        printf("GlobalPlanner::startPlan() A* path found in %d iterations. raw waypoints=%zu, pruned waypoints=%zu \n", 
               iters, raw_path.size(), pruned_path.size());

        // 转换为 PointPose 并存储
        planner_results_.reserve(pruned_path.size());
        for (const auto & idx : pruned_path) {
            auto p = gridToWorld(idx);
            planner_results_.push_back({static_cast<double>(p.x()),
                                        static_cast<double>(p.y()),
                                        static_cast<double>(p.z())});
        }
        return true;
    }

    // ------------------------------------------------------------
    // 路径剪枝（去除共线点）
    // ------------------------------------------------------------
    std::vector<GridIndex> GlobalPlanner::prunePath(const std::vector<GridIndex>& path) const
    {
        if (path.size() < 3) return path;
        std::vector<GridIndex> pruned;
        pruned.reserve(path.size());
        pruned.push_back(path.front());
        for (size_t i = 1; i < path.size() - 1; ++i) {
            const auto& p0 = path[i-1];
            const auto& p1 = path[i];
            const auto& p2 = path[i+1];
            // 检查是否共线（叉积为零）
            int dx1 = p1.x - p0.x, dy1 = p1.y - p0.y, dz1 = p1.z - p0.z;
            int dx2 = p2.x - p1.x, dy2 = p2.y - p1.y, dz2 = p2.z - p1.z;
            // 三维叉积的每个分量均为零表示共线
            if (dx1*dy2 - dx2*dy1 != 0 || dx1*dz2 - dx2*dz1 != 0 || dy1*dz2 - dy2*dz1 != 0) {
                pruned.push_back(p1);
            }
        }
        pruned.push_back(path.back());
        return pruned;
    }

    // ------------------------------------------------------------
    // 原有辅助函数（部分已修改使用 inv_resolution_）
    // ------------------------------------------------------------
    std::vector<GridIndex> GlobalPlanner::reconstructPath(
        const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
        GridIndex current) const
    {
        std::vector<GridIndex> path;
        path.push_back(current);
        while (came_from.find(current) != came_from.end()) 
        {
            current = came_from.at(current);
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    bool GlobalPlanner::findNearestFreeCell(const GridIndex & seed, double robot_radius, 
                                            int radius_cells, bool require_ground_support,
                                            bool strict_direct_ground_support, 
                                            int support_xy_radius_cells, int support_depth_cells,
                                            GridIndex & out) const
    {
        // 使用预计算集合或实时检查均可，此处为了统一，使用 isCellTraversable（如果预计算存在，可能会重复计算，但只在起终点调用，影响小）
        // 为了性能，可以优先查 traversable_cells_，但 findNearestFreeCell 只在起终点调用，可忽略
        if (isCellTraversable(
            seed, robot_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells))
        {
            out = seed;
            return true;
        }

        for (int r = 1; r <= radius_cells; ++r) {
            for (int dz = 0; dz <= r; ++dz) {
                for (int dx = -r; dx <= r; ++dx) {
                    for (int dy = -r; dy <= r; ++dy) {
                        if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
                            continue;
                        }

                        GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
                        if (isCellTraversable(
                            c1, robot_radius, require_ground_support, strict_direct_ground_support,
                            support_xy_radius_cells, support_depth_cells))
                        {
                            out = c1;
                            return true;
                        }

                        if (dz > 0) {
                            GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
                            if (isCellTraversable(
                                c2, robot_radius, require_ground_support, strict_direct_ground_support,
                                support_xy_radius_cells, support_depth_cells))
                            {
                                out = c2;
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    double GlobalPlanner::getPreblockedCost(const GridIndex & idx) const
    {
        const auto it = preblocked_costmap_.find(idx);
        if (it == preblocked_costmap_.end()) {
            return 0.0;
        }
        return it->second;
    }

    std::vector<GridIndex> GlobalPlanner::make26Directions() const
    {
        std::vector<GridIndex> dirs;
        dirs.reserve(24);
        for (int dx = -1; dx <= 1; ++dx) 
        {
            for (int dy = -1; dy <= 1; ++dy) 
            {
                for (int dz = -1; dz <= 1; ++dz) 
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }
                    // A wheeled robot cannot move vertically without making
                    // horizontal progress, even if the voxel above is free.
                    if (dx == 0 && dy == 0 && dz != 0)
                        continue;
                    dirs.push_back(GridIndex{dx, dy, dz});
                }
            }
        }
        return dirs;
    }

    bool GlobalPlanner::isCellTraversable(const GridIndex & idx, double robot_radius, 
                                          bool require_ground_support, bool strict_direct_ground_support,
                                          int support_xy_radius_cells, int support_depth_cells) const
    {
        if (!octree_) {
            return false;
        }
        
        if (!isInsideMetricBounds(idx)) {
            return false;
        }

        if (require_ground_support &&
            !hasGroundSupport(
                idx, strict_direct_ground_support, support_xy_radius_cells, support_depth_cells))
        {
            return false;
        }

        const octomap::point3d center = gridToWorld(idx);
        const double r = octree_->getResolution();
        const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
        const int height_cells = std::max(1, static_cast<int>(std::ceil(robot_height_ / r)));
        const double radius_sq = robot_radius * robot_radius;
        
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

        for (int dx = -n; dx <= n; ++dx) {
            for (int dy = -n; dy <= n; ++dy) {
                const double dist_x = static_cast<double>(dx) * r;
                const double dist_y = static_cast<double>(dy) * r;
                const double horizontal_dist_sq = dist_x * dist_x + dist_y * dist_y;
                if (horizontal_dist_sq > radius_sq)
                    continue;
                for (int dz = 0; dz <= height_cells; ++dz) {
                    const octomap::point3d p(
                        center.x() + static_cast<float>(dx * r),
                        center.y() + static_cast<float>(dy * r),
                        center.z() + static_cast<float>(dz * r));
                    
                    if (p.x() < static_cast<float>(min_x) - 0.01f || p.x() > static_cast<float>(max_x) + 0.01f ||
                        p.y() < static_cast<float>(min_y) - 0.01f || p.y() > static_cast<float>(max_y) + 0.01f ||
                        p.z() < static_cast<float>(min_z) - 0.01f || p.z() > static_cast<float>(max_z) + 0.01f) {
                        continue;
                    }
                    
                    const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
                    if (preblocked_cells_.find(nearby_idx) != preblocked_cells_.end()) {
                        return false;
                    }
                    const octomap::OcTreeNode * node = octree_->search(p);
                    if (node && octree_->isNodeOccupied(node)) {
                        const double max_ground_rise = robot_radius *
                            tan(max_traversable_slope_deg_ * M_PI / 180.0) + r;
                        const bool within_ground_band = static_cast<double>(dz) * r <= max_ground_rise;
                        const octomap::point3d above(
                            p.x(), p.y(), p.z() + static_cast<float>(r));
                        const octomap::OcTreeNode* above_node = octree_->search(above);
                        const bool vertically_continuous =
                            above_node && octree_->isNodeOccupied(above_node);
                        // A smooth ramp intersects a world-aligned cylindrical
                        // footprint near its lower edge. Permit only exposed
                        // top-surface voxels within the configured slope band;
                        // vertical structures and higher obstacles still block.
                        if (within_ground_band && !vertically_continuous)
                            continue;
                        return false;
                    }
                }
            }
        }
        return true;
    }

    bool GlobalPlanner::hasGroundSupport(const GridIndex & idx, bool strict_direct_ground_support, 
                                         int support_xy_radius_cells, int support_depth_cells) const
    {
        if (!octree_) {
            return false;
        }
        
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        
        if (strict_direct_ground_support) {
            GridIndex below{idx.x, idx.y, idx.z - 1};
            if (!isInsideMetricBounds(below)) {
                return false;
            }
            const auto p = gridToWorld(below);
            
            if (p.x() < static_cast<float>(min_x) - 0.01f || p.x() > static_cast<float>(max_x) + 0.01f ||
                p.y() < static_cast<float>(min_y) - 0.01f || p.y() > static_cast<float>(max_y) + 0.01f ||
                p.z() < static_cast<float>(min_z) - 0.01f || p.z() > static_cast<float>(max_z) + 0.01f) {
                return false;
            }
            
            const octomap::OcTreeNode * node = octree_->search(p);
            return node && octree_->isNodeOccupied(node);
        }

        for (int dz = 1; dz <= std::max(1, support_depth_cells); ++dz) {
            for (int dx = -support_xy_radius_cells; dx <= support_xy_radius_cells; ++dx) {
                for (int dy = -support_xy_radius_cells; dy <= support_xy_radius_cells; ++dy) {
                    GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
                    if (!isInsideMetricBounds(below)) {
                        continue;
                    }
                    const auto p = gridToWorld(below);
                    
                    if (p.x() < static_cast<float>(min_x) - 0.01f || p.x() > static_cast<float>(max_x) + 0.01f ||
                        p.y() < static_cast<float>(min_y) - 0.01f || p.y() > static_cast<float>(max_y) + 0.01f ||
                        p.z() < static_cast<float>(min_z) - 0.01f || p.z() > static_cast<float>(max_z) + 0.01f) {
                        continue;
                    }
                    
                    const octomap::OcTreeNode * node = octree_->search(p);
                    if (node && octree_->isNodeOccupied(node)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void GlobalPlanner::rebuildPreblockedCells()
    {
        preblocked_cells_.clear();
        if (!octree_) {
            return;
        }

        std::unordered_set<GridIndex, GridIndexHash> candidates;
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (!octree_->isNodeOccupied(*it)) {
                continue;
            }
            const GridIndex occ = worldToGrid(it.getX(), it.getY(), it.getZ());
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    GridIndex candidate{occ.x + dx, occ.y + dy, occ.z};
                    if (!isInsideMetricBounds(candidate)) {
                        continue;
                    }
                    candidates.insert(candidate);
                }
            }
        }

        for (const auto & c : candidates) {
            if (!isInsideMetricBounds(c)) {
                continue;
            }
            if (isOccupiedCell(c)) {
                continue;
            }
            const GridIndex below0{c.x, c.y, c.z - 1};
            const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
            if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
                preblocked_cells_.insert(c);
                continue;
            }
            const GridIndex above1{c.x, c.y, c.z + 1};
            const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
            if (!hasNonOccupiedNeighborSameLevel(c)) {
                continue;
            }
            if (above1_occ) {
                continue;
            }
            const GridIndex below1{c.x, c.y, c.z - 1};
            if (!isInsideMetricBounds(below1)) {
                continue;
            }
            const bool below1_non_occupied = !isOccupiedCell(below1);
            if (below1_non_occupied) {
                preblocked_cells_.insert(c);
            }
        }

        for (const auto & c : external_preblocked_cells_) {
            if (isInsideMetricBounds(c) && !isOccupiedCell(c)) {
                preblocked_cells_.insert(c);
            }
        }
        printf("Preprocess mask rebuilt. preblocked_cells=%zu external=%zu \n",
               preblocked_cells_.size(), external_preblocked_cells_.size());
    }

    bool GlobalPlanner::hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
                if (!isInsideMetricBounds(n)) {
                    continue;
                }
                const GridIndex n_above1{n.x, n.y, n.z + 1};
                if (!isInsideMetricBounds(n_above1)) {
                    continue;
                }
                if (isOccupiedCell(n_above1)) {
                    return true;
                }
            }
        }
        return false;
    }

    bool GlobalPlanner::hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
                if (!isInsideMetricBounds(n)) {
                    continue;
                }
                if (!isOccupiedCell(n)) {
                    return true;
                }
            }
        }
        return false;
    }

    void GlobalPlanner::rebuildDerivedLayers()
    {
        traversable_cells_.clear();
        if (!octree_) {
            return;
        }

        const bool require_ground_support = require_ground_support_;
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells = ground_support_xy_radius_cells_;  
        const int support_depth_cells = ground_support_depth_cells_;
        const double robot_radius = robot_radius_;
        const bool lowest_traversable_only = lowest_traversable_only_;

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

        const double max_dimension = 1000.0;
        if (std::abs(min_x) > max_dimension || std::abs(max_x) > max_dimension ||
            std::abs(min_y) > max_dimension || std::abs(max_y) > max_dimension ||
            std::abs(min_z) > max_dimension || std::abs(max_z) > max_dimension) {
            printf("GlobalPlanner::rebuildDerivedLayers() ERROR: Map bounds are abnormally large, skipping traversable cell computation!\n");
            return;
        }

        const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
        const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

        const int max_grid_range = 10000;
        if (max_idx.x - min_idx.x > max_grid_range ||
            max_idx.y - min_idx.y > max_grid_range ||
            max_idx.z - min_idx.z > max_grid_range) {
            printf("GlobalPlanner::rebuildDerivedLayers() ERROR: Grid range exceeds safety limit, skipping traversable cell computation!\n");
            return;
        }

        for (int x = min_idx.x; x <= max_idx.x; ++x) {
            for (int y = min_idx.y; y <= max_idx.y; ++y) {
                for (int z = min_idx.z; z <= max_idx.z; ++z) {
                    const GridIndex idx{x, y, z};
                    if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) {
                        continue;
                    }
                    if (isCellTraversable(
                        idx, robot_radius, require_ground_support, strict_direct_ground_support,
                        support_xy_radius_cells, support_depth_cells))
                    {
                        traversable_cells_.insert(idx);
                        if (lowest_traversable_only) {
                            break;
                        }
                    }
                }
            }
        }
    }

    bool GlobalPlanner::isInsideMetricBounds(const GridIndex & idx) const
    {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const auto p = gridToWorld(idx);
        return p.x() >= static_cast<float>(min_x) && p.x() <= static_cast<float>(max_x) &&
            p.y() >= static_cast<float>(min_y) && p.y() <= static_cast<float>(max_y) &&
            p.z() >= static_cast<float>(min_z) && p.z() <= static_cast<float>(max_z);
    }

    bool GlobalPlanner::isOccupiedCell(const GridIndex & idx) const
    {
        if (!octree_) {
            return false;
        }
        if (!isInsideMetricBounds(idx)) {
            return false;
        }
        const auto p = gridToWorld(idx);
        
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        
        if (p.x() < static_cast<float>(min_x) - 0.01f || p.x() > static_cast<float>(max_x) + 0.01f ||
            p.y() < static_cast<float>(min_y) - 0.01f || p.y() > static_cast<float>(max_y) + 0.01f ||
            p.z() < static_cast<float>(min_z) - 0.01f || p.z() > static_cast<float>(max_z) + 0.01f) {
            return false;
        }
        
        const octomap::OcTreeNode * node = octree_->search(p);
        return node && octree_->isNodeOccupied(node);
    }

    // ------------------------------------------------------------
    // 坐标转换（使用 inv_resolution_ 加速）
    // ------------------------------------------------------------
    GridIndex GlobalPlanner::worldToGrid(double x, double y, double z) const
    {
        // 使用乘倒数代替除法
        return GridIndex{
            static_cast<int>(std::floor(x * inv_resolution_)),
            static_cast<int>(std::floor(y * inv_resolution_)),
            static_cast<int>(std::floor(z * inv_resolution_))
        };
    }

    octomap::point3d GlobalPlanner::gridToWorld(const GridIndex & idx) const
    {
        const double r = 1.0 / inv_resolution_;  // 分辨率
        return octomap::point3d(
            static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
            static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
            static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
    }

    // ------------------------------------------------------------
    // 预阻塞代价图重建（保持原样）
    // ------------------------------------------------------------
    void GlobalPlanner::rebuildPreblockedCostmap()
    {
        preblocked_costmap_.clear();
        if (!octree_) {
            return;
        }
        const bool enable = enable_preblocked_costmap_;
        if (!enable) {
            return;
        }

        const int radius_cells = std::max(
            1, static_cast<int>(preblocked_costmap_radius_cells_));
        const double denom = static_cast<double>(radius_cells) + 1.0;

        for (const auto & c : preblocked_cells_) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const GridIndex n{c.x + dx, c.y + dy, c.z + dz};
                        if (!isInsideMetricBounds(n)) {
                            continue;
                        }
                        if (traversable_cells_.find(n) == traversable_cells_.end()) {
                            continue;
                        }
                        if (preblocked_cells_.find(n) != preblocked_cells_.end()) {
                            continue;
                        }
                        const double d = std::sqrt(
                            static_cast<double>(dx * dx + dy * dy + dz * dz));
                        if (d > static_cast<double>(radius_cells)) {
                            continue;
                        }
                        const double cst = std::max(0.0, (denom - d) / denom);
                        auto it = preblocked_costmap_.find(n);
                        if (it == preblocked_costmap_.end() || cst > it->second) {
                            preblocked_costmap_[n] = cst;
                        }
                    }
                }
            }
        }

        printf("Preblocked costmap rebuilt. cells=%zu radius=%d \n",
               preblocked_costmap_.size(), radius_cells);
    }

    // ------------------------------------------------------------
    // 遗留函数：onGoalPose（保留为空，以防外部调用）
    // ------------------------------------------------------------
    void GlobalPlanner::onGoalPose(const PointPose goal)
    {
        // 空实现
    }

}
