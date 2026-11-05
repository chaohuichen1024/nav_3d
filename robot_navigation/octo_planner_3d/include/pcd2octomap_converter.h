#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace octo_planner_3d
{

struct Key
{
  unsigned int k[3];

  bool operator==(const Key & other) const;
};

struct KeyHash
{
  std::size_t operator()(const Key & key) const;
};

class Pcd2OctomapConverter
{
public:
  Pcd2OctomapConverter();

  bool convert();
  
  bool convertFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  
  void updateFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void clear();

  bool isPointFree(const octomap::point3d & p) const;
  bool isSpaceFree(const octomap::point3d & min_pt, const octomap::point3d & max_pt) const;
  
  std::shared_ptr<octomap::OcTree> getOctomap();

  void printOccupiedNodes() const;
  void printQueryExamples() const;
  void visualizeWithOctovis() const;

  std::shared_ptr<octomap::OcTree> getTree() const;

  void setParams(double resolution, int min_points_per_voxel, int min_cluster_voxels);
  void setInputPcdPath(const std::string& path);
  void setOutputBtPath(const std::string& path);
  void setRealtimeMode(bool enable);

private:
  bool loadPointCloud();
  void buildVoxelCounts();
  void filterByPointCount();
  void filterByConnectedClusters();
  void fillOcTree();
  bool saveOctomap() const;
  void insertPointCloudIntoOctree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private:
  std::string input_pcd_;
  std::string output_bt_;

  double resolution_;
  int min_points_per_voxel_;
  int min_cluster_voxels_;
  bool realtime_mode_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::shared_ptr<octomap::OcTree> tree_;

  std::unordered_map<Key, std::size_t, KeyHash> voxel_counts_;
  std::unordered_set<Key, KeyHash> occupied_keys_;
};

}
