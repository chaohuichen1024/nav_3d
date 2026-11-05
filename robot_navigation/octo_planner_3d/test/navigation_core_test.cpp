#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <boost/make_shared.hpp>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <local_planner.h>
#include <pcd2octomap_converter.h>
#include <global_planner.h>

namespace
{

TEST(OctomapRealtimeUpdate, InsertsFreeRaysAndClearsOldEndpoints)
{
  octo_planner_3d::Pcd2OctomapConverter converter;
  converter.setParams(0.2, 1, 1);
  converter.setOccupancyParams(0.7, 0.4, 0.12, 0.97);

  auto first_scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  first_scan->push_back(pcl::PointXYZ(2.0f, 0.0f, 0.0f));
  const octomap::point3d origin(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < 3; ++i)
    converter.updateFromPointCloud(first_scan, origin, 10.0);

  auto tree = converter.getOctomap();
  ASSERT_NE(tree, nullptr);
  auto* free_node = tree->search(1.0, 0.0, 0.0);
  auto* occupied_node = tree->search(2.0, 0.0, 0.0);
  ASSERT_NE(free_node, nullptr);
  ASSERT_NE(occupied_node, nullptr);
  EXPECT_FALSE(tree->isNodeOccupied(free_node));
  EXPECT_TRUE(tree->isNodeOccupied(occupied_node));

  auto farther_scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  farther_scan->push_back(pcl::PointXYZ(3.0f, 0.0f, 0.0f));
  for (int i = 0; i < 10; ++i)
    converter.updateFromPointCloud(farther_scan, origin, 10.0);

  occupied_node = tree->search(2.0, 0.0, 0.0);
  ASSERT_NE(occupied_node, nullptr);
  EXPECT_FALSE(tree->isNodeOccupied(occupied_node));
}

TEST(LocalPlanner3DTracking, GoalToleranceIncludesHeight)
{
  octo_planner_3d::LocalPlanner planner;

  auto odom = boost::make_shared<nav_msgs::Odometry>();
  odom->header.stamp = ros::Time(1.0);
  odom->pose.pose.orientation.w = 1.0;
  planner.updateRobotState(odom);

  auto path = boost::make_shared<nav_msgs::Path>();
  path->header.frame_id = "map";
  geometry_msgs::PoseStamped goal;
  goal.pose.position.z = 1.0;
  goal.pose.orientation.w = 1.0;
  path->poses.push_back(goal);
  planner.setGlobalPath(path);

  geometry_msgs::Twist command;
  ASSERT_TRUE(planner.computeVelocity(command));
  EXPECT_FALSE(planner.isGoalReached());

  odom->header.stamp = ros::Time(1.1);
  odom->pose.pose.position.z = 0.8;
  planner.updateRobotState(odom);
  ASSERT_TRUE(planner.computeVelocity(command));
  EXPECT_TRUE(planner.isGoalReached());
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST(LocalPlannerTerrain, FitsSmoothSlopeFromSurfaceNormals)
{
  octo_planner_3d::LocalPlanner planner;
  auto odom = boost::make_shared<nav_msgs::Odometry>();
  odom->header.stamp = ros::Time(1.0);
  odom->pose.pose.orientation.w = 1.0;
  planner.updateRobotState(odom);

  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int ix = -10; ix <= 10; ++ix) {
    for (int iy = -10; iy <= 10; ++iy) {
      const float x = 0.1f * static_cast<float>(ix);
      const float y = 0.1f * static_cast<float>(iy);
      cloud->push_back(pcl::PointXYZ(x, y, 0.1f * x));
    }
  }
  planner.updateLocalCloud(cloud);

  EXPECT_NEAR(planner.getTerrainSlope(), std::atan(0.1), 0.03);
}

TEST(LocalPlannerTerrain, FlatPathIgnoresBaseHeightOffset)
{
  octo_planner_3d::LocalPlanner planner;

  // A planner waypoint is located in the free voxel above the floor, while
  // odometry is base_footprint.  A constant 12 cm offset must not turn a
  // level upper floor into an uphill command.
  auto odom = boost::make_shared<nav_msgs::Odometry>();
  odom->header.stamp = ros::Time(1.0);
  odom->pose.pose.position.z = 5.18;
  odom->pose.pose.orientation.w = 1.0;
  planner.updateRobotState(odom);

  auto path = boost::make_shared<nav_msgs::Path>();
  geometry_msgs::PoseStamped current;
  current.pose.position.z = 5.30;
  current.pose.orientation.w = 1.0;
  path->poses.push_back(current);
  geometry_msgs::PoseStamped target = current;
  target.pose.position.x = 1.2;
  path->poses.push_back(target);
  planner.setGlobalPath(path);

  geometry_msgs::Twist command;
  ASSERT_TRUE(planner.computeVelocity(command));
  EXPECT_EQ(planner.getTerrainMode(), octo_planner_3d::LocalPlanner::TerrainMode::FLAT);
}

TEST(GlobalPlannerResults, FailedPlanClearsPreviousPath)
{
  auto tree = std::make_shared<octomap::OcTree>(1.0);
  for (int x = -1; x <= 6; ++x) {
    for (int y = -1; y <= 1; ++y) {
      tree->updateNode(octomap::point3d(x + 0.5f, y + 0.5f, 0.5f), true);
      tree->updateNode(octomap::point3d(x + 0.5f, y + 0.5f, 1.5f), false);
      tree->updateNode(octomap::point3d(x + 0.5f, y + 0.5f, 2.5f), false);
    }
  }
  tree->updateInnerOccupancy();

  octo_planner_3d::GlobalPlanner planner;
  planner.setParams(0.1, 10000, 2, true, true, 0, 1,
                    false, 3, 1.5, false, 5.0, true, 0.5, 35.0);
  planner.setOctomap(tree);
  planner.makePlan({0.5, 0.5, 1.5}, {4.5, 0.5, 1.5});

  std::vector<octo_planner_3d::PointPose> path;
  planner.getPlannerResults(path);
  ASSERT_FALSE(path.empty());

  planner.makePlan({0.5, 0.5, 1.5}, {4.5, 0.5, 8.5});
  planner.getPlannerResults(path);
  EXPECT_TRUE(path.empty());
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "navigation_core_test", ros::init_options::AnonymousName);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
