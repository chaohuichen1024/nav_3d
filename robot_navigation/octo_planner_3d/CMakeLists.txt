cmake_minimum_required(VERSION 2.8.3)
project(octo_planner_3d)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  octomap_msgs
  actionlib
  move_base_msgs
  tf
)

find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES octomap_builder global_planner local_planner
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
                 pcl_conversions octomap_msgs actionlib move_base_msgs tf
  DEPENDS PCL octomap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(octomap_builder
  src/pcd2octomap_converter.cpp
)

add_library(global_planner
  src/global_planner.cpp
)

add_library(local_planner
  src/local_planner.cpp
)

add_executable(octomap_builder_node src/octomap_builder_node.cpp)
target_link_libraries(octomap_builder_node
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(global_planner_node src/global_planner_node.cpp)
target_link_libraries(global_planner_node
  global_planner
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node
  local_planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(navigation_core_test test/navigation_core_test.cpp)
  if(TARGET navigation_core_test)
    target_link_libraries(navigation_core_test
      octomap_builder
      global_planner
      local_planner
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OCTOMAP_LIBRARIES}
    )
  endif()
endif()

install(TARGETS octomap_builder global_planner local_planner 
        octomap_builder_node global_planner_node local_planner_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
