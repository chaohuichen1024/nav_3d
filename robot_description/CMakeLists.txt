cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  urdf
  xacro
  roscpp
  sensor_msgs
  tf
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES gazebo_ros_robosense_laser gazebo_ros_robosense_gpu_laser gazebo_ros_robosense_laser_v2 gazebo_ros_robosense_gpu_laser_v2
  CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_ros_robosense_laser src/GazeboRosRoboSenseLaser.cpp)
target_link_libraries(gazebo_ros_robosense_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(gazebo_ros_robosense_gpu_laser src/GazeboRosRoboSenseLaser.cpp)
target_link_libraries(gazebo_ros_robosense_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)
target_compile_definitions(gazebo_ros_robosense_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)

# V2 插件: 适配 fast_livo2 robosense_ros::Point 字段格式 (timestamp=FLOAT64)
add_library(gazebo_ros_robosense_laser_v2 src/GazeboRosRoboSenseLaserV2.cpp)
target_link_libraries(gazebo_ros_robosense_laser_v2
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(gazebo_ros_robosense_gpu_laser_v2 src/GazeboRosRoboSenseLaserV2.cpp)
target_link_libraries(gazebo_ros_robosense_gpu_laser_v2
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)
target_compile_definitions(gazebo_ros_robosense_gpu_laser_v2 PRIVATE GAZEBO_GPU_RAY=1)

install(TARGETS gazebo_ros_robosense_laser gazebo_ros_robosense_gpu_laser gazebo_ros_robosense_laser_v2 gazebo_ros_robosense_gpu_laser_v2
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

