<!--
 * @Descripttion: 
 * @FilePath: /src/Readme.md
 * @Author: chaohui_chen1024
 * @Date: 2026-07-20 21:46:41
 * @LastEditors: chaohui_chen1024
 * @LastEditTime: 2026-11-05 19:12:26
-->

# 机器人建图方法

## 1.基础节点

### 启动机器人仿真

```bash
roslaunch robot_description gazebo.launch
```

### 启动手柄

```bash
roslaunch robot_joy robot_joy.launch 
```



## 2.建图节点

### 采用fast_lio2建图算法

```bash
roslaunch fast_lio2 mapping_robosense.launch
```



# 导航


## 3D导航

```bash
roslaunch octo_planner_3d offline_3d_nav_scan.launch
```
或者
```bash
roslaunch octo_planner_3d offline_3d_nav.launch
```

# 导航方式

定位：采用 FAST-LIO2 激光惯性里程计，融合三维激光雷达与 IMU，
实时输出 6 自由度位姿和配准点云，与预先建好的点云地图对齐。

全局规划：先把点云地图转成 OctoMap 三维八叉树，在八叉树上做 3D A* 搜索，
26 邻域扩展，并加入地面可站立性、坡度、机器人外形膨胀和高度代价约束，
输出一条可通行的三维路径。

局部规划：采用 DWA 动态窗口法跟踪全局路径。以机器人实测速度为中心
采样线速度/角速度，前向仿真出若干候选轨迹，按朝向、避障安全距离、
速度和与全局路径的贴合度加权评分，选出最优速度指令；同时用实时点云
做局部避障，上坡时自动限制爬坡速度，卡住超时会自动重新规划。
也可以采用SCAN-Planner算法进行局部路径规划
