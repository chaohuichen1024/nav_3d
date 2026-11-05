#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

'''
Descripttion: 手柄：手动开车 + 三维导航点记录 / 选点导航（配合 octo_planner_3d）
              B 键把机器人当前位置记成导航点并写入 YAML，A 键让机器人导航到选中的点
              （起点自动取机器人当前位姿，通过 /global_planner/goal_pose 发给全局规划器）。
FilePath: /src/robot_joy/scripts/robot_joy.py
Author: chaohui_chen1024
Date: 2025-12-04 10:14:07
LastEditors: chaohui_chen1024
LastEditTime: 2026-09-19 00:00:00
'''

import math
import os
import threading
import time

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA, Empty
from visualization_msgs.msg import Marker, MarkerArray


class NavPointTeleopNode(object):
    def __init__(self):
        rospy.init_node('nav_point_teleop')

        # ---------------- 参数 ----------------
        self.point_file = os.path.expanduser(
            rospy.get_param('~point_file', 'nav_points_3d.yaml'))
        self.map_frame = rospy.get_param('~map_frame', 'map')
        odom_topic = rospy.get_param('~odom_topic', '/odom')
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        goal_topic = rospy.get_param('~goal_topic', '/global_planner/goal_pose')
        stop_topic = rospy.get_param('~stop_topic', '/navigation/stop')

        self.linear_axis = rospy.get_param('~axis_linear', 1)
        self.angular_axis = rospy.get_param('~axis_angular', 3)
        self.scale_linear = rospy.get_param('~scale_linear', 0.5)
        self.scale_angular = rospy.get_param('~scale_angular', 0.8)

        self.deadzone = rospy.get_param('~deadzone', 0.05)
        # 导航过程中推杆超过该值即视为人工接管，避免误碰打断导航
        self.takeover_threshold = rospy.get_param('~takeover_threshold', 0.4)
        # 松杆后补发一次零速，让底盘立刻停住
        self.zero_vel_delay = rospy.get_param('~zero_vel_delay', 0.15)

        # ---------------- 按键映射（Xbox） ----------------
        self.btn_go = 0        # A键  - 导航到选中点
        self.btn_record = 1    # B键  - 记录当前位置为导航点
        self.btn_prev = 2      # X键  - 选中上一个点
        self.btn_next = 3      # Y键  - 选中下一个点
        self.btn_delete = 4    # LB键 - 删除最后一个点
        self.btn_cancel = 5    # RB键 - 取消导航 / 手动接管
        self.btn_clear = 6     # BACK键 - 清空所有点
        self.btn_list = 7      # START键 - 打印已记录的点

        # ---------------- 状态 ----------------
        self.lock = threading.RLock()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        self.points = []
        self.selected = 0
        self.navigating = False

        self.last_buttons = []
        self.had_input = False
        self.last_input_time = 0.0

        # ---------------- ROS 接口 ----------------
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber('joy', Joy, self.joy_callback)

        self.vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher(stop_topic, Empty, queue_size=1)
        self.marker_pub = rospy.Publisher('/recorded_points_markers', MarkerArray,
                                          queue_size=10, latch=True)

        self.load_points()
        self.print_control_info()

        rospy.loginfo("三维导航点手柄节点已启动（odom=%s, goal=%s, cmd_vel=%s）",
                      odom_topic, goal_topic, cmd_vel_topic)
        if not self.odom_ready:
            rospy.logwarn("尚未收到 %s，请确认定位已启动", odom_topic)

    # ================= 手柄回调 =================
    def odom_callback(self, msg):
        with self.lock:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.x = p.x
            self.y = p.y
            self.z = p.z
            self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.odom_ready = True

    def joy_callback(self, msg):
        max_axis = max(self.linear_axis, self.angular_axis)
        max_button = max(self.btn_go, self.btn_record, self.btn_prev, self.btn_next,
                         self.btn_delete, self.btn_cancel, self.btn_clear, self.btn_list)
        if len(msg.axes) <= max_axis or len(msg.buttons) <= max_button:
            return

        now = time.time()
        self.handle_buttons(msg, now)
        self.handle_teleop(msg, now)

    # ================= 手动开车 =================
    def handle_teleop(self, msg, now):
        raw_linear = msg.axes[self.linear_axis]
        raw_angular = msg.axes[self.angular_axis]
        linear = raw_linear * self.scale_linear
        angular = raw_angular * self.scale_angular
        moving = abs(linear) > self.deadzone or abs(angular) > self.deadzone

        if self.navigating:
            # 导航中只有明显推杆才接管，避免手碰一下就把导航打断（阈值按摇杆原始量判断）
            if (abs(raw_linear) <= self.takeover_threshold and
                    abs(raw_angular) <= self.takeover_threshold):
                return
            self.navigating = False
            self.cancel_pub.publish(Empty())
            rospy.logwarn("检测到摇杆输入，已接管手动控制并停止导航")

        if moving:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.vel_pub.publish(twist)
            self.last_input_time = now
            self.had_input = True
        elif self.had_input and (now - self.last_input_time) > self.zero_vel_delay:
            # 松杆后补发一次零速，底盘立刻停住；之后不再重复发零速
            self.vel_pub.publish(Twist())
            self.had_input = False

    # ================= 按键处理 =================
    def handle_buttons(self, msg, now):
        # 第一帧只记录按键状态：避免启动瞬间正被按住的键（例如 BACK 清空）被当成一次按下
        if not self.last_buttons:
            self.last_buttons = list(msg.buttons)
            return

        def pressed(idx):
            return bool(msg.buttons[idx]) and not self.last_buttons[idx]

        if pressed(self.btn_record):
            self.record_point()
        if pressed(self.btn_go):
            self.navigate_to_selected()
        if pressed(self.btn_prev):
            self.select_point(-1)
        if pressed(self.btn_next):
            self.select_point(1)
        if pressed(self.btn_delete):
            self.delete_last_point()
        if pressed(self.btn_cancel):
            self.cancel_navigation()
        if pressed(self.btn_clear):
            self.clear_points()
        if pressed(self.btn_list):
            self.print_points()

        self.last_buttons = list(msg.buttons)

    # ================= 记录 / 选择 / 删除 =================
    def record_point(self):
        if not self.odom_ready:
            rospy.logwarn("还没收到里程计，无法记录当前位置")
            return

        with self.lock:
            index = len(self.points)
            point = {
                'id': 'point%d' % index,
                'x': round(self.x, 3),
                'y': round(self.y, 3),
                'z': round(self.z, 3),
                'yaw': round(self.yaw, 4),
            }
            self.points.append(point)
            self.selected = index
            self.save_points()
            self.visualize_points()

        rospy.loginfo("已记录 %s: (%.3f, %.3f, %.3f) yaw=%.1f deg，当前共 %d 个点",
                      point['id'], point['x'], point['y'], point['z'],
                      math.degrees(point['yaw']), len(self.points))

    def select_point(self, step):
        with self.lock:
            if not self.points:
                rospy.logwarn("还没有记录任何导航点")
                return
            self.selected = (self.selected + step) % len(self.points)
            point = self.points[self.selected]
            self.visualize_points()

        rospy.loginfo("当前选中 [%d/%d] %s: (%.3f, %.3f, %.3f)",
                      self.selected + 1, len(self.points), point['id'],
                      point['x'], point['y'], point['z'])

    def delete_last_point(self):
        with self.lock:
            if not self.points:
                rospy.logwarn("没有点可删除")
                return
            removed = self.points.pop()
            self.selected = max(0, min(self.selected, len(self.points) - 1))
            self.save_points()
            self.visualize_points()
        rospy.loginfo("已删除 %s，剩余 %d 个点", removed['id'], len(self.points))

    def clear_points(self):
        with self.lock:
            count = len(self.points)
            self.points = []
            self.selected = 0
            self.save_points()
            self.visualize_points()
        rospy.logwarn("已清空全部 %d 个导航点", count)

    def print_points(self):
        with self.lock:
            if not self.points:
                rospy.loginfo("还没有记录任何导航点")
                return
            rospy.loginfo("共 %d 个导航点（选中第 %d 个）:", len(self.points), self.selected + 1)
            for i, point in enumerate(self.points):
                mark = " <== 选中" if i == self.selected else ""
                rospy.loginfo("  [%d] %s: (%.3f, %.3f, %.3f) yaw=%.1f deg%s",
                              i, point['id'], point['x'], point['y'], point['z'],
                              math.degrees(point.get('yaw', 0.0)), mark)

    # ================= 导航 =================
    def navigate_to_selected(self):
        with self.lock:
            if not self.points:
                rospy.logwarn("还没有记录任何导航点，先按 B 键记录")
                return
            point = self.points[self.selected]

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = point['x']
        goal.pose.position.y = point['y']
        goal.pose.position.z = point['z']
        goal.pose.orientation.w = 1.0

        # 先交出手动控制，再把目标点发给全局规划器（起点由规划器取当前里程计位姿）
        self.vel_pub.publish(Twist())
        self.goal_pub.publish(goal)
        self.navigating = True

        rospy.loginfo("导航到 [%d/%d] %s: (%.3f, %.3f, %.3f)",
                      self.selected + 1, len(self.points), point['id'],
                      point['x'], point['y'], point['z'])

    def cancel_navigation(self):
        self.navigating = False
        self.cancel_pub.publish(Empty())
        self.vel_pub.publish(Twist())
        rospy.logwarn("已取消导航，可以用摇杆手动开车了")

    # ================= 文件读写 =================
    def save_points(self):
        try:
            directory = os.path.dirname(self.point_file)
            if directory and not os.path.isdir(directory):
                os.makedirs(directory)
            with open(self.point_file, 'w') as f:
                yaml.dump({'points': self.points}, f,
                          default_flow_style=False, allow_unicode=True)
        except Exception as e:
            rospy.logerr("保存导航点失败: %s", str(e))

    def load_points(self):
        if not os.path.exists(self.point_file):
            rospy.loginfo("未找到点文件 %s，将从空列表开始记录", self.point_file)
            return
        try:
            with open(self.point_file, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr("读取点文件失败: %s", str(e))
            return

        if not config or 'points' not in config:
            return

        loaded = []
        for raw in config['points'] or []:
            try:
                if 'pose' in raw:  # 兼容旧脚本写的格式
                    position = raw['pose']['position']
                    loaded.append({
                        'id': raw.get('id', 'point%d' % len(loaded)),
                        'x': float(position['x']),
                        'y': float(position['y']),
                        'z': float(position['z']),
                        'yaw': float(raw.get('yaw', 0.0)),
                    })
                else:
                    loaded.append({
                        'id': raw.get('id', 'point%d' % len(loaded)),
                        'x': float(raw['x']),
                        'y': float(raw['y']),
                        'z': float(raw['z']),
                        'yaw': float(raw.get('yaw', 0.0)),
                    })
            except (KeyError, TypeError, ValueError) as e:
                rospy.logwarn("跳过无法解析的点: %s", str(e))

        with self.lock:
            self.points = loaded
            self.selected = 0
            self.visualize_points()

        if loaded:
            rospy.loginfo("已从 %s 载入 %d 个导航点", self.point_file, len(loaded))

    # ================= RViz 可视化 =================
    def visualize_points(self):
        marker_array = MarkerArray()

        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = rospy.Time.now()
        clear.ns = 'nav_points'
        clear.id = 0
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for i, point in enumerate(self.points):
            selected = (i == self.selected)

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = 'nav_points'
            sphere.id = i * 3 + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = point['x']
            sphere.pose.position.y = point['y']
            sphere.pose.position.z = point['z']
            sphere.pose.orientation.w = 1.0
            size = 0.45 if selected else 0.28
            sphere.scale.x = size
            sphere.scale.y = size
            sphere.scale.z = size
            sphere.color = ColorRGBA(0.1, 1.0, 0.1, 1.0) if selected else \
                ColorRGBA(0.2, 0.4, 1.0, 1.0)
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = rospy.Time.now()
            text.ns = 'nav_points'
            text.id = i * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = point['x']
            text.pose.position.y = point['y']
            text.pose.position.z = point['z'] + 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text.text = '%d:%s' % (i, point['id'])
            marker_array.markers.append(text)

            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = 'nav_points'
            arrow.id = i * 3 + 3
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = point['x']
            arrow.pose.position.y = point['y']
            arrow.pose.position.z = point['z']
            yaw = point.get('yaw', 0.0)
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x = 0.5
            arrow.scale.y = 0.08
            arrow.scale.z = 0.08
            arrow.color = ColorRGBA(1.0, 0.4, 0.0, 1.0)
            marker_array.markers.append(arrow)

        self.marker_pub.publish(marker_array)

    # ================= 提示 =================
    def print_control_info(self):
        rospy.loginfo("==================== 手柄控制 ====================")
        rospy.loginfo("左摇杆     : 手动开车（前/后 + 左/右转）")
        rospy.loginfo("B键        : 记录机器人当前位置为导航点")
        rospy.loginfo("A键        : 导航到选中的点（起点=当前位置）")
        rospy.loginfo("X键 / Y键  : 选中上一个 / 下一个点")
        rospy.loginfo("LB键       : 删除最后一个点")
        rospy.loginfo("RB键       : 取消导航（也可直接推杆接管）")
        rospy.loginfo("BACK键     : 清空所有点")
        rospy.loginfo("START键    : 在终端打印已记录的点")
        rospy.loginfo("点文件     : %s", self.point_file)
        rospy.loginfo("=================================================")

    def cleanup(self):
        try:
            self.vel_pub.publish(Twist())
        except Exception:
            pass
        with self.lock:
            self.save_points()
        rospy.loginfo("导航点手柄节点退出，已保存 %d 个点", len(self.points))


if __name__ == "__main__":
    node = None
    try:
        node = NavPointTeleopNode()
        rospy.on_shutdown(node.cleanup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("导航点手柄节点异常: %s", str(e))
