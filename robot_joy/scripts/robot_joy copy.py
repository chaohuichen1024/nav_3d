#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
Descripttion: 
FilePath: /src/robot_joy/scripts/robot_joy copy.py
Author: chaohui_chen1024
Date: 2026-09-19 10:19:31
LastEditors: chaohui_chen1024
LastEditTime: 2026-09-19 10:19:32
'''

'''
Descripttion: 
FilePath: /src/robot_joy/scripts/robot_joy.py
Author: chaohui_chen1024
Date: 2025-12-04 10:14:07
LastEditors: chaohui_chen1024
LastEditTime: 2026-04-04 11:32:44
'''

import rospy
import yaml
import os
import threading
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math

# 导入导航相关的消息和服务
try:
    from robot_msgs.msg import MultiPointNavigationAction, MultiPointNavigationGoal
    from robot_msgs.srv import NavigationControl, NavigationControlResponse
    import actionlib
except ImportError:
    rospy.logwarn("无法导入导航相关消息，导航功能将不可用")

class PointRecorderNode:
    def __init__(self):
        rospy.init_node('point_recorder_node')
        
        # 模式控制
        self.current_mode = "mapping"  # "mapping" 或 "navigation"
        self.last_mode_switch_time = 0
        self.mode_switch_delay = 0.5  # 模式切换防抖时间(秒)
        
        # 位置信息
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.quaternion = (0.0, 0.0, 0.0, 1.0)
        self.points = []
        self.point_counter = 0
        
        # 动作编号
        self.current_action = 0
        
        # 按键状态
        self.record_flag = False
        self.save_flag = False
        self.clear_flag = False
        self.delete_last_flag = False
        
        # 动作选择相关
        self.action_select_mode = False  # 是否处于动作选择模式
        self.lb_pressed = False  # LB键是否被按下
        self.rt_pressed = False  # RT键是否被按下
        self.lt_pressed = False  # LT键是否被按下
        
        # 导航模式按键状态
        self.nav_start_flag = False
        self.nav_stop_flag = False
        self.nav_pause_flag = False
        self.nav_resume_flag = False
        
        # 速度控制相关
        self.last_linear_input = 0.0
        self.last_angular_input = 0.0
        self.zero_velocity_threshold = 0.05  # 速度阈值，小于这个值视为零
        self.last_cmd_vel_time = 0  # 上次发布速度命令的时间
        self.cmd_vel_timeout = 0.2  # 速度命令超时时间（秒）
        
        # 获取参数
        self.point_file = rospy.get_param('~point_file', 'navigation_points.yaml')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        
        # 手柄参数
        self.linear_axis = rospy.get_param('~axis_linear', 1)
        self.angular_axis = rospy.get_param('~axis_angular', 3)
        self.scale_linear = rospy.get_param('~scale_linear', 0.5)
        self.scale_angular = rospy.get_param('~scale_angular', 0.8)
        
        # 按键映射（Xbox手柄）
        # 通用按键
        self.mode_switch_btn = 5  # RB键 - 切换模式
        
        # 建图采点模式按键
        self.record_btn = 1          # B键 - 记录点
        self.save_btn = 2            # X键 - 保存到文件
        self.clear_btn = 6           # BACK键 - 清空所有点
        self.delete_last_btn = 7     # START键 - 删除最后一个点
        self.action_select_btn = 4   # LB键 - 动作选择模式
        self.add_action_btn = 5      # RT键 - 增加动作编号（需按下LB）
        self.subtract_action_btn = 2 # LT键 - 减少动作编号（需按下LB）
        
        # 导航模式按键
        self.nav_start_btn = 7       # START键 - 开始导航
        self.nav_stop_btn = 1        # B键 - 停止导航
        self.nav_pause_btn = 2       # X键 - 暂停导航
        self.nav_resume_btn = 3      # Y键 - 继续导航
        self.nav_jump_btn = 0        # A键 - 跳转到下一点
        
        # 创建锁，确保线程安全
        self.lock = threading.RLock()
        
        # 订阅话题
        odom_topic = rospy.get_param('~odom_topic', '/odometry/global')
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        
        # 发布话题
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.marker_pub = rospy.Publisher('/recorded_points_markers', MarkerArray, queue_size=10, latch=True)
        self.status_pub = rospy.Publisher('/recorder_status', Marker, queue_size=10)
        
        # 导航相关客户端
        self.nav_action_client = None
        self.nav_control_client = None
        self.init_navigation_clients()
        
        # 加载已存在的点
        self.load_existing_points()
        
        # 启动工作线程
        self.running = True
        self.record_thread = threading.Thread(target=self.record_loop, daemon=True)
        self.status_thread = threading.Thread(target=self.status_display_loop, daemon=True)
        
        self.record_thread.start()
        self.status_thread.start()
        
        # 显示控制信息
        self.print_control_info()
        
        rospy.loginfo("采点节点已启动，当前模式: {}".format("建图采点模式" if self.current_mode == "mapping" else "导航模式"))
        
    def init_navigation_clients(self):
        """初始化导航相关客户端"""
        try:
            # 初始化动作客户端
            self.nav_action_client = actionlib.SimpleActionClient(
                'multi_point_navigation', 
                MultiPointNavigationAction
            )
            rospy.loginfo("等待多点导航动作服务器...")
            if self.nav_action_client.wait_for_server(rospy.Duration(5.0)):
                rospy.loginfo("已连接到多点导航动作服务器")
            else:
                rospy.logwarn("无法连接到多点导航动作服务器，导航功能可能受限")
                
            # 初始化服务客户端
            rospy.wait_for_service('navigation_control', timeout=5.0)
            self.nav_control_client = rospy.ServiceProxy(
                'navigation_control',
                NavigationControl
            )
            rospy.loginfo("已初始化导航控制服务客户端")
            
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn("初始化导航客户端失败: {}".format(e))
            self.nav_action_client = None
            self.nav_control_client = None
            
    def load_existing_points(self):
        """加载已存在的点"""
        try:
            if os.path.exists(self.point_file):
                with open(self.point_file, 'r') as f:
                    config = yaml.safe_load(f)
                    
                if config and 'points' in config:
                    self.points = config['points']
                    self.point_counter = len(self.points)
                    rospy.loginfo(f"已加载 {self.point_counter} 个已存在的点")
                    
                    # 更新可视化
                    self.visualize_points()
            else:
                rospy.loginfo("未找到已有的点文件，将创建新文件")
        except Exception as e:
            rospy.logerr(f"加载已存在点失败: {e}")
            
    def odom_callback(self, msg):
        """更新当前位姿信息"""
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.z = msg.pose.pose.position.z
            self.quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            
    def joy_callback(self, msg):
        """处理手柄输入"""
        current_time = time.time()

        # 检查 axes/buttons 长度是否满足配置
        max_btn = max(self.mode_switch_btn, self.record_btn, self.save_btn,
                      self.clear_btn, self.delete_last_btn,
                      self.action_select_btn, self.nav_start_btn,
                      self.nav_stop_btn, self.nav_pause_btn, self.nav_resume_btn)
        if len(msg.axes) <= max(self.linear_axis, self.angular_axis) or \
           len(msg.buttons) <= max_btn:
            return  # 手柄按键数不足，跳过本次回调

        # 模式切换 (RB键)
        if msg.buttons[self.mode_switch_btn]:
            if current_time - self.last_mode_switch_time > self.mode_switch_delay:
                self.last_mode_switch_time = current_time
                self.switch_mode()
                rospy.sleep(0.1)  # 防抖
        
        # 读取摇杆控制
        linear_input = msg.axes[self.linear_axis] * self.scale_linear
        angular_input = msg.axes[self.angular_axis] * self.scale_angular
        
        # 只有在建图采点模式下才发布速度命令
        if self.current_mode == "mapping":
            # 检查是否有摇杆输入（超过阈值）
            has_input = (abs(linear_input) > self.zero_velocity_threshold or 
                        abs(angular_input) > self.zero_velocity_threshold)
            
            if has_input:
                # 有输入，发布速度命令
                twist = Twist()
                twist.linear.x = linear_input
                twist.angular.z = angular_input
                self.vel_pub.publish(twist)
                self.last_cmd_vel_time = current_time
                self.last_linear_input = linear_input
                self.last_angular_input = angular_input
            else:
                # 没有输入，检查是否需要发布零速度命令（防止机器人漂移）
                # 只有当之前有输入，并且距离上次发布命令超过一定时间时，才发布零速度
                if (abs(self.last_linear_input) > self.zero_velocity_threshold or 
                    abs(self.last_angular_input) > self.zero_velocity_threshold) and \
                   (current_time - self.last_cmd_vel_time > self.cmd_vel_timeout):
                    
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.vel_pub.publish(twist)
                    self.last_linear_input = 0.0
                    self.last_angular_input = 0.0
        
        # 根据当前模式处理按键
        with self.lock:
            if self.current_mode == "mapping":
                self.handle_mapping_mode(msg, current_time)
            else:
                self.handle_navigation_mode(msg)
                
    def handle_mapping_mode(self, msg, current_time):
        """处理建图采点模式下的按键"""
        # 读取建图模式按键状态
        self.record_flag = bool(msg.buttons[self.record_btn])
        self.save_flag = bool(msg.buttons[self.save_btn])
        self.clear_flag = bool(msg.buttons[self.clear_btn])
        self.delete_last_flag = bool(msg.buttons[self.delete_last_btn])
        
        # 处理LB键 - 动作选择模式
        lb_pressed = bool(msg.buttons[self.action_select_btn])
        
        if lb_pressed:
            # 读取模拟轴状态
            current_lt = msg.axes[self.subtract_action_btn]  # LT键
            current_rt = msg.axes[self.add_action_btn]      # RT键
            
            # 检测RT键按下
            rt_pressed = current_rt < -0.5
            if rt_pressed and not self.rt_pressed:
                # RT键从释放变为按下
                self.current_action += 1
                rospy.loginfo(f"动作编号增加: {self.current_action}")
            self.rt_pressed = rt_pressed
            
            # 检测LT键按下
            lt_pressed = current_lt < -0.5
            if lt_pressed and not self.lt_pressed:
                # LT键从释放变为按下
                self.current_action = max(0, self.current_action - 1)
                rospy.loginfo(f"动作编号减少: {self.current_action}")
            self.lt_pressed = lt_pressed
            
            # 更新动作选择模式状态
            self.action_select_mode = True
            self.lb_pressed = True
        else:
            # LB键释放，退出动作选择模式
            if self.lb_pressed:
                rospy.loginfo("退出动作选择模式")
            self.action_select_mode = False
            self.lb_pressed = False
            self.rt_pressed = False
            self.lt_pressed = False
                
        # 禁用导航按键
        self.nav_start_flag = False
        self.nav_stop_flag = False
        self.nav_pause_flag = False
        self.nav_resume_flag = False
        
    def handle_navigation_mode(self, msg):
        """处理导航模式下的按键"""
        # 读取导航模式按键状态
        self.nav_start_flag = bool(msg.buttons[self.nav_start_btn])
        self.nav_stop_flag = bool(msg.buttons[self.nav_stop_btn])
        self.nav_pause_flag = bool(msg.buttons[self.nav_pause_btn])
        self.nav_resume_flag = bool(msg.buttons[self.nav_resume_btn])
        
        # 禁用建图按键
        self.record_flag = False
        self.save_flag = False
        self.clear_flag = False
        self.delete_last_flag = False
        self.lb_pressed = False
        self.action_select_mode = False
        
    def switch_mode(self):
        """切换模式"""
        if self.current_mode == "mapping":
            self.current_mode = "navigation"
            # 切换到导航模式时，发送一个零速度命令，确保机器人停止
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
            rospy.loginfo("=====================================")
            rospy.loginfo("已切换到: 导航模式")
            rospy.loginfo("=====================================")
        else:
            self.current_mode = "mapping"
            rospy.loginfo("=====================================")
            rospy.loginfo("已切换到: 建图采点模式")
            rospy.loginfo("=====================================")
            
        self.print_control_info()
        
    def record_loop(self):
        """记录点的主循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        last_record_state = False
        last_save_state = False
        last_clear_state = False
        last_delete_state = False
        last_nav_start_state = False
        last_nav_stop_state = False
        last_nav_pause_state = False
        last_nav_resume_state = False
        
        while self.running and not rospy.is_shutdown():
            try:
                with self.lock:
                    # 根据当前模式处理按键
                    if self.current_mode == "mapping":
                        # 记录点（按键按下时）
                        if self.record_flag and not last_record_state:
                            self.record_point()
                            
                        # 保存到文件（按键按下时）
                        if self.save_flag and not last_save_state:
                            self.save_points()
                            
                        # 清空所有点（按键按下时）
                        if self.clear_flag and not last_clear_state:
                            self.clear_points()
                            
                        # 删除最后一个点（按键按下时）
                        if self.delete_last_flag and not last_delete_state:
                            self.delete_last_point()
                            
                    else:  # navigation mode
                        # 开始导航（按键按下时）
                        if self.nav_start_flag and not last_nav_start_state:
                            self.start_navigation()
                            
                        # 停止导航（按键按下时）
                        if self.nav_stop_flag and not last_nav_stop_state:
                            self.stop_navigation()
                            
                        # 暂停导航（按键按下时）
                        if self.nav_pause_flag and not last_nav_pause_state:
                            self.pause_navigation()
                            
                        # 继续导航（按键按下时）
                        if self.nav_resume_flag and not last_nav_resume_state:
                            self.resume_navigation()
                    
                    # 更新上一次的状态
                    last_record_state = self.record_flag
                    last_save_state = self.save_flag
                    last_clear_state = self.clear_flag
                    last_delete_state = self.delete_last_flag
                    last_nav_start_state = self.nav_start_flag
                    last_nav_stop_state = self.nav_stop_flag
                    last_nav_pause_state = self.nav_pause_flag
                    last_nav_resume_state = self.nav_resume_flag
                    
            except Exception as e:
                rospy.logerr("记录循环异常: {}".format(e))
                
            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                break
                
    def record_point(self):
        """记录当前位置点"""
        point_id = f"point{self.point_counter}"
        
        point_data = {
            'id': point_id,
            'action_type': self.current_action,
            'description': f"第{self.point_counter + 1}个导航点",
            'pose': {
                'position': {
                    'x': round(self.x, 3),
                    'y': round(self.y, 3),
                    'z': round(self.z, 3)
                },
                'orientation': {
                    'w': round(self.quaternion[3], 6),
                    'x': round(self.quaternion[0], 6),
                    'y': round(self.quaternion[1], 6),
                    'z': round(self.quaternion[2], 6)
                }
            },
            'action_params': {}
        }
        
        self.points.append(point_data)
        self.point_counter += 1
        
        rospy.loginfo("已记录点 {} (动作: {}, 位置: [{:.3f}, {:.3f}])".format(
            point_id, self.current_action, self.x, self.y))
        
        # 更新可视化
        self.visualize_points()
        
        # 重置动作编号
        self.current_action = 0
        
    def save_points(self):
        """保存所有点到文件"""
        try:
            config = {
                'points': self.points
            }
            
            with open(self.point_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
                
            rospy.loginfo("已保存 {} 个点到文件: {}".format(len(self.points), self.point_file))
            
        except Exception as e:
            rospy.logerr("保存点失败: {}".format(e))
            
    def clear_points(self):
        """清空所有点"""
        with self.lock:
            self.points = []
            self.point_counter = 0
            
            # 清空文件
            try:
                with open(self.point_file, 'w') as f:
                    yaml.dump({'points': []}, f)
            except Exception as e:
                rospy.logerr("清空文件失败: {}".format(e))
                
            # 更新可视化
            self.visualize_points()
            
            rospy.loginfo("已清空所有点")
            
    def delete_last_point(self):
        """删除最后一个点"""
        if not self.points:
            rospy.logwarn("没有点可删除")
            return
            
        with self.lock:
            deleted_point = self.points.pop()
            self.point_counter -= 1
            
            # 更新文件
            self.save_points()
            
            # 更新可视化
            self.visualize_points()
            
            rospy.loginfo("已删除点: {}".format(deleted_point['id']))
            
    def visualize_points(self):
        """可视化所有记录的点"""
        marker_array = MarkerArray()
        
        # 删除所有现有标记
        delete_marker = Marker()
        delete_marker.header.frame_id = self.map_frame
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "recorded_points"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 为每个点创建标记
        for i, point in enumerate(self.points):
            pose_data = point['pose']
            action_type = point.get('action_type', 0)
            
            # 创建球形标记
            sphere_marker = Marker()
            sphere_marker.header.frame_id = self.map_frame
            sphere_marker.header.stamp = rospy.Time.now()
            sphere_marker.ns = "recorded_points"
            sphere_marker.id = i * 3 + 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            
            sphere_marker.pose.position.x = pose_data['position']['x']
            sphere_marker.pose.position.y = pose_data['position']['y']
            sphere_marker.pose.position.z = pose_data['position']['z']
            sphere_marker.pose.orientation.w = 1.0
            
            sphere_marker.scale.x = 0.3
            sphere_marker.scale.y = 0.3
            sphere_marker.scale.z = 0.3
            
            # 根据动作类型设置颜色
            if action_type == 0:
                sphere_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # 绿色 - 无动作
            else:
                sphere_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # 蓝色 - 有动作
                
            sphere_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(sphere_marker)
            
            # 创建文本标记
            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "point_labels"
            text_marker.id = i * 3 + 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = pose_data['position']['x']
            text_marker.pose.position.y = pose_data['position']['y']
            text_marker.pose.position.z = pose_data['position']['z'] + 0.4
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.25
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text_marker.text = f"{point['id']}\nA:{action_type}"
            
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)
            
            # 创建方向箭头
            arrow_marker = Marker()
            arrow_marker.header.frame_id = self.map_frame
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "point_orientations"
            arrow_marker.id = i * 3 + 3
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            arrow_marker.pose.position.x = pose_data['position']['x']
            arrow_marker.pose.position.y = pose_data['position']['y']
            arrow_marker.pose.position.z = pose_data['position']['z']
            
            arrow_marker.pose.orientation.w = pose_data['orientation']['w']
            arrow_marker.pose.orientation.x = pose_data['orientation']['x']
            arrow_marker.pose.orientation.y = pose_data['orientation']['y']
            arrow_marker.pose.orientation.z = pose_data['orientation']['z']
            
            arrow_marker.scale.x = 0.4  # 长度
            arrow_marker.scale.y = 0.08  # 宽度
            arrow_marker.scale.z = 0.08  # 高度
            
            arrow_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # 红色
            
            arrow_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(arrow_marker)
            
        # 发布标记
        self.marker_pub.publish(marker_array)
        
    def start_navigation(self):
        """开始导航"""
        try:
            if self.nav_action_client is None:
                rospy.logerr("导航动作客户端未初始化")
                # 尝试重新初始化
                self.init_navigation_clients()
                if self.nav_action_client is None:
                    rospy.logerr("重新初始化导航客户端失败")
                    return
            
            # 检查是否有导航点
            if len(self.points) == 0:
                rospy.logerr("没有导航点，请先在建图模式下记录点")
                return
                
            # 创建导航目标
            goal = MultiPointNavigationGoal()
            goal.task_id = f"manual_navigation_{time.time()}"
            goal.from_current_pose = True
            goal.start_index = 0
            
            # 发送目标
            self.nav_action_client.send_goal(goal)
            rospy.loginfo(f"已发送导航任务: {goal.task_id}")
            rospy.loginfo(f"将导航到 {len(self.points)} 个点")
            
        except Exception as e:
            rospy.logerr("开始导航失败: {}".format(e))
            
    def stop_navigation(self):
        """停止导航"""
        try:
            if self.nav_control_client is None:
                rospy.logerr("导航控制服务客户端未初始化")
                # 尝试重新初始化
                rospy.wait_for_service('navigation_control', timeout=2.0)
                self.nav_control_client = rospy.ServiceProxy(
                    'navigation_control',
                    NavigationControl
                )
                
            response = self.nav_control_client(command=4)  # 停止命令
            if response.success:
                rospy.loginfo("已发送停止导航命令: {}".format(response.message))
            else:
                rospy.logwarn("停止导航命令失败: {}".format(response.message))
                
        except rospy.ServiceException as e:
            rospy.logerr("调用导航控制服务失败: {}".format(e))
        except Exception as e:
            rospy.logerr("停止导航失败: {}".format(e))
            
    def pause_navigation(self):
        """暂停导航"""
        try:
            if self.nav_control_client is None:
                rospy.logerr("导航控制服务客户端未初始化")
                # 尝试重新初始化
                rospy.wait_for_service('navigation_control', timeout=2.0)
                self.nav_control_client = rospy.ServiceProxy(
                    'navigation_control',
                    NavigationControl
                )
                
            response = self.nav_control_client(command=2)  # 暂停命令
            if response.success:
                rospy.loginfo("已发送暂停导航命令: {}".format(response.message))
            else:
                rospy.logwarn("暂停导航命令失败: {}".format(response.message))
                
        except rospy.ServiceException as e:
            rospy.logerr("调用导航控制服务失败: {}".format(e))
        except Exception as e:
            rospy.logerr("暂停导航失败: {}".format(e))
            
    def resume_navigation(self):
        """继续导航"""
        try:
            if self.nav_control_client is None:
                rospy.logerr("导航控制服务客户端未初始化")
                # 尝试重新初始化
                rospy.wait_for_service('navigation_control', timeout=2.0)
                self.nav_control_client = rospy.ServiceProxy(
                    'navigation_control',
                    NavigationControl
                )
                
            response = self.nav_control_client(command=3)  # 继续命令
            if response.success:
                rospy.loginfo("已发送继续导航命令: {}".format(response.message))
            else:
                rospy.logwarn("继续导航命令失败: {}".format(response.message))
                
        except rospy.ServiceException as e:
            rospy.logerr("调用导航控制服务失败: {}".format(e))
        except Exception as e:
            rospy.logerr("继续导航失败: {}".format(e))
                
    def status_display_loop(self):
        """显示状态信息"""
        rate = rospy.Rate(1)  # 1Hz
        
        while self.running and not rospy.is_shutdown():
            try:
                # 发布状态标记到RViz
                self.publish_status_marker()
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                break
                
    def publish_status_marker(self):
        """发布状态标记到RViz"""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "recorder_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 固定在某个位置显示状态
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0  # 显示在顶部
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.3
        marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)  # 黄色
        
        if self.current_mode == "mapping":
            status_text = "建图采点模式 | "
            status_text += f"已记录: {len(self.points)} 点 | "
            if self.action_select_mode:
                status_text += "LB按住中，按LT/RT选择动作 | "
            status_text += f"动作: {self.current_action}"
        else:
            status_text = "导航模式 | "
            status_text += f"已记录: {len(self.points)} 点"
            
        marker.text = status_text
        marker.lifetime = rospy.Duration(1.0)
        
        self.status_pub.publish(marker)
        
    def print_control_info(self):
        """打印控制信息"""
        rospy.loginfo("=====================================")
        rospy.loginfo("=== 手柄控制节点 - 当前模式: {} ===".format(
            "建图采点模式" if self.current_mode == "mapping" else "导航模式"
        ))
        
        if self.current_mode == "mapping":
            rospy.loginfo("=== 建图采点模式控制 ===")
            rospy.loginfo("左摇杆: 控制机器人移动")
            rospy.loginfo("B键(1): 记录当前位置为导航点")
            rospy.loginfo("X键(2): 保存所有点到文件")
            rospy.loginfo("BACK键(6): 清空所有点")
            rospy.loginfo("START键(7): 删除最后一个点")
            rospy.loginfo("LB键(4): 按住不放")
            rospy.loginfo("同时按LT键(模拟轴2): 减少动作编号")
            rospy.loginfo("同时按RT键(模拟轴5): 增加动作编号")
            rospy.loginfo("RB键(5): 切换到导航模式")
        else:
            rospy.loginfo("=== 导航模式控制 ===")
            rospy.loginfo("左摇杆: 不发布速度命令（导航控制）")
            rospy.loginfo("START键(7): 开始导航")
            rospy.loginfo("B键(1): 停止导航")
            rospy.loginfo("X键(2): 暂停导航")
            rospy.loginfo("Y键(3): 继续导航")
            rospy.loginfo("RB键(5): 切换到建图采点模式")
            
        rospy.loginfo("=====================================")
        rospy.loginfo("点文件: {}".format(self.point_file))
        rospy.loginfo("当前已记录: {} 个点".format(len(self.points)))
        rospy.loginfo("=====================================")
        
    def cleanup(self):
        """清理函数"""
        self.running = False
        rospy.loginfo("采点节点关闭，保存点...")
        self.save_points()
        
        # 发送一个零速度命令，确保机器人停止
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)
        
        # 等待线程结束
        if self.record_thread.is_alive():
            self.record_thread.join(timeout=1.0)
        if self.status_thread.is_alive():
            self.status_thread.join(timeout=1.0)
            
        rospy.loginfo("已保存点数据")

if __name__ == "__main__":
    try:
        node = PointRecorderNode()
        
        # 注册关闭时的清理函数
        rospy.on_shutdown(node.cleanup)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"采点节点异常: {e}")