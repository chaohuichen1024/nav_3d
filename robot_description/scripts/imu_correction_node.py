#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf.transformations as tf_trans
import tf2_ros
from geometry_msgs.msg import Quaternion

class IMUCorrector:
    def __init__(self):
        rospy.init_node('imu_correction_node')
        
        # 参数
        self.smoothing_window = 10
        self.velocity_history = []
        self.time_history = []
        self.max_history = 50
        
        # 速度校准参数 - 基于odom与cmd_vel的比例
        self.velocity_scale_factors = {
            'linear_x': 0.944,  # odom_x / cmd_vel_x
            'angular_z': 0.70   # odom_angular_z / cmd_vel_angular_z
        }
        
        # 坐标系修正
        self.imu_frame_id = "imu_link"
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.publish_static_tf()
        
        # 订阅原始话题
        self.imu_sub = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 发布修正后的话题
        self.corrected_imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.velocity_comparison_pub = rospy.Publisher('/debug/velocity_comparison', TwistStamped, queue_size=10)
        
        # 状态变量
        self.last_odom_time = None
        self.last_odom_linear_vel = np.zeros(3)
        self.last_odom_angular_vel = np.zeros(3)
        self.last_cmd_vel = None
        self.cmd_vel_to_odom_ratio = {'linear_x': 1.0, 'angular_z': 1.0}
        
        rospy.loginfo("IMU校正节点已启动，使用odom作为基准")
    
    def publish_static_tf(self):
        """发布imu_link到base_link的静态坐标变换"""
        static_transform = TransformStamped()
        
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "base_link"
        static_transform.child_frame_id = self.imu_frame_id
        
        static_transform.transform.translation.x = 0.265
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.5355
        
        quat = tf_trans.quaternion_from_euler(0, 0, 0)
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(static_transform)
        rospy.loginfo(f"发布了静态TF变换: base_link -> {self.imu_frame_id}")
    
    def cmd_vel_callback(self, msg):
        """记录速度命令，用于比较但不用于校正"""
        self.last_cmd_vel = {
            'linear': np.array([msg.linear.x, msg.linear.y, msg.linear.z]),
            'angular': np.array([msg.angular.x, msg.angular.y, msg.angular.z]),
            'time': rospy.Time.now().to_sec()
        }
    
    def odom_callback(self, msg):
        """处理里程计数据 - 主要的速度数据源"""
        current_time = msg.header.stamp.to_sec()
        
        # 从odom获取实际速度
        linear_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y, 
            msg.twist.twist.linear.z
        ])
        angular_vel = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        
        # 更新速度历史（主要使用odom数据）
        self.update_velocity_history(current_time, linear_vel, angular_vel, source='odom')
        
        # 计算cmd_vel到odom的比例（用于调试和校准）
        if self.last_cmd_vel is not None:
            time_diff = abs(current_time - self.last_cmd_vel['time'])
            if time_diff < 0.1:  # 只在时间接近时比较
                if abs(self.last_cmd_vel['linear'][0]) > 0.01:
                    self.cmd_vel_to_odom_ratio['linear_x'] = linear_vel[0] / self.last_cmd_vel['linear'][0]
                if abs(self.last_cmd_vel['angular'][2]) > 0.01:
                    self.cmd_vel_to_odom_ratio['angular_z'] = angular_vel[2] / self.last_cmd_vel['angular'][2]
        
        # 保存最新的odom速度用于IMU校正
        self.last_odom_linear_vel = linear_vel
        self.last_odom_angular_vel = angular_vel
        self.last_odom_time = current_time
    
    def update_velocity_history(self, current_time, linear_vel, angular_vel, source='odom'):
        """更新速度历史记录"""
        self.velocity_history.append({
            'time': current_time,
            'linear_vel': linear_vel,
            'angular_vel': angular_vel,
            'source': source
        })
        
        if len(self.velocity_history) > self.max_history:
            self.velocity_history.pop(0)
    
    def calculate_acceleration_from_odom(self):
        """基于odom数据计算加速度"""
        if len(self.velocity_history) < 2:
            return None, None
        
        # 只使用odom数据
        odom_data = [d for d in self.velocity_history if d['source'] == 'odom']
        if len(odom_data) < 2:
            return None, None
        
        recent_data = odom_data[-min(self.smoothing_window, len(odom_data)):]
        
        times = np.array([d['time'] for d in recent_data])
        linear_vels = np.array([d['linear_vel'] for d in recent_data])
        angular_vels = np.array([d['angular_vel'] for d in recent_data])
        
        dt = times[-1] - times[0]
        if dt <= 0:
            return None, None
        
        linear_accel = np.zeros(3)
        angular_accel = np.zeros(3)
        
        for i in range(3):
            if len(times) > 2:
                # 使用线性回归计算加速度
                A = np.vstack([times - times[0], np.ones(len(times))]).T
                linear_slope, _ = np.linalg.lstsq(A, linear_vels[:, i], rcond=None)[0]
                angular_slope, _ = np.linalg.lstsq(A, angular_vels[:, i], rcond=None)[0]
                linear_accel[i] = linear_slope
                angular_accel[i] = angular_slope
            else:
                # 简单差分
                linear_accel[i] = (linear_vels[-1, i] - linear_vels[0, i]) / dt
                angular_accel[i] = (angular_vels[-1, i] - angular_vels[0, i]) / dt
        
        return linear_accel, angular_accel
    
    def detect_imu_anomaly(self, imu_angular_vel, odom_angular_vel):
        """检测IMU角速度异常（以odom为基准）"""
        if odom_angular_vel is None:
            return False
        
        # 检查z轴角速度差异
        angular_diff = abs(imu_angular_vel[2] - odom_angular_vel[2])
        threshold = 0.15  # 0.15 rad/s 的阈值
        
        if angular_diff > threshold:
            rospy.logdebug(f"角速度异常: IMU={imu_angular_vel[2]:.3f}, ODOM={odom_angular_vel[2]:.3f}, 差异={angular_diff:.3f}")
        
        return angular_diff > threshold
    
    def correct_imu_data(self, imu_msg, calculated_linear_accel, calculated_angular_accel):
        """修正IMU数据，主要依赖odom数据"""
        corrected_imu = Imu()
        
        # 使用正确的坐标系
        corrected_imu.header = imu_msg.header
        corrected_imu.header.frame_id = self.imu_frame_id
        
        # 保持原始姿态数据
        corrected_imu.orientation = imu_msg.orientation
        corrected_imu.orientation_covariance = imu_msg.orientation_covariance
        
        # 线性加速度处理
        if calculated_linear_accel is not None:
            corrected_imu.linear_acceleration.x = calculated_linear_accel[0]
            corrected_imu.linear_acceleration.y = calculated_linear_accel[1]
            corrected_imu.linear_acceleration.z = 9.8  # 重力加速度
        else:
            # 使用IMU原始数据，但修正z轴重力
            corrected_imu.linear_acceleration.x = imu_msg.linear_acceleration.x
            corrected_imu.linear_acceleration.y = imu_msg.linear_acceleration.y
            corrected_imu.linear_acceleration.z = 9.8
        
        # 角速度处理 - 主要使用odom数据
        if self.last_odom_angular_vel is not None:
            # 直接使用最新的odom角速度
            corrected_imu.angular_velocity.x = self.last_odom_angular_vel[0]
            corrected_imu.angular_velocity.y = self.last_odom_angular_vel[1]
            corrected_imu.angular_velocity.z = self.last_odom_angular_vel[2]
        else:
            # 备用：使用IMU原始数据
            corrected_imu.angular_velocity = imu_msg.angular_velocity
        
        # 调整协方差 - 使用odom数据时降低不确定性
        corrected_imu.linear_acceleration_covariance = (1e-06, 0.0, 0.0, 0.0, 1e-06, 0.0, 0.0, 0.0, 1e-06)
        corrected_imu.angular_velocity_covariance = (1e-06, 0.0, 0.0, 0.0, 1e-06, 0.0, 0.0, 0.0, 1e-06)
        
        return corrected_imu
    
    def imu_callback(self, msg):
        """处理原始IMU数据"""
        # 基于odom数据计算加速度
        calculated_linear_accel, calculated_angular_accel = self.calculate_acceleration_from_odom()
        
        # 检测IMU角速度异常（以odom为基准）
        imu_angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        is_anomaly = self.detect_imu_anomaly(imu_angular_vel, self.last_odom_angular_vel)
        
        if is_anomaly:
            rospy.logwarn_throttle(5, 
                f"IMU角速度异常: IMU_z={imu_angular_vel[2]:.3f}, ODOM_z={self.last_odom_angular_vel[2] if self.last_odom_angular_vel is not None else 'N/A'}")
        
        # 修正IMU数据
        corrected_imu = self.correct_imu_data(msg, calculated_linear_accel, calculated_angular_accel)
        
        # 发布修正后的IMU数据
        self.corrected_imu_pub.publish(corrected_imu)
        
        # 发布速度比较信息（用于调试）
        self.publish_velocity_comparison()
    
    def publish_velocity_comparison(self):
        """发布速度比较信息"""
        if self.last_cmd_vel is not None and self.last_odom_angular_vel is not None:
            comparison_msg = TwistStamped()
            comparison_msg.header.stamp = rospy.Time.now()
            comparison_msg.header.frame_id = "base_link"
            
            # 线速度比较
            comparison_msg.twist.linear.x = self.last_cmd_vel['linear'][0]  # 命令速度
            comparison_msg.twist.linear.y = self.last_odom_linear_vel[0]    # 实际速度
            comparison_msg.twist.linear.z = self.cmd_vel_to_odom_ratio['linear_x']  # 比例
            
            # 角速度比较
            comparison_msg.twist.angular.x = self.last_cmd_vel['angular'][2]  # 命令角速度
            comparison_msg.twist.angular.y = self.last_odom_angular_vel[2]    # 实际角速度
            comparison_msg.twist.angular.z = self.cmd_vel_to_odom_ratio['angular_z']  # 比例
            
            self.velocity_comparison_pub.publish(comparison_msg)

if __name__ == '__main__':
    try:
        corrector = IMUCorrector()
        rospy.spin()
            
    except rospy.ROSInterruptException:
        pass