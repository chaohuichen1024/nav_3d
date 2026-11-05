#!/usr/bin/env python3
'''
Descripttion: 
FilePath: /src/robot_slam/fast_lio2/scripts/tf_bridge_node.py
Author: chaohui_chen1024
Date: 2026-07-07 17:40:47
LastEditors: chaohui_chen1024
LastEditTime: 2026-07-07 20:49:15
'''
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import quaternion_matrix

class TfBridgeNode:
    def __init__(self):
        rospy.init_node('tf_bridge_node', anonymous=True)

        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        self.parent_frame_id = rospy.get_param('~parent_frame_id', 'map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_footprint_frame_id = rospy.get_param('~base_footprint_frame_id', 'base_footprint')
        self.imu_frame_id = rospy.get_param('~imu_frame_id', 'imu_base')
        
        self.offset = None
        self.last_tf_stamp = None
        self.br = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        rospy.loginfo(f"Subscribing to odometry topic: {self.odom_topic}")
        rospy.loginfo(f"Parent frame: {self.parent_frame_id}")
        rospy.loginfo(f"Odom frame: {self.odom_frame_id}")
        rospy.loginfo(f"Base footprint frame: {self.base_footprint_frame_id}")
        rospy.loginfo(f"IMU frame: {self.imu_frame_id}")
        
        self.query_offset()
        
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        
        rospy.loginfo("TF bridge node started")

    def query_offset(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        rospy.sleep(1.0)
        
        try:
            trans = tf_buffer.lookup_transform(
                self.base_footprint_frame_id,
                self.imu_frame_id,
                rospy.Time(0),
                rospy.Duration(2.0)
            )
            
            self.offset = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])
            
            rospy.loginfo(f"Successfully queried IMU offset: {self.offset}")
            
        except Exception as e:
            rospy.logwarn(f"Failed to query IMU offset from TF: {e}")
            rospy.logwarn("Using default offset (0.265, 0, 0.5355)")
            self.offset = np.array([0.265, 0.0, 0.5355])

    def quaternion_to_rotation_matrix(self, q):
        return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    def odom_callback(self, odom_msg):
        if self.offset is None:
            return
        
        P_imu = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])
        
        Q_imu = odom_msg.pose.pose.orientation
        
        R_imu = self.quaternion_to_rotation_matrix(Q_imu)
        
        offset_in_map = R_imu @ self.offset
        
        P_base = P_imu - offset_in_map

        # FAST-LIO2 可能用同一个 header.stamp 发出多帧 /Odometry；重复以相同
        # 时间戳广播 TF 会触发 tf2 的 "TF_REPEATED_DATA" 警告并把日志刷爆。
        # 同一时间戳只广播一次 TF，/odom 仍按原样发布。
        stamp_key = odom_msg.header.stamp.to_sec()
        new_tf_stamp = (self.last_tf_stamp is None or stamp_key != self.last_tf_stamp)
        self.last_tf_stamp = stamp_key

        if new_tf_stamp:
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = self.parent_frame_id
            t.child_frame_id = self.odom_frame_id
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)

            t2 = TransformStamped()
            t2.header.stamp = odom_msg.header.stamp
            t2.header.frame_id = self.odom_frame_id
            t2.child_frame_id = self.base_footprint_frame_id
            t2.transform.translation.x = P_base[0]
            t2.transform.translation.y = P_base[1]
            t2.transform.translation.z = P_base[2]
            t2.transform.rotation = Q_imu
            self.br.sendTransform(t2)
        
        odom_out = Odometry()
        odom_out.header.stamp = odom_msg.header.stamp
        odom_out.header.frame_id = self.odom_frame_id
        odom_out.child_frame_id = self.base_footprint_frame_id
        
        odom_out.pose.pose.position.x = P_base[0]
        odom_out.pose.pose.position.y = P_base[1]
        odom_out.pose.pose.position.z = P_base[2]
        odom_out.pose.pose.orientation = Q_imu
        
        v_imu = np.array([
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z
        ])
        v_base = R_imu @ v_imu
        
        odom_out.twist.twist.linear.x = v_base[0]
        odom_out.twist.twist.linear.y = v_base[1]
        odom_out.twist.twist.linear.z = v_base[2]
        
        omega_imu = np.array([
            odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z
        ])
        omega_base = R_imu @ omega_imu
        
        odom_out.twist.twist.angular.x = omega_base[0]
        odom_out.twist.twist.angular.y = omega_base[1]
        odom_out.twist.twist.angular.z = omega_base[2]
        
        self.odom_pub.publish(odom_out)

if __name__ == '__main__':
    try:
        node = TfBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF bridge node interrupted")