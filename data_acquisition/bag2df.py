#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
in package.xml <exec_depend>rclpy</exec_depend>...
in setup.py add entry point in 'console_scripts' example:
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
the contents of the setup.cfg file should be correctly popilatted automatically, like so:
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
# from geometry_msgs.msg import PoseArray
from ros2_aruco_interfaces.msg import ArucoMarkers
# from sensor_msgs.msg import Image

from tf_transformations import euler_from_quaternion

import pandas as pd


columns_tf = ['counter', 'tag_id', 'trans x', 'trans y', 'trans z', 'rot q x', 'rot q y', 'rot q z', 'rot q w', 'roll', 'pitch', 'yaw']
columns_pose = ['counter', 'tag_id', 'trans x', 'trans y', 'trans z', 'rot q x', 'rot q y', 'rot q z', 'rot q w', 'roll', 'pitch', 'yaw']
# columns_pose = ['counter', 'trans x', 'trans y', 'trans z', 'rot q x', 'rot q y', 'rot q z', 'rot q w', 'roll', 'pitch', 'yaw']



original_columns = ['time (ns)', 'frame index',
                    'trans x original', 'trans y original', 'trans z original',
                    'rot w original', 'rot x original', 'rot y original', 'rot z original']
mot3_z3_columns = ['trans x mot3z3', 'trans y mot3z3', 'trans z mot3z3', 
                   'rot x deg mot3z3', 'rot y deg mot3z3', 'rot z deg mot3z3']
mot3_z2_columns = ['trans x mot3z2', 'trans y mot3z2', 'trans z mot3z2',
                   'rot x deg mot3z2', 'rot y deg mot3z2', 'rot z deg mot3z2']
mot3_iqr_columns = ['trans x mot3iqr', 'trans y mot3iqr', 'trans z mot3iqr', 
                    'rot x deg mot3iqr', 'rot y deg mot3iqr', 'rot z deg mot3iqr']
mot5_z3_columns = ['trans x mot5z3', 'trans y mot5z3', 'trans z mot5z3',
                   'rot x deg mot5z3', 'rot y deg mot5z3', 'rot z deg mot5z3']
mot5_z2_columns = ['trans x mot5z2', 'trans y mot5z2', 'trans z mot5z2',
                   'rot x deg mot5z2', 'rot y deg mot5z2', 'rot z deg mot5z2']
mot5_iqr_columns = ['trans x mot5iqr', 'trans y mot5iqr', 'trans z mot5iqr',
                    'rot x deg mot5iqr', 'rot y deg mot5iqr', 'rot z deg mot5iqr']
mot10_z3_columns = ['trans x mot10z3', 'trans y mot10z3', 'trans z mot10z3',
                    'rot x deg mot10z3', 'rot y deg mot10z3', 'rot z deg mot10z3']
mot10_z2_columns = ['trans x mot10z2', 'trans y mot10z2', 'trans z mot10z2',
                    'rot x deg mot10z2', 'rot y deg mot10z2', 'rot z deg mot10z2']
mot10_iqr_columns = ['trans x mot10iqr', 'trans y mot10iqr', 'trans z mot10iqr', 
                     'rot x deg mot10iqr', 'rot y deg mot10iqr', 'rot z deg mot10iqr']

time = 0


csv_name_tf = '/home/kathrin/dev_ws/csv_files/test/30_0_0_10_tf.csv'
csv_name_pose = '/home/kathrin/dev_ws/csv_files/test/30_0_0_10_aruco_poses.csv'

class TfFromBag(Node):
	
    def __init__(self):
        super().__init__('df_from_bag')
        self.tf_subscription = self.create_subscription(
			TFMessage,
			'tf',
			self.tf_listener_callback,
			10) # queue size 10
        self.tf_subscription 
        self.df_tf = pd.DataFrame(columns=columns_tf)
        self.count_tf = 0
        #self.tf_all = 1
		
        self.poses_subscription = self.create_subscription(
			ArucoMarkers,
			'aruco_markers',
			self.pose_listener_callback,
			10)
        
#         self.poses_subscription = self.create_subscription(
# 			ArucoMarkers,
# 			'aruco_markers',
# 			self.pose_listener_callback,
# 			10)
        
        self.poses_subscription
        self.df_pose = pd.DataFrame(columns=columns_pose)
        self.count_pose = 0
        self.pose_all = 1
		
#         self.frames_subscription = self.create_subscription(
# 			Image,
# 			'image_raw',
# 			self.image_listener_callback,
# 			10)
#         self.frames_subscription
#         self.count_frames = 1


    def tf_listener_callback(self, msg):
        # self.get_logger().info('TF all:' + str(self.tf_all))
        # self.tf_all +=1
        # TODO if time >= 60 * 10⁹ / < 60 * 10⁹
        if self.count_tf < 80:
            for tf in msg.transforms:
                tag_id = tf.child_frame_id
                if tag_id.endswith(':4'):
                    self.get_logger().info('Heard tf transform of: ' + str(tag_id) + ' nr ' + str(self.count_tf))
                    trans = tf.transform.translation
                    rot = tf.transform.rotation
                    (roll, pitch, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
                    row = pd.Series([self.count_tf, tag_id, trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w, roll, pitch, yaw], index=columns_tf)
                    self.df_tf = self.df_tf.append(row, ignore_index=True)
                    self.count_tf += 1
                    if self.count_tf == 80:
                        self.df_tf.to_csv(csv_name_tf, ';')
                        self.get_logger().info('Dataframe tf saved.')
                        break
						
				
    def pose_listener_callback(self, msg):
        # self.get_logger().info('POSE all:' + str(self.pose_all))
        # self.pose_all +=1
        if self.count_pose < 80:
            for i, pose in enumerate(msg.poses):
                tag_id = msg.marker_ids[i]
                if int(tag_id) == 4:
                    self.get_logger().info('Heard pose transform of: ' + str(tag_id) + ' nr ' + str(self.count_pose))
                    pos = pose.position
                    orient = pose.orientation
                    (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
                    row = pd.Series([self.count_pose, tag_id, pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w, roll, pitch, yaw], index=columns_pose)
                    self.df_pose = self.df_pose.append(row, ignore_index=True)
                    self.count_pose += 1
                    if self.count_pose == 80:
                        self.df_pose.to_csv(csv_name_pose, ';')
                        self.get_logger().info('Dataframe pose saved.')
                        break

	
    # def pose_listener_callback(self, msg):
    #     # self.get_logger().info('POSE all:' + str(self.pose_all))
    #     # self.pose_all +=1
    #     if self.count_pose < 100:
    #         for i, pose in enumerate(msg.poses):
    #             self.get_logger().info('Heard pose transform of: nr ' + str(self.count_pose))
    #             pos = pose.position
    #             orient = pose.orientation
    #             (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    #             row = pd.Series([self.count_pose, pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w, roll, pitch, yaw], index=columns_pose)
    #             self.df_pose = self.df_pose.append(row, ignore_index=True)
    #             self.count_pose += 1
    #             if self.count_pose == 100:
    #                 self.df_pose.to_csv(csv_name_pose, ';')
    #                 self.get_logger().info('Dataframe pose saved.')
    #                 break
						
    def image_listener_callback(self, img):
        self.get_logger().info('IMAGE all:' + str(self.count_frames))
        self.count_frames +=1
			
			
		

def main(args=None):
    # print('Hi from data_acquisition.')
    rclpy.init(args=args)
    
    tf_from_bag = TfFromBag()
    rclpy.spin(tf_from_bag)
    
    tf_from_bag.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
