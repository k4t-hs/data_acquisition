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

csv_name_tf = '30_0_0_10_tfmot3iqr.csv'
csv_name_pose = '30_0_0_10_posemot3iqr.csv'

class TfFromBag(Node):
	
    def __init__(self):
        super().__init__('df_from_bag')
        self.tf_subscription = self.create_subscription(
			TFMessage,
			'tf_mot_3_iqr',
			self.tf_listener_callback,
			10) # queue size 10
        self.tf_subscription 
        self.df_tf = pd.DataFrame(columns=columns_tf)
        self.count_tf = 0
        #self.tf_all = 1
		
        self.poses_subscription = self.create_subscription(
			ArucoMarkers,
			'pose_mot_3_iqr',
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
