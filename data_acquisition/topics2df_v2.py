#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
import scipy.stats as stats
import warnings

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from ros2_aruco_interfaces.msg import ArucoMarkers


topic_types = ['original', 
               'mot3z3', 'mot3z2' ,'mot3iqr',
               'mot5z3', 'mot5z2', 'mot5iqr',
               'mot10z3', 'mot10z2', 'mot10iqr',
               'mog3', 'mog5', 'mog9']

original_columns = ['time (ns)', 'frame index',
                    'trans x original', 'trans y original', 'trans z original',
                    'rot w original', 'rot x original', 'rot y original', 'rot z original',
                    'rot x deg original', 'rot y deg original', 'rot z deg originl']

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

mog3_columns = ['trans x mog3', 'trans y mog3', 'trans z mog3', 
                'rot x deg mog3', 'rot y deg mog3', 'rot z deg mog3']
mog5_columns = ['trans x mog5', 'trans y mog5', 'trans z mog5', 
                'rot x deg mog5', 'rot y deg mog5', 'rot z deg mog5']
mog9_columns = ['trans x mog9', 'trans y mog9', 'trans z mog9', 
                'rot x deg mog9', 'rot y deg mog9', 'rot z deg mog9'] 

apriltag_path= '/home/kathrin/dev_ws/csv_files/test/'
aruco_path = '/home/kathrin/dev_ws/csv_files/test/'

grid_sizes = {1 : 0,
              3 : 0, 
              5 : 1,
              9 : 2}


class EvaluationData(Node):

    def __init__(self):
        super().__init__("improvement_tests")
        global apriltag_path, aruco_path, topic_types
        
        # (parameter name, parameter value)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('aruco_file', 'aruco_data.xlsx'),
                ('apriltag_file', 'apriltag_data.xlsx'),
                ("grid_size", 1)])
        apriltag_path += str(self.get_parameter('apriltag_file').value)
        aruco_path += str(self.get_parameter('aruco').value) 
        
        self.time = 0
        
        self.sub_apriltag = self.create_subscription(
            TFMessage,
            "tf",
            self.apriltag_listener_callback,
            10)
        self.sub_apriltag
        
        # self.sub_aruco = self.create_subscription(
        #     ArucoMarkers,
        #     "aruco_markers",
        #     self.aruco_listener_callback,
        #     10)
        # self.tf_subscription
     

    def apriltag_listener_callback(self, msg):
        markers_id4 = []
        for tf in msg.transforms:
            tag_id = tf.child_frame_id
            if tag_id.endswith(':4'):
                markers_id4.append(tf)
                
        if len(markers_id4) == int(self.get_parameter('grid_size').value):
            print(f'successfully received msg with {len(markers_id4)}')
     
    # def aruco_listener_callback(self, msg):
        


         
def main(args=None):

    rclpy.init(args=args)
    
    eval_data = EvaluationData()
    rclpy.spin(eval_data)
    
    eval_data.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
