#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import os
from math import pi

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from ros2_aruco_interfaces.msg import ArucoMarkers

from tf_transformations import euler_from_quaternion, quaternion_from_euler


topic_types = ['original', 
               'mot3z3', 'mot3z2' ,'mot3iqr',
               'mot5z3', 'mot5z2', 'mot5iqr',
               'mot10z3', 'mot10z2', 'mot10iqr',
               'mog3', 'mog5', 'mog9']

# gt_columns = ['time', 'frame index',
#               'trans x original', 'trans y original', 'trans z original',
#               'rot x deg original', 'rot y deg original', 'rot z deg original']

original_columns = ['time', 'frame index',
                    'trans x original', 'trans y original', 'trans z original',
                    'rot w original', 'rot x original', 'rot y original', 'rot z original',
                    'rot x deg original', 'rot y deg original', 'rot z deg original']

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

# mog3_columns = ['trans x mog3', 'trans y mog3', 'trans z mog3', 
#                 'rot x deg mog3', 'rot y deg mog3', 'rot z deg mog3']
# mog5_columns = ['trans x mog5', 'trans y mog5', 'trans z mog5', 
#                 'rot x deg mog5', 'rot y deg mog5', 'rot z deg mog5']
# mog9_columns = ['trans x mog9', 'trans y mog9', 'trans z mog9', 
#                 'rot x deg mog9', 'rot y deg mog9', 'rot z deg mog9'] 

path_apriltag= '/home/kathrin/dev_ws/csv_files/test/'
path_aruco = '/home/kathrin/dev_ws/csv_files/test/'

duration_max = 10.0

# grid_sizes = {1 : 0,
#               3 : 0, 
#               5 : 1,
#               9 : 2}


class EvaluationData(Node):

    def __init__(self):
        super().__init__('eval_data')
        global path_apriltag, path_aruco, topic_types
        
        # (parameter name, parameter value)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('aruco_file', 'aruco_data.xlsx'),
                ('apriltag_file', 'apriltag_data.xlsx'),
                ('grid_size', 1),
                ('gt_vals', [0.0, 0.0, 30.0, 0.0, 0.0, 0.0]),
                ('record', True)])
        path_apriltag += str(self.get_parameter('apriltag_file').value)
        path_aruco += str(self.get_parameter('aruco_file').value) 
        
        gt_vals_orig = self.get_parameter('gt_vals').value
        gt_vals  = [-1, -1] + gt_vals_orig[:3]
        gt_vals += self.get_quaternion_from_deg(gt_vals_orig[3:]) + gt_vals_orig[3:]
        
        self.start_time = -1
        self.frame_index = -1
        self.all_series = list()
        
        if os.path.isfile(path_apriltag):
            self.df_apriltag = pd.read_excel(path_apriltag, sheet_name='AprilTag')
            self.df_apriltag = self.df_apriltag.iloc[: , 1:]
            self.all_series.append(pd.Series(data=gt_vals, index=original_columns))
        else:
            dictionary = dict(zip(original_columns, gt_vals))
            self.df_apriltag = pd.DataFrame(data=dictionary, index=[0])

        self.sub_apriltag = self.create_subscription(
            TFMessage,
            "tf",
            self.apriltag_listener_callback,
            10)
        self.sub_apriltag
        
        self.apriltag_mot3 = list()
        self.apriltag_mot5 = list()
        self.apriltag_mot10 = list()
        self.aruco_mot3 = list()
        self.aruco_mot5 = list()
        self.aruco_mot10 = list()
        
        
        # self.sub_aruco = self.create_subscription(
        #     ArucoMarkers,
        #     "aruco_markers",
        #     self.aruco_listener_callback,
        #     10)
        # self.tf_subscription
     

    def apriltag_listener_callback(self, msg):
        if self.get_parameter('record').value:
            #grid_size = int(self.get_parameter('grid_size').value)
            markers_id4 = self.get_markers_with_id(msg, ':4')
            
            if len(markers_id4) == 1:#grid_size:
                resulting_series = list()
                
                time_diff = self.get_time(msg.transforms[0].header.stamp)
                self.frame_index += 1
                
                trans = markers_id4[0].transform.translation
                rot = markers_id4[0].transform.rotation
                
                original_vals = self.get_data_from_msg(trans, rot, True, time_diff)
                
                resulting_series.append(pd.Series(data=original_vals, index=original_columns))
                
                # TODO add mot
                
                self.all_series.append(pd.concat(resulting_series))
                
                print(f'Frame {self.frame_index} recorded at time {time_diff}.')
                    
                if time_diff >= duration_max:
                    self.df_apriltag = pd.concat(
                        [self.df_apriltag, pd.concat(self.all_series, axis=1).T],
                        ignore_index=True,
                        sort=False)
                    
                    print(f'DF columns: {self.df_apriltag.columns}')
                    print(self.df_apriltag.head(5))
                    print(self.df_apriltag.iloc[-2:])
                    print(f'Save path: {path_apriltag}')
                    
                    self.start_time = -1
                    self.frame_index=0
                    self.set_parameters([Parameter(
                        'record',
                        Parameter.Type.BOOL,
                        False)])
                
       
     
    # def aruco_listener_callback(self, msg):
        
    def get_data_from_msg(self, trans, rot, original=False, time_diff=None):
        if original and (time_diff is None):
            print("Please define time_diff for original messages.")
            time_diff = -1
            
        rot_deg = self.get_deg_from_quaternion(rot)
        
        values = list()
        if original:
            values += [time_diff, self.frame_index]
        values += [trans.x, trans.y, trans.z]
        if original:
            values += [rot.w, rot.x, rot.y, rot.z]
        values += rot_deg
        
        return values
        
        
    def get_quaternion_from_deg(self, deg):
        (x, y, z, w) =  quaternion_from_euler(
            (deg[0]/180)*pi,
            (deg[1]/180)*pi,
            (deg[2]/180)*pi)
        return [w, x, y, z]
    
    def get_deg_from_quaternion(self, rot):
        # TODO make applicable for ArUco
        (roll, pitch, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        return [(180/pi)*roll, (180/pi)*pitch, (180/pi)*yaw]
      
    def get_markers_with_id(self, msg, id_string):
        # TODO make applicable for ArUco
        markers_id4 = []
        for tf in msg.transforms:
            tag_id = tf.child_frame_id
            if tag_id.endswith(id_string):
                markers_id4.append(tf)
        return markers_id4
      
    def get_time(self, stamp):
        nanoseconds = stamp.nanosec
        seconds = stamp.sec
        time_current = seconds + (nanoseconds * 10**(-9))
        if self.start_time < 0:
            self.start_time = time_current
        return time_current - self.start_time 
        

         
def main(args=None):

    rclpy.init(args=args)
    
    eval_data = EvaluationData()
    rclpy.spin(eval_data)
    
    eval_data.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
