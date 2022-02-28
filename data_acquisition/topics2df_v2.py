#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import os
from math import pi
import warnings
import numpy as np
import scipy.stats as stats

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

columns_orig = ['time', 'frame index',
                    'trans x original', 'trans y original', 'trans z original',
                    'rot w original', 'rot x original', 'rot y original', 'rot z original',
                    'rot x deg original', 'rot y deg original', 'rot z deg original']

dict_mot = {'columns_mot3z3' :
                ['trans x mot3z3', 'trans y mot3z3', 'trans z mot3z3',
                 'rot x deg mot3z3', 'rot y deg mot3z3', 'rot z deg mot3z3'],
            'columns_mot3z2' : 
                ['trans x mot3z2', 'trans y mot3z2', 'trans z mot3z2',
                 'rot x deg mot3z2', 'rot y deg mot3z2', 'rot z deg mot3z2'],
            'columns_mot3iqr' : 
                ['trans x mot3iqr', 'trans y mot3iqr', 'trans z mot3iqr', 
                 'rot x deg mot3iqr', 'rot y deg mot3iqr', 'rot z deg mot3iqr'],
            'columns_mot5z3' :
                ['trans x mot5z3', 'trans y mot5z3', 'trans z mot5z3',
                 'rot x deg mot5z3', 'rot y deg mot5z3', 'rot z deg mot5z3'],
            'columns_mot5z2' : 
                ['trans x mot5z2', 'trans y mot5z2', 'trans z mot5z2',
                 'rot x deg mot5z2', 'rot y deg mot5z2', 'rot z deg mot5z2'],
            'columns_mot5iqr' :
                ['trans x mot5iqr', 'trans y mot5iqr', 'trans z mot5iqr',
                 'rot x deg mot5iqr', 'rot y deg mot5iqr', 'rot z deg mot5iqr'],
            'columns_mot10z3' :
                ['trans x mot10z3', 'trans y mot10z3', 'trans z mot10z3',
                 'rot x deg mot10z3', 'rot y deg mot10z3', 'rot z deg mot10z3'],
            'columns_mot10z2' :
                ['trans x mot10z2', 'trans y mot10z2', 'trans z mot10z2',
                 'rot x deg mot10z2', 'rot y deg mot10z2', 'rot z deg mot10z2'],
            'columns_mot10iqr' : 
                ['trans x mot10iqr', 'trans y mot10iqr', 'trans z mot10iqr', 
                 'rot x deg mot10iqr', 'rot y deg mot10iqr', 'rot z deg mot10iqr']}

# mog3_columns = ['trans x mog3', 'trans y mog3', 'trans z mog3', 
#                 'rot x deg mog3', 'rot y deg mog3', 'rot z deg mog3']
# mog5_columns = ['trans x mog5', 'trans y mog5', 'trans z mog5', 
#                 'rot x deg mog5', 'rot y deg mog5', 'rot z deg mog5']
# mog9_columns = ['trans x mog9', 'trans y mog9', 'trans z mog9', 
#                 'rot x deg mog9', 'rot y deg mog9', 'rot z deg mog9'] 

path_excel= '/home/kathrin/dev_ws/csv_files/test/'

duration_max = 10.0

# grid_sizes = {1 : 0,
#               3 : 0, 
#               5 : 1,
#               9 : 2}


class EvaluationData(Node):

    def __init__(self):
        super().__init__('eval_data')
        global path_excel, topic_types
        
        # (parameter name, parameter value)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('path_excel', 'eval_data.xlsx'),
                ('grid_size', 1),
                ('vals_gt', [0.0, 0.0, 30.0, 0.0, 0.0, 0.0]),
                ('record', True)])
        path_excel += str(self.get_parameter('path_excel').value)
        
        vals_gt_orig = self.get_parameter('vals_gt').value
        vals_gt  = [-1, -1] + vals_gt_orig[:3]
        vals_gt += self.get_quaternion_from_deg(vals_gt_orig[3:]) + vals_gt_orig[3:]
        
        self.start_time = -1
        self.frame_index = -1
        self.series_all = list()
        
        if os.path.isfile(path_excel):
            # try:
                self.df_apriltag = pd.read_excel(path_excel, sheet_name='AprilTag')
                self.df_apriltag = self.df_apriltag.iloc[:, 1:]
            # except:
            #     # do same as else
            #     print("test")
            # try:
            #     self.df_aruco = pd.read_excel(path_excel, sheet_name='ArUco')
            #     self.df_aruco = self.df_aruco.iloc[:, 1:]
            # except:
            #     # do same as else
            #     print("test")

        else:
            dictionary = dict(zip(columns_orig, vals_gt))
            self.df_apriltag = pd.DataFrame(data=dictionary, index=[0])
            self.frame_index=0

        self.sub_apriltag = self.create_subscription(
            TFMessage,
            "tf",
            self.apriltag_listener_callback,
            10)
        self.sub_apriltag
        
        self.apriltag_mot3 = list()
        self.apriltag_mot5 = list()
        self.apriltag_mot10 = list()
     
        # self.sub_aruco = self.create_subscription(
        #     ArucoMarkers,
        #     "aruco_markers",
        #     self.aruco_listener_callback,
        #     10)
        # self.tf_subscription
        
        # self.aruco_mot3 = list()
        # self.aruco_mot5 = list()
        # self.aruco_mot10 = list()
     

    def apriltag_listener_callback(self, msg):
        if self.get_parameter('record').value:
            
            if self.frame_index < 0:
                vals_gt_orig = list(self.get_parameter('vals_gt').value)
                print(vals_gt_orig)
                vals_gt  = [-1, -1] + vals_gt_orig[:3]
                vals_gt += self.get_quaternion_from_deg(vals_gt_orig[3:]) + vals_gt_orig[3:]
                self.series_all.append(pd.Series(data=vals_gt, index=columns_orig))
                self.frame_index += 1
                
            #grid_size = int(self.get_parameter('grid_size').value)
            markers_id4 = self.get_markers_with_id(msg, ':4')
            
            if len(markers_id4) == 1:#grid_size:
                series_resulting = list()
                
                time_diff = self.get_time(msg.transforms[0].header.stamp)
                
                trans = markers_id4[0].transform.translation
                rot = markers_id4[0].transform.rotation
                
                vals_orig = self.get_data_from_msg(trans, rot, True, time_diff)
                
                series_orig = pd.Series(data=vals_orig, index=columns_orig)
                
                series_resulting.append(series_orig)
                
                # TODO add mot
                self.apriltag_mot3.append(series_orig)
                self.apriltag_mot5.append(series_orig)
                self.apriltag_mot10.append(series_orig)
                
                if len(self.apriltag_mot3) == 3:
                    # create mot series
                    print("mot3")
                    means = self.get_means(self.apriltag_mot3)
                    series_resulting += means
                    self.apriltag_mot3.clear()
                    
                if len(self.apriltag_mot5) == 5:
                    # create mot series
                    print("mot5")
                    means = self.get_means(self.apriltag_mot5)
                    series_resulting += means
                    self.apriltag_mot5.clear()
                    
                if len(self.apriltag_mot10) == 10:
                    # create mot series
                    print("mot10")
                    means = self.get_means(self.apriltag_mot10)
                    series_resulting += means
                    self.apriltag_mot10.clear()
                
                self.series_all.append(pd.concat(series_resulting, sort=False))
                self.frame_index += 1
                
                print(f'Frame {self.frame_index} recorded at time {time_diff}.')
                # print(f'Resulting series: {pd.concat(series_resulting, sort=False)}')
                    
                if time_diff >= duration_max:
                    self.df_apriltag = pd.concat(
                        [self.df_apriltag, 
                         pd.concat(self.series_all, axis=1, sort=False).T],
                        ignore_index=True,
                        sort=False)
                    
                    print("--------------------------------------------------",
                          "--------------------------------------------------")
                    print(f'DF columns: {self.df_apriltag.columns}')
                    print("--------------------------------------------------",
                          "--------------------------------------------------")
                    # print(self.df_apriltag[dict_mot['columns_mot3z3'][0]])
                    print(self.df_apriltag.head(15))
                    # print(self.df_apriltag.iloc[-2:])
                    print("--------------------------------------------------",
                          "--------------------------------------------------")
                    print(f'Save path: {path_excel}')
                    print("--------------------------------------------------",
                          "--------------------------------------------------")
                    
                    self.df_apriltag.to_excel(path_excel, sheet_name='AprilTag')
                    
                    self.start_time = -1
                    self.frame_index = -1
                    self.apriltag_mot3.clear()
                    self.apriltag_mot5.clear()
                    self.apriltag_mot10.clear()

                    self.set_parameters([Parameter(
                        'record',
                        Parameter.Type.BOOL,
                        False)])
                    
    def aruco_listener_callback(self, msg):
        if self.get_parameter('record').value:
            
            if self.frame_index < 0:
                vals_gt_orig = list(self.get_parameter('vals_gt').value)
                print(vals_gt_orig)
                vals_gt  = [-1, -1] + vals_gt_orig[:3]
                vals_gt += self.get_quaternion_from_deg(vals_gt_orig[3:]) + vals_gt_orig[3:]
                self.series_all.append(pd.Series(data=vals_gt, index=columns_orig))
                self.frame_index += 1
                
            #grid_size = int(self.get_parameter('grid_size').value)
            markers_id4 = self.get_markers_with_id(msg, ':4')
            
            if len(markers_id4) == 1:#grid_size:
                series_resulting = list()
                
                time_diff = self.get_time(msg.transforms[0].header.stamp)
                
                trans = markers_id4[0].transform.translation
                rot = markers_id4[0].transform.rotation
                
                vals_orig = self.get_data_from_msg(trans, rot, True, time_diff)
                
                series_orig = pd.Series(data=vals_orig, index=columns_orig)
                
                series_resulting.append(series_orig)
                
                # TODO add mot
                self.apriltag_mot3.append(series_orig)
                self.apriltag_mot5.append(series_orig)
                self.apriltag_mot10.append(series_orig)
                
                if len(self.apriltag_mot3) == 3:
                    # create mot series
                    print("mot3")
                    means = self.get_means(self.apriltag_mot3)
                    series_resulting += means
                    self.apriltag_mot3.clear()
                    
                if len(self.apriltag_mot5) == 5:
                    # create mot series
                    print("mot5")
                    means = self.get_means(self.apriltag_mot5)
                    series_resulting += means
                    self.apriltag_mot5.clear()
                    
                if len(self.apriltag_mot10) == 10:
                    # create mot series
                    print("mot10")
                    means = self.get_means(self.apriltag_mot10)
                    series_resulting += means
                    self.apriltag_mot10.clear()
                
                self.series_all.append(pd.concat(series_resulting, sort=False))
                self.frame_index += 1
                
                print(f'Frame {self.frame_index} recorded at time {time_diff}.')
                # print(f'Resulting series: {pd.concat(series_resulting, sort=False)}')
                    
                if time_diff >= duration_max:
                    self.df_apriltag = pd.concat(
                        [self.df_apriltag, 
                         pd.concat(self.series_all, axis=1, sort=False).T],
                        ignore_index=True,
                        sort=False)
                    
                    print("--------------------------------------------------",
                          "--------------------------------------------------")
                    print(f'DF columns: {self.df_apriltag.columns}')
                    print("--------------------------------------------------",
                          "--------------------------------------------------")
                    # print(self.df_apriltag[dict_mot['columns_mot3z3'][0]])
                    print(self.df_apriltag.head(15))
                    # print(self.df_apriltag.iloc[-2:])
                    print("--------------------------------------------------",
                          "--------------------------------------------------")
                    print(f'Save path: {path_excel}')
                    print("--------------------------------------------------",
                          "--------------------------------------------------")
                    
                    self.df_apriltag.to_excel(path_excel, sheet_name='ArUco')
                    
                    self.start_time = -1
                    self.frame_index = -1
                    self.apriltag_mot3.clear()
                    self.apriltag_mot5.clear()
                    self.apriltag_mot10.clear()

                    self.set_parameters([Parameter(
                        'record',
                        Parameter.Type.BOOL,
                        False)])
           
     
    # def aruco_listener_callback(self, msg):
        
    def get_means(self, mot_list): 
        num_frames = len(mot_list)
        series_resulting = list()
        
        df = pd.concat(mot_list, axis=1, sort=False).T
        df = pd.concat([df[columns_orig[2:5]], df[columns_orig[-3:]]], 
                       axis=1, 
                       sort=False)
        
        mean_z3 = self.get_mean_z(df, max_val=3)
        new_index = dict(zip(df.columns, dict_mot[f'columns_mot{num_frames}z3']))
        series_resulting.append(mean_z3.rename(new_index))
        
        mean_z2 = self.get_mean_z(df, max_val=2)
        new_index = dict(zip(df.columns, dict_mot[f'columns_mot{num_frames}z2']))
        series_resulting.append(mean_z2.rename(new_index))
        
        mean_iqr = self.get_mean_iqr(df)
        new_index = dict(zip(df.columns, dict_mot[f'columns_mot{num_frames}iqr']))
        series_resulting.append(mean_iqr.rename(new_index))
        
        return series_resulting
        
        
        
    def get_mean_z(self, df, max_val=3):
        warnings.filterwarnings('error')
        
        try:
            check_df = df.loc[:, (df.std()!=0)]
            if not check_df.empty:
                z_scores = np.abs(stats.zscore(check_df))
                df = df[(z_scores<max_val).all(axis=1)]
        except Warning:
            print("--------------------------------------------")
            print(df)
            print(check_df)
            print(not check_df.empty)
            print(max_val)
            
        return df.mean()
        
    
    
    def get_mean_iqr(self, df):
        q1 = df.quantile(q=.25)
        q3 = df.quantile(q=.75)
        iqr = df.apply(stats.iqr)
        
        lower_range = q1-(1.5*iqr)
        upper_range = q3 + (1.5*iqr)
        
        df = df[~((df<lower_range) | (df>upper_range)).any(axis=1)]
        
        return df.mean()
        
        
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
