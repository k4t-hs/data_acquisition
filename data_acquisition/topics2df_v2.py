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

import tf_transformations as tf2#import euler_from_quaternion, quaternion_from_euler


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

dict_motmog = {'columns_mot3z3' :
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
                    'rot x deg mot10iqr', 'rot y deg mot10iqr', 'rot z deg mot10iqr'],
               'columns_mog3z3' :
                   ['trans x mog3z3', 'trans y mog3z3', 'trans z mog3z3',
                    'rot x deg mog3z3', 'rot y deg mog3z3', 'rot z deg mog3z3'],
               'columns_mog3z2' : 
                   ['trans x mog3z2', 'trans y mog3z2', 'trans z mog3z2',
                    'rot x deg mog3z2', 'rot y deg mog3z2', 'rot z deg mog3z2'],
               'columns_mog3iqr' : 
                   ['trans x mog3iqr', 'trans y mog3iqr', 'trans z mog3iqr', 
                    'rot x deg mog3iqr', 'rot y deg mog3iqr', 'rot z deg mog3iqr'],
               'columns_mog5z3' :
                   ['trans x mog5z3', 'trans y mog5z3', 'trans z mog5z3',
                    'rot x deg mog5z3', 'rot y deg mog5z3', 'rot z deg mog5z3'],
               'columns_mog5z2' : 
                   ['trans x mog5z2', 'trans y mog5z2', 'trans z mog5z2',
                    'rot x deg mog5z2', 'rot y deg mog5z2', 'rot z deg mog5z2'],
               'columns_mog5iqr' :
                   ['trans x mog5iqr', 'trans y mog5iqr', 'trans z mog5iqr',
                    'rot x deg mog5iqr', 'rot y deg mog5iqr', 'rot z deg mog5iqr'],
               'columns_mog9z3' :
                   ['trans x mog9z3', 'trans y mog9z3', 'trans z mog9z3',
                    'rot x deg mog9z3', 'rot y deg mog9z3', 'rot z deg mog9z3'],
               'columns_mog9z2' :
                   ['trans x mog9z2', 'trans y mog9z2', 'trans z mog9z2',
                    'rot x deg mog9z2', 'rot y deg mog9z2', 'rot z deg mog9z2'],
               'columns_mog9iqr' : 
                   ['trans x mog9iqr', 'trans y mog9iqr', 'trans z mog9iqr', 
                    'rot x deg mog9iqr', 'rot y deg mog9iqr', 'rot z deg mog9iqr']}


path_apriltag = '/home/kathrin/dev_ws/csv_files/new-method/'
path_aruco= '/home/kathrin/dev_ws/csv_files/new-method/'

duration_max = 10.0

grid_size = 1


class EvaluationData(Node):

    def __init__(self):
        super().__init__('eval_data')
        global path_apriltag, path_aruco, topic_types, grid_size
        
        # (parameter name, parameter value)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('name_apriltag', 'data_apriltag.xlsx'),
                ('name_aruco', 'data_aruco.xlsx'),
                ('grid_size', 1),
                ('vals_gt', [0.0, 0.0, 30.0, 0.0, 0.0, 0.0]),
                ('record_apriltag', True),
                ('record_aruco', True)])
        path_apriltag += str(self.get_parameter('name_apriltag').value)
        path_aruco += str(self.get_parameter('name_aruco').value) 
        
        self.start_time_apriltag = -1
        self.frame_idx_apriltag = -1
        self.series_all_apriltag = list()
        
        self.start_time_aruco = -1
        self.frame_idx_aruco = -1
        self.series_all_aruco = list()
        
        self.df_apriltag = None
        self.df_aruco = None
        
        self.sub_apriltag = self.create_subscription(
            TFMessage,
            "tf",
            self.apriltag_listener_callback,
            10)
        self.sub_apriltag
        
        self.mot3_apriltag = list()
        self.mot5_apriltag = list()
        self.mot10_apriltag = list()
     
        self.sub_aruco = self.create_subscription(
            ArucoMarkers,
            "aruco_markers",
            self.aruco_listener_callback,
            10)
        self.sub_aruco
        
        self.mot3_aruco = list()
        self.mot5_aruco = list()
        self.mot10_aruco = list()
     

    def apriltag_listener_callback(self, msg):
        global grid_size
        columns_standard = columns_orig        
        if self.get_parameter('record_apriltag').value:
            
            if self.frame_idx_apriltag < 0:
                vals_gt_orig = list(self.get_parameter('vals_gt').value)
                # if type(vals_gt_orig) is not list:
                #     vals_gt_orig = vals_gt_orig.tolist()
                vals_gt  = [-1, -1] + vals_gt_orig[:3]
                vals_gt += self.get_quaternion_from_deg(vals_gt_orig[3:]) + vals_gt_orig[3:]
                grid_size = int(self.get_parameter('grid_size').value)
                if grid_size == 1:
                    if os.path.isfile(path_apriltag):
                        self.df_apriltag = pd.read_excel(path_apriltag, sheet_name='AprilTag')
                        self.df_apriltag = self.df_apriltag.iloc[:, 1:]
                        self.series_all_apriltag.append(pd.Series(data=vals_gt, index=columns_standard))
                    else:
                        dictionary = dict(zip(columns_standard, vals_gt))
                        self.df_apriltag = pd.DataFrame(data=dictionary, index=[0])
                        # self.frame_idx_apriltag=0
                else:
                    filename = path_apriltag.replace('.xlsx', f'_mog{grid_size}.xlsx')
                    if os.path.isfile(filename):
                        self.df_apriltag = pd.read_excel(filename, sheet_name='AprilTag')
                        self.df_apriltag = self.df_apriltag.iloc[:, 1:]
                        self.series_all_apriltag.append(pd.Series(data=vals_gt, index=columns_standard))
                    else:
                        dictionary = dict(zip(columns_standard, vals_gt))
                        self.df_apriltag = pd.DataFrame(data=dictionary, index=[0])
                        # self.frame_idx_apriltag=0
                        
                self.frame_idx_apriltag += 1
                
            markers_id4 = self.get_markers_with_id(msg, ':4')
            
            if len(markers_id4) == grid_size:#1:#grid_size:
                series_resulting = list()
                
                for idx, marker in enumerate(markers_id4):                    
                    time_diff = self.get_time(marker.header.stamp)#msg.transforms[0].header.stamp)
                    
                    trans = marker.transform.translation
                    rot = marker.transform.rotation
                    
                    vals_orig = self.get_data_from_msg(trans, rot, self.frame_idx_apriltag, True, time_diff)
                    
                    if grid_size == 1:
                        series_orig = pd.Series(data=vals_orig, index=columns_standard)
                    else:
                        series_orig = pd.Series(data=vals_orig, 
                                                index=[s+' '+str(idx) for s in columns_standard])
                     
                    series_resulting.append(series_orig)
                    
                if grid_size == 1:
                    self.mot3_apriltag.append(series_orig)
                    self.mot5_apriltag.append(series_orig)
                    self.mot10_apriltag.append(series_orig)
                    
                    if len(self.mot3_apriltag) == 3:
                        means = self.get_means(self.mot3_apriltag)
                        series_resulting += means
                        self.mot3_apriltag.clear()
                        
                    if len(self.mot5_apriltag) == 5:
                        means = self.get_means(self.mot5_apriltag)
                        series_resulting += means
                        self.mot5_apriltag.clear()
                        
                    if len(self.mot10_apriltag) == 10:
                        means = self.get_means(self.mot10_apriltag)
                        series_resulting += means
                        self.mot10_apriltag.clear()
                else:
                    series_resulting += self.get_means(series_resulting, 'mog')
                
                self.series_all_apriltag.append(pd.concat(series_resulting, sort=False))
                self.frame_idx_apriltag += 1
                
                print(f'AprilTag Marker with index {self.frame_idx_apriltag} recorded at time {time_diff}.')
                
                if time_diff >= duration_max:
                    self.df_apriltag = pd.concat(
                        [self.df_apriltag, 
                         pd.concat(self.series_all_apriltag, axis=1, sort=False).T],
                        ignore_index=True,
                        sort=False)
                    
                    if grid_size == 1:
                        self.df_apriltag.to_excel(path_apriltag, sheet_name='AprilTag')
                        print("--------------------------------------------------",
                              "--------------------------------------------------")
                        print(f'Saved AprilTag data in: {path_apriltag}')
                        print("--------------------------------------------------",
                              "--------------------------------------------------")
                    else:
                        self.df_apriltag.to_excel(
                            path_apriltag.replace('.xlsx', f'_mog{grid_size}.xlsx'),
                            sheet_name='AprilTag')
                        print("--------------------------------------------------",
                              "--------------------------------------------------")
                        print(f'Saved AprilTag data in: {path_apriltag.replace(".xlsx", f"_mog{grid_size}.xlsx")}')
                        print("--------------------------------------------------",
                              "--------------------------------------------------")
                    
                    self.start_time_apriltag = -1
                    self.frame_idx_apriltag = -1
                    self.mot3_apriltag.clear()
                    self.mot5_apriltag.clear()
                    self.mot10_apriltag.clear()
                    self.series_all_apriltag.clear()

                    self.set_parameters([Parameter(
                        'record_apriltag',
                        Parameter.Type.BOOL,
                        False)])
                    
    def aruco_listener_callback(self, msg):
        global grid_size
        columns_standard = columns_orig
        if self.get_parameter('record_aruco').value:
            
            if self.frame_idx_aruco < 0:
                vals_gt_orig = list(self.get_parameter('vals_gt').value)
                vals_gt  = [-1, -1] + vals_gt_orig[:3]
                vals_gt += self.get_quaternion_from_deg(vals_gt_orig[3:]) + vals_gt_orig[3:]
                grid_size = int(self.get_parameter('grid_size').value)
                if grid_size == 1:
                    if os.path.isfile(path_aruco):
                        self.df_aruco = pd.read_excel(path_aruco, sheet_name='ArUco')
                        self.df_aruco = self.df_aruco.iloc[:, 1:]
                        self.series_all_aruco.append(pd.Series(data=vals_gt, index=columns_standard))
                    else:
                        dictionary = dict(zip(columns_standard, vals_gt))
                        self.df_aruco= pd.DataFrame(data=dictionary, index=[0])
                        # self.frame_idx_aruco=0
                else:
                    filename = path_aruco.replace('.xlsx', f'_mog{grid_size}.xlsx')
                    if os.path.isfile(filename):
                        self.df_aruco = pd.read_excel(filename, sheet_name='ArUco')
                        self.df_aruco = self.df_aruco.iloc[:, 1:]
                        self.series_all_aruco.append(pd.Series(data=vals_gt, index=columns_standard))
                    else:
                        dictionary = dict(zip(columns_standard, vals_gt))
                        self.df_aruco = pd.DataFrame(data=dictionary, index=[0])
                        # self.frame_idx_aruco=0
                        
                self.frame_idx_aruco += 1
                
            # TODO: add id as parameter
            markers_id4 = self.get_markers_with_id(msg, 4)
            
            if len(markers_id4) == grid_size:#1:#grid_size:
                series_resulting = list()
                
                for idx, marker in enumerate(markers_id4): 
                    time_diff = self.get_time(msg.header.stamp, False)
                    
                    # TODO adapt to other coordinate system (aruco <--> apriltag)
                    pos = marker.position
                    orient = marker.orientation
                    
                    vals_orig = self.get_data_from_msg(pos, orient, self.frame_idx_aruco, True, time_diff)
                    # print(f'Angles original:\n\t{vals_orig[-3:]}')
                    
                    # # Tz = tf2.rotation_matrix(pi, (0, 0, 1))
                    # # print(f'Rotation matix before:\n\t{Tz[:3,:3]}')
                    # # Rz = np.array(Tz[:3,:3]).transpose()
                    # # # Rz = Rz.transpose()
                    # # Tz[:3,:3] = Rz
                    # Tz = np.array([[1,0,0,0],
                    #                [0,-1,0,0],
                    #                [0,0,-1,0],
                    #                [0,0,0,1]])
                    # # Rz = Tz[:3,:3].transpose()
                    # # Tz[:3,:3] = Rz
                    # # print(f'Rotation matix:\n\t{Rz}')
                    # print(f'Rotation matix:\n\t{Tz}')
                    # print(f'Orientation original:\n\t{orient}')
                    # # print(f'Position original:\n\t{pos}')
                    # # (pos.x, pos.y, pos.z) = np.matmul(Rz, np.array([pos.x, pos.y, pos.z]))
                    # # print(f'Transformed position:\n\t{pos}')
                    # (orient.x, orient.y, orient.z, orient.w) = np.matmul(Tz, np.array([orient.x, orient.y, orient.z, orient.w]))
                    # print(f'Transformed orientation:\n\t{orient}')
                    
                    # vals_orig = self.get_data_from_msg(pos, orient, self.frame_idx_aruco, True, time_diff)
                    # print(f'Transformed angles:\n\t{vals_orig[-3:]}')
                    
                    if grid_size == 1:
                        series_orig = pd.Series(data=vals_orig, index=columns_standard)
                    else:
                        series_orig = pd.Series(data=vals_orig, 
                                                index=[s+' '+str(idx) for s in columns_standard])
                    
                    series_resulting.append(series_orig)
                
                if grid_size ==1:
                    self.mot3_aruco.append(series_orig)
                    self.mot5_aruco.append(series_orig)
                    self.mot10_aruco.append(series_orig)
                    
                    if len(self.mot3_aruco) == 3:
                        means = self.get_means(self.mot3_aruco)
                        series_resulting += means
                        self.mot3_aruco.clear()
                        
                    if len(self.mot5_aruco) == 5:
                        means = self.get_means(self.mot5_aruco)
                        series_resulting += means
                        self.mot5_aruco.clear()
                        
                    if len(self.mot10_aruco) == 10:
                        means = self.get_means(self.mot10_aruco)
                        series_resulting += means
                        self.mot10_aruco.clear()
                else:
                    series_resulting += self.get_means(series_resulting, 'mog')
                
                self.series_all_aruco.append(pd.concat(series_resulting, sort=False))
                self.frame_idx_aruco += 1
                
                print(f'ArUco Marker with index {self.frame_idx_aruco} recorded at time {time_diff}.')
                    
                if time_diff >= duration_max:
                    self.df_aruco = pd.concat(
                        [self.df_aruco, 
                          pd.concat(self.series_all_aruco, axis=1, sort=False).T],
                        ignore_index=True,
                        sort=False)
                    
                    if grid_size == 1:
                        self.df_aruco.to_excel(path_aruco, sheet_name='ArUco')
                        print("--------------------------------------------------",
                              "--------------------------------------------------")
                        print(f'Saved ArUco data in: {path_aruco}')
                        print("--------------------------------------------------",
                              "--------------------------------------------------")
                    else:
                        self.df_aruco.to_excel(
                            path_aruco.replace('.xlsx', f'_mog{grid_size}.xlsx'),
                            sheet_name='ArUco')
                        print("--------------------------------------------------",
                              "--------------------------------------------------")
                        print(f'Saved ArUco data in: {path_aruco.replace(".xlsx", f"_mog{grid_size}.xlsx")}')
                        print("--------------------------------------------------",
                              "--------------------------------------------------")
                        
                    self.start_time_aruco = -1
                    self.frame_idx_aruco = -1
                    self.mot3_aruco.clear()
                    self.mot5_aruco.clear()
                    self.mot10_aruco.clear()
                    self.series_all_aruco.clear()

                    self.set_parameters([Parameter(
                        'record_aruco',
                        Parameter.Type.BOOL,
                        False)])
           
    
        
    def set_start_df(self, vals_gt):        
        grid_size = self.get_parameter('grid_size').value 
        
        if grid_size == 1:
            if os.path.isfile(path_apriltag):
                self.df_apriltag = pd.read_excel(path_apriltag, sheet_name='AprilTag')
                self.df_apriltag = self.df_apriltag.iloc[:, 1:]
            else:
                dictionary = dict(zip(columns_orig, vals_gt))
                self.df_apriltag = pd.DataFrame(data=dictionary, index=[0])
                self.frame_idx_apriltag=0
                
            if os.path.isfile(path_aruco):
                self.df_aruco = pd.read_excel(path_aruco, sheet_name='ArUco')
                self.df_aruco = self.df_aruco.iloc[:, 1:]
            else:
                dictionary = dict(zip(columns_orig, vals_gt))
                self.df_aruco = pd.DataFrame(data=dictionary, index=[0])
                self.frame_idx_aruco=0
        else:
            filename = path_apriltag.replace('.xlsx', f'_mog{grid_size}.xlsx')
            if os.path.isfile(filename):
                self.df_apriltag = pd.read_excel(filename, sheet_name='AprilTag')
                self.df_apriltag = self.df_apriltag.iloc[:, 1:]
            else:
                dictionary = dict(zip(columns_orig, vals_gt))
                self.df_apriltag = pd.DataFrame(data=dictionary, index=[0])
                self.frame_idx_apriltag=0
              
            filename = path_aruco.replace('.xlsx', f'_mog{grid_size}.xlsx')
            if os.path.isfile(filename):
                self.df_aruco = pd.read_excel(filename, sheet_name='ArUco')
                self.df_aruco = self.df_aruco.iloc[:, 1:]
            else:
                dictionary = dict(zip(columns_orig, vals_gt))
                self.df_aruco = pd.DataFrame(data=dictionary, index=[0])
                self.frame_idx_aruco=0
                
        
    def get_means(self, val_list, method_type='mot'): 
        num_frames = len(val_list)
        series_resulting = list()
        
        if method_type == 'mot':
            df = pd.concat(val_list, axis=1, sort=False).T
            columns_all = df.columns
        else:
            df = pd.concat(val_list, sort=False).T
            columns_all = df.index
        columns_wanted = [name for name in columns_all 
                          if (('original' in name) 
                          and (('trans' in name) 
                               or ('deg' in name)))]
        df = df[columns_wanted]
        
        if method_type == 'mog':
            columns_new = columns_orig[2:5]+columns_orig[-3:]
            vals = list()
            for x in range(grid_size):
                v = list()
                for y in (columns_new):
                    v.append(df[y + ' ' + str(x)])
                vals.append(v)
                
            df = pd.DataFrame(data=vals, columns=columns_new)
        
        mean_z3 = self.get_mean_z(df, max_val=3)
        new_index = dict(zip(df.columns, dict_motmog[f'columns_{method_type}{num_frames}z3']))
        series_resulting.append(mean_z3.rename(new_index))
        
        mean_z2 = self.get_mean_z(df, max_val=2)
        new_index = dict(zip(df.columns, dict_motmog[f'columns_{method_type}{num_frames}z2']))
        series_resulting.append(mean_z2.rename(new_index))
        
        mean_iqr = self.get_mean_iqr(df)
        new_index = dict(zip(df.columns, dict_motmog[f'columns_{method_type}{num_frames}iqr']))
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
            print('warning warning warning warning warning warning warning')
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
        
        
    def get_data_from_msg(self, trans, rot, idx, original=False, time_diff=None):
        if original and (time_diff is None):
            print("Please define time_diff for original messages.")
            time_diff = -1
            
        rot_deg = self.get_deg_from_quaternion(rot)
        
        values = list()
        if original:
            values += [time_diff, idx]
        values += [trans.x, trans.y, trans.z]
        if original:
            values += [rot.w, rot.x, rot.y, rot.z]
        values += rot_deg
        
        return values
        
        
    def get_quaternion_from_deg(self, deg):
        (x, y, z, w) =  tf2.quaternion_from_euler(
            (deg[0]/180)*pi,
            (deg[1]/180)*pi,
            (deg[2]/180)*pi)
        return [w, x, y, z]
    
    
    def get_deg_from_quaternion(self, rot):
        (roll, pitch, yaw) = tf2.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        deg_list = self.get_deg([roll, pitch,yaw])
        return deg_list
    
    def get_deg(self, rad_list):
        deg_list = list()
        for x in rad_list:
            deg_list.append((180/pi)*x)
        return deg_list
            
      
        
    def get_markers_with_id(self, msg, id_val):
        markers_id4 = []
        
        if type(msg) == TFMessage:
            for transf in msg.transforms:
                tag_id = transf.child_frame_id
                if tag_id.endswith(id_val):
                    markers_id4.append(transf)
        elif type(msg) == ArucoMarkers:        
            for i, pose in enumerate(msg.poses):
                tag_id = msg.marker_ids[i]
                if tag_id == id_val:
                    markers_id4.append(pose)        
                
        return markers_id4
      
        
    def get_time(self, stamp, is_apriltag=True):
        nanoseconds = stamp.nanosec
        seconds = stamp.sec
        time_current = seconds + (nanoseconds * 10**(-9))
        if is_apriltag:
            if self.start_time_apriltag < 0:
                self.start_time_apriltag = time_current
            return time_current - self.start_time_apriltag 
        else:
            if self.start_time_aruco < 0:
                self.start_time_aruco = time_current
            return time_current - self.start_time_aruco
        

         
def main(args=None):

    rclpy.init(args=args)
    
    eval_data = EvaluationData()
    rclpy.spin(eval_data)
    
    eval_data.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
