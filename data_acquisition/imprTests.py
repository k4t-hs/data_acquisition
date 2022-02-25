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




columns = ["translation x", "translation y", "translation z",
           "rotation x", "rotation y", "rotation z", "rotation w"]

class Improvement_Tests(Node):

    def __init__(self):
        super().__init__("improvement_tests")
        
        self.tf_subscription = self.create_subscription(
            TFMessage,
            "tf",
            self.tf_listener_callback,
            10)
        self.tf_subscription
        
        self.pose_subscription = self.create_subscription(
            ArucoMarkers,
            "aruco_markers",
            self.pose_listener_callback,
            10)
        self.tf_subscription
        
        # ---------------------------------------------------------------------
        
        self.tf_mot_publishers = []
        for num in [3,5,10]:
            pubs = []
            for std in ["zscore2", "zscore3", "iqr"]:# z score / interquartile range # 75,60,90]:
                pub = self.create_publisher(TFMessage, "tf_mot_"+str(num) + "_" + str(std), 10)
                pubs.append(pub)
            self.tf_mot_publishers.append(pubs)
        self.three_tfs = pd.DataFrame(columns=columns)
        self.five_tfs = pd.DataFrame(columns=columns)
        self.ten_tfs = pd.DataFrame(columns=columns)  
        
        self.tf_mmm_publisher = self.create_publisher(TFMessage, "tf_mmm", 10)
        
        # 3, 5 or 9
        self.declare_parameter("grid_size", 3.0)
        
        self.pose_mot_publishers = []
        for num in [3,5,10]:
            pubs = []
            for std in ["zscore2", "zscore3", "iqr"]:# z score / interquartile range # 75,60,90]:
                pub = self.create_publisher(ArucoMarkers, "pose_mot_"+str(num) + "_" + str(std), 10)
                pubs.append(pub)
            self.pose_mot_publishers.append(pubs)
        self.three_poses = pd.DataFrame(columns=columns)
        self.five_poses = pd.DataFrame(columns=columns)
        self.ten_poses = pd.DataFrame(columns=columns)  
        
        self.pose_mmm_publisher = self.create_publisher(ArucoMarkers, "pose_mmm", 10)
        

    def tf_listener_callback(self, msg):
        self.tf_publish_mot(msg)
        self.tf_publish_mmm(msg)
     
    def pose_listener_callback(self, msg):
        self.pose_publish_mot(msg)
        self.pose_publish_mmm(msg)
                
    def tf_publish_mmm(self, msg):
        tfs_id4 = []
        df_id4 = pd.DataFrame(columns=columns)
        for tf in msg.transforms:
            tag_id = tf.child_frame_id
            if tag_id.endswith(':4'):
                tfs_id4.append(tf)
                trans = tf.transform.translation
                rot = tf.transform.rotation
                row = pd.Series([trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w],
                                index=columns)
                df_id4 = df_id4.append(row, ignore_index=True)
         
        grid_size = self.get_parameter('grid_size').get_parameter_value().double_value  
        if len(tfs_id4) == grid_size:
            means = df_id4.mean()
            new_msg = self.create_new_mean_msg(msg, tfs_id4, means)
            self.tf_mmm_publisher.publish(new_msg)
            
    def pose_publish_mmm(self, msg):
        poses_id4 = []
        df_id4 = pd.DataFrame(columns=columns)
        for i, pose in enumerate(msg.poses):
            tag_id = msg.marker_ids[i]
            if tag_id == 4:
                poses_id4.append(pose)
                pos = pose.position
                orient = pose.orientation
                row = pd.Series([pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w],
                                index=columns)
                df_id4 = df_id4.append(row, ignore_index=True)
         
        grid_size = self.get_parameter('grid_size').get_parameter_value().double_value  
        if len(poses_id4) == grid_size:
            means = df_id4.mean()
            new_msg = self.create_new_mean_msg(msg, poses_id4, means)
            self.tf_mmm_publisher.publish(new_msg)            
            
        
                
       
    def tf_publish_mot(self, msg):
        tfs_id4 = []
        for tf in msg.transforms:
            tag_id = tf.child_frame_id
            if tag_id.endswith(':4'):
                tfs_id4.append(tf)
        
        if len(tfs_id4) == 1:
            trans = tfs_id4[0].transform.translation
            rot = tfs_id4[0].transform.rotation
            row = pd.Series([trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w],
                            index=columns)
            
            if len(self.three_tfs.index) < 3:
                self.three_tfs = self.three_tfs.append(row, ignore_index=True)
            else: 
                means = self.clean_mean_z_score(self.three_tfs, max_val=2)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[0][0].publish(new_msg) # 3 zscore2
                
                means = self.clean_mean_z_score(self.three_tfs)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[0][1].publish(new_msg) # 3 zscore3
                
                means = self.clean_mean_iqr(self.three_tfs)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[0][2].publish(new_msg) # 3 iqr
                
                self.three_tfs = self.three_tfs.iloc[0:0]
                
            # -----------------------------------------------------------------    
                
            if len(self.five_tfs.index) < 5:
                self.five_tfs = self.five_tfs.append(row, ignore_index=True)
            else: 
                means = self.clean_mean_z_score(self.five_tfs, max_val=2)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[1][0].publish(new_msg) # 5 zscore2
                
                means = self.clean_mean_z_score(self.five_tfs)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[1][1].publish(new_msg) # 5 zscore3
                
                means = self.clean_mean_iqr(self.five_tfs)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[1][2].publish(new_msg) # 5 iqr
                
                self.five_tfs = self.five_tfs.iloc[0:0]
             
            # -----------------------------------------------------------------
 
            if len(self.ten_tfs.index) < 10:
                self.ten_tfs = self.ten_tfs.append(row, ignore_index=True)
            else:  
                means = self.clean_mean_z_score(self.ten_tfs, max_val=2)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[2][0].publish(new_msg) # 10 zscore2
                
                means = self.clean_mean_z_score(self.ten_tfs)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[2][1].publish(new_msg) # 10 zscore3
                
                means = self.clean_mean_iqr(self.ten_tfs)
                new_msg = self.create_new_mean_msg(msg, [tfs_id4[0]], means)
                self.tf_mot_publishers[2][2].publish(new_msg) # 10 iqr
                
                self.ten_tfs = self.ten_tfs.iloc[0:0]
                
                
    def pose_publish_mot(self, msg):
        poses_id4 = []
        for i, pose in enumerate(msg.poses):
            tag_id = msg.marker_ids[i]
            if tag_id == 4:
                poses_id4.append(pose)
        
        if len(poses_id4) == 1:
            pos = poses_id4[0].position
            orient = poses_id4[0].orientation
            row = pd.Series([pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w],
                            index=columns)
            
            if len(self.three_poses.index) < 3:
                self.three_poses = self.three_poses.append(row, ignore_index=True)
            else: 
                means = self.clean_mean_z_score(self.three_poses, max_val=2)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[0][0].publish(new_msg) # 3 zscore2
                
                means = self.clean_mean_z_score(self.three_poses)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[0][1].publish(new_msg) # 3 zscore3
                
                means = self.clean_mean_iqr(self.three_poses)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[0][2].publish(new_msg) # 3 iqr
                
                self.three_poses = self.three_poses.iloc[0:0]
                
            # -----------------------------------------------------------------    
                
            if len(self.five_poses.index) < 5:
                self.five_poses = self.five_poses.append(row, ignore_index=True)
            else: 
                means = self.clean_mean_z_score(self.five_poses, max_val=2)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[1][0].publish(new_msg) # 5 zscore2
                
                means = self.clean_mean_z_score(self.five_poses)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[1][1].publish(new_msg) # 5 zscore3
                
                means = self.clean_mean_iqr(self.five_poses)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[1][2].publish(new_msg) # 5 iqr
                
                self.five_poses = self.five_poses.iloc[0:0]
             
            # -----------------------------------------------------------------
     
            if len(self.ten_poses.index) < 10:
                self.ten_poses = self.ten_poses.append(row, ignore_index=True)
            else:  
                means = self.clean_mean_z_score(self.ten_poses, max_val=2)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[2][0].publish(new_msg) # 10 zscore2
                
                means = self.clean_mean_z_score(self.ten_poses)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[2][1].publish(new_msg) # 10 zscore3
                
                means = self.clean_mean_iqr(self.ten_poses)
                new_msg = self.create_new_mean_msg(msg, [poses_id4[0]], means)
                self.pose_mot_publishers[2][2].publish(new_msg) # 10 iqr
                
                self.ten_poses = self.ten_poses.iloc[0:0]           
               
                
 
    # def create_new_mean_msg(self, msg, tf, means):
    #     tf.transform.translation.x = means[0]
    #     tf.transform.translation.y = means[1]
    #     tf.transform.translation.z = means[2]
    #     tf.transform.rotation.x = means[3]
    #     tf.transform.rotation.y = means[4]
    #     tf.transform.rotation.z = means[5]
    #     tf.transform.rotation.w = means[6]
        
    #     new_msg = msg
    #     new_msg.transforms = [tf]
        
    #     return new_msg
    
    
    def create_new_mean_msg(self, msg, tfs_poses, means):
        new_msg = msg
        
        if type(new_msg) == TFMessage:
            new_msg.transforms = []
            for tf in tfs_poses:
                tf.transform.translation.x = means[0]
                tf.transform.translation.y = means[1]
                tf.transform.translation.z = means[2]
                tf.transform.rotation.x = means[3]
                tf.transform.rotation.y = means[4]
                tf.transform.rotation.z = means[5]
                tf.transform.rotation.w = means[6]
                new_msg.transforms.append(tf)
        elif type(new_msg) == ArucoMarkers:
            new_msg.poses = []
            for pose in tfs_poses:
                pose.position.x = means[0]
                pose.position.y = means[1]
                pose.position.z = means[2]
                pose.orientation.x = means[3]
                pose.orientation.y = means[4]
                pose.orientation.z = means[5]
                pose.orientation.w = means[6]
                new_msg.poses.append(pose)
            
        return new_msg
    
    
    
    def clean_mean_z_score(self, df, max_val=3):
        # print("--------------------------------------------")
        # print(len(df))
        # print(len(self.three_tfs.index))
        # print(len(self.five_tfs.index))
        # print(len(self.ten_tfs.index))
        warnings.filterwarnings('error')
        try:
            check_df = df.loc[:, (df.std()!=0)]
            if not check_df.empty:
                z_scores = np.abs(stats.zscore(check_df))
                # print("Zscores", z_scores)
                df = df[(z_scores<max_val).all(axis=1)]
                # (print("df", df))
        except Warning:
            print("--------------------------------------------")
            print(df)
            print(check_df)
            print(not check_df.empty)
            print(max_val)
            
        
        return df.mean()
        
    
    
    def clean_mean_iqr(self, df):
        q1 = df.quantile(q=.25)
        q3 = df.quantile(q=.75)
        iqr = df.apply(stats.iqr)
        lower_range = q1-(1.5*iqr)
        upper_range = q3 + (1.5*iqr)
        df = df[~((df<lower_range) | (df>upper_range)).any(axis=1)]
        return df.mean()
        





            
            
def main(args=None):

    rclpy.init(args=args)
    
    imprTests = Improvement_Tests()
    rclpy.spin(imprTests)
    
    imprTests.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()