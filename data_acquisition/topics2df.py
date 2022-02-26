#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node


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


class EvaluationData(Node):
    
    def __init__(self):
        super().__init__('eval_data')
        
        self.declare_parameter("is_tf", True)
        # self.declare_parameters(namespace='',
        #                         parameters=[
        #                             ('param_name1', None),
        #                             ('param_name2', None)])
        
        is_tf = bool(self.get_parameter('is_tf').value)#get_parameter_value().double_value 
        print(is_tf)
        
        # param_str = self.get_parameter('my_str')
        # param_int = self.get_parameter('my_int')
        # param_double_array = self.get_parameter('my_double_array')
        # self.get_logger().info("str: %s, int: %s, double[]: %s" %
        #                        (str(param_str.value),
        #                         str(param_int.value),
        #                         str(param_double_array.value),))
        
        
        
def main(args=None):
    rclpy.init(args=args)
    
    eval_data = EvaluationData()
    rclpy.spin(eval_data)
    
    eval_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()