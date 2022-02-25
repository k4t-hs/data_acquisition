#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import numpy as np
# import pandas as pd
# import scipy.stats as stats
# import warnings

# import rclpy
from rclpy.node import Node
from std_msgs.msg import String




columns = ["translation x", "translation y", "translation z",
           "rotation x", "rotation y", "rotation z", "rotation w"]

class Key_Input_Node(Node):

    def __init__(self):
        super().__init__("key_input")
               
        self.key_publisher = self.create_publisher(String, "key_pub", 10) 
        
        # print("Next action?: ")
        key_input = input("Next action?: ")
        print("Received raw_input:", key_input)


            
            
def main(args=None):

    # rclpy.init(args=args)
    
    key_node = Key_Input_Node()
    # rclpy.spin(key_node)
    
    # while not is_shutdown():
        
    
    
    key_node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()