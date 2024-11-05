#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
import copy

class ScanInfoNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME
        
        # declare parameters
        self.declare_parameter('robot_name', "")
        self.get_parameters()
        self.get_logger().warn("Robot NAME="+str(self.robot_name))
        
        self.first = False
        self.callback_group = ReentrantCallbackGroup()

        self.scan_sub_ = self.create_subscription(LaserScan, 
                            '{}/scan'.format(self.robot_name), 
                            self.scan_callback, QoSProfile(depth=1, 
                                                        reliability=ReliabilityPolicy.BEST_EFFORT),
                            callback_group=self.callback_group
        )

    def get_parameters(self):
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

    def scan_callback(self, msg):

        if self.first == False:
            self.get_logger().info("Scan Info: ranges -> {}".format(len(msg.ranges)))
            self.get_logger().info("Angle min : {}, max : {}".format(msg.angle_min, msg.angle_max))
            self.get_logger().info("Angle Increment : {}".format(msg.angle_increment))
            self.first = True


        # # We publish the filtered data hat we are going to use
        # f_scan_msg = copy.deepcopy(msg)
        # f_scan_msg.ranges = [self.SCAN_BIG_DIST] * len(msg.ranges)

        # self.laser_length = len(f_scan_msg.ranges)

        # # We get the left closest reading, x[:int(x.size/2)]
        # self.left_min_dist_index = np.nanargmin(msg.ranges[:int(len(msg.ranges)/2)])
        # self.left_min_value = msg.ranges[self.left_min_dist_index]
        # f_scan_msg.ranges[self.left_min_dist_index] = self.left_min_value

        # # We get the right closest reading, x[int(x.size/2):]
        # self.right_min_dist_index = np.nanargmin(msg.ranges[int(len(msg.ranges)/2):]) + int(len(msg.ranges)/2)
        # self.right_min_value = msg.ranges[self.right_min_dist_index]
        # f_scan_msg.ranges[self.right_min_dist_index] = self.right_min_value


        # # We calculate the angle in radians between both detections
        # angle_left = f_scan_msg.angle_increment * self.left_min_dist_index
        # angle_right = f_scan_msg.angle_increment * self.right_min_dist_index
        # self.angle_between_left_right = angle_right - angle_left
        

        # self.check_if_table()
        # self.detection_table_side()
        
        # self.scan_pub_.publish(f_scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanInfoNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()



