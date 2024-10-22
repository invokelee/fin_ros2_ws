#! /usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import ReliabilityPolicy

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import copy

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class TrashTableFinder(Node):
    def __init__(self, robot_name):
        super().__init__('trash_table_finder')

        self.callback_group = MutuallyExclusiveCallbackGroup()

        # declare parameters
        self.robot_name = robot_name
        self.get_logger().warn("Robot NAME="+str(self.robot_name))

        self.ranges = []

        # We init the positional values
        self.left_min_dist_index = None
        self.left_min_value = None
        self.right_min_dist_index = None
        self.right_min_value = None
        self.laser_length = None
        self.angle_between_left_right = None
        self.is_table = False
        self.table_status = None
        self.MAX_DIST_FROM_TABLE = 1.5
        self.MEAN_DIST_FROM_TABLE = 1.0
        self.MIN_DIST_FROM_TABLE = 0.2
        self.CHK_LEG_DIST = 0.35
        self.left_leg_idx = None
        self.right_leg_idx = None
        self.front_leg_delta_idx = None
        self.find_near_front_leg = False
        self.RBot = True
        self.ns = self.robot_name
        if self.robot_name == "rb1_robot":
            self.RBot = False
            self.ns = ""
            self.Table_Width = 0.58
        else:
            self.Table_Width = 0.42
 
        self.P = 0.9
        self.D = 0.05

        self.front_range_ix = 540   # 500
        
        self.angular_controller = Controller(P=self.P, D=self.D)

        self.scan_sub_ = self.create_subscription(LaserScan, '{}/scan'.format(self.ns), self.scan_callback, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.callback_group
        )

        self.table_pos = "nothing"
        self.SCAN_BIG_DIST = 100.0
        # qos_profile_publisher = QoSProfile(depth=1)
        # qos_profile_publisher.reliability = QoSReliabilityPolicy.BEST_EFFORT
        # self.scan_pub_ = self.create_publisher(msg_type=LaserScan,
        #                                         topic='{}/scan_filtered'.format(self.ns),
        #                                         qos_profile=qos_profile_publisher,
        #                                         callback_group=self.callback_group)

        # self.action_gu_server_name = self.robot_name + '/go_under_table'
        # self._action_gu_server = ActionServer(self, GoUnderTable, self.action_gu_server_name, self.go_under_table, callback_group=self.callback_group)
        if self.RBot:
            self.vel_pub_ = self.create_publisher(Twist, '{}/cmd_vel'.format(self.robot_name), 10, callback_group=self.callback_group)
        else:
            self.vel_pub_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped',
                                    10, callback_group=self.callback_group)
        self.vel_cmd = Twist()

    def scan_callback(self, msg):
        # We publish the filtered data hat we are going to use
        f_scan_msg = copy.deepcopy(msg)
        f_scan_msg.ranges = [self.SCAN_BIG_DIST] * len(msg.ranges)

        self.laser_length = len(f_scan_msg.ranges)

        # We get the left closest reading, x[:int(x.size/2)]
        self.left_min_dist_index = np.nanargmin(msg.ranges[:int(len(msg.ranges)/2)])
        self.left_min_value = msg.ranges[self.left_min_dist_index]
        f_scan_msg.ranges[self.left_min_dist_index] = self.left_min_value

        front_idx = int(len(msg.ranges)/2)
        front_leg_fg = 0
        front_range = int(1.5707/msg.angle_increment)
        for i in range(front_idx, front_idx - front_range, -1):
            if msg.ranges[i] < self.CHK_LEG_DIST:   # 0.35
                self.left_leg_idx = front_idx - i
                front_leg_fg += 1
                break

        # We get the right closest reading, x[int(x.size/2):]
        self.right_min_dist_index = np.nanargmin(msg.ranges[int(len(msg.ranges)/2):]) + int(len(msg.ranges)/2)
        self.right_min_value = msg.ranges[self.right_min_dist_index]
        f_scan_msg.ranges[self.right_min_dist_index] = self.right_min_value

        for i in range(front_idx+1, front_idx + front_range):
            if msg.ranges[i] < self.CHK_LEG_DIST:   # 0.35
                self.right_leg_idx = i - front_idx
                front_leg_fg += 1
                break

        # We calculate the angle in radians between both detections
        angle_left = f_scan_msg.angle_increment * self.left_min_dist_index
        angle_right = f_scan_msg.angle_increment * self.right_min_dist_index
        self.angle_between_left_right = angle_right - angle_left

        if front_leg_fg >= 2:
            self.front_leg_delta_idx = self.left_leg_idx - self.right_leg_idx
            self.find_near_front_leg = True
            print("Front leg - left: {}, right: {}".format(self.left_leg_idx, self.right_leg_idx))
        else:
            self.front_leg_delta_idx = 0
            self.find_near_front_leg = False

        self.check_if_table()
        self.detection_table_side()
        
        # self.scan_pub_.publish(f_scan_msg)
        

    def detection_table_side(self):
        S = self.laser_length
        if self.left_min_dist_index <= (3.0/16.0)*S and self.right_min_dist_index >= (13.0/16.0)*S:
            # Table legs on the back of the robot
            self.table_pos = "back"
        else:
            # Table Forwards or just in the middle of the legs
            self.table_pos = "front"

    
    def check_if_table(self, error = 0.05):
        """
        Based on cosine law, we get the distance between the two points
        based on the distance readings of each point an dthe angle
        If the value is around what the table should mesure, we say is a table
        """
        d1 = self.right_min_value
        d2 = self.left_min_value
        beta = self.angle_between_left_right
        aux = np.square(d2) + np.square(d1) - 2*d1*d2*np.cos(beta)
        w = np.sqrt(aux)

        # print("d1: {}, d2: {}, beta: {}".format(d1, d2, beta))
        # print("looking for Table_with : {}, Actual w : {}".format(self.Table_Width, w))

        width_ok = w >= self.Table_Width - error and w <= self.Table_Width + error
        if width_ok:
            distance_ok = d1 <= self.MAX_DIST_FROM_TABLE and d2 <= self.MAX_DIST_FROM_TABLE
            if not distance_ok:
                self.get_logger().debug("TABLE TOO FAR AWAY=d1="+str(d1)+",d2="+str(d2))
                self.is_table = False
                self.table_status = "too_far_away"
            else:
                # We nee dto check that its INFRONT, not on the back
                table_is_front = self.table_pos == "front"
                if not table_is_front:
                    self.get_logger().debug("NOT IN FRONT="+str(self.table_pos))
                    self.table_status = "not_in_front"
                else:
                    self.get_logger().debug("IS TABLE IN FRONT =d1="+str(d1)+",d2="+str(d2)+"w="+str(w))
                    self.is_table = True
                    self.table_status = "in_front"
                    print("Find the table in front : left: {}, right: {}, w: {}".format(d2, d1, w))
                    print("left leg idx: {}, right leg idx: {}".format(self.left_leg_idx, self.right_leg_idx))

        else:
            self.get_logger().debug("WIDTH TABLE WRONG=w="+str(w))
            self.is_table = False
            self.table_status = "not_table"
        # if self.is_table:
        #     print("- detected table({}) - Actual with {}".format(self.Table_Width, w))

    def find_table(self):
        """
        Start rotating looking for detecting objects in boh sides at a maximum distance
        When done, it check if its a possible table
        If not then continues
        """
        
        rate = self.create_rate(10)
        cnt = 10 * 12   # 12 sec 
        while not self.is_table and cnt > 0:
            cnt -= 1
            self.rotate(direction="left", w=0.5, x=0.0)
            self.executor.spin_once()       # rclpy.spin_once(self.node)
            rate.sleep()

        self.stop()
        if self.is_table:
            self.get_logger().info("Robot Found Table")
        else:
            self.get_logger().info("Robot Not Found Table")

        return self.is_table 

    def stop(self):
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub_.publish(self.vel_cmd)

    def rotate(self, direction, w=0.5, x=0.0):
        if direction == "left":
            self.vel_cmd.angular.z = w
            self.vel_cmd.linear.x = x
        elif direction == "right":
            self.vel_cmd.angular.z = -w
            self.vel_cmd.linear.x = x
        elif direction is None:
            self.vel_cmd.angular.z = w
            self.vel_cmd.linear.x = x
        self.vel_pub_.publish(self.vel_cmd)

    def align_to_front_leg(self):
        if not self.find_near_front_leg:
            print("Align to front leg - Not found near front leg...")
            return False
        rate = self.create_rate(10)
        aligned_fg = False
        cnt = 0
        while not aligned_fg:
            control_variable = -1*(self.front_leg_delta_idx) / 100.0
            vel_w = self.angular_controller.update(control_variable)
            if cnt % 5 == 0:
                print("Aligning to the center of the leg delta({}), Rotate the robot... - w : {}".format(self.front_leg_delta_idx, vel_w))
            self.rotate(direction=None, w=vel_w, x=0.0)
            rclpy.spin_once(self)
            # rate.sleep()
            if abs(self.front_leg_delta_idx) < 3:     # sim: 5 * 0.004364 / 3.141592 * 180 = 
                aligned_fg = True
            cnt += 1
        self.stop()
        print("Aligned to the center of the leg : {}".format(self.front_leg_delta_idx))
        self.get_logger().info("Robot aligned to the front legs")
        return True 

def main(args=None):
    rclpy.init(args=args)

    try:
        cleaner2_finder = TrashTableFinder(args.robot_name) 
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(cleaner2_finder)

        try:
            executor.spin()

        finally:
            executor.shutdown()
            cleaner2_finder.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
