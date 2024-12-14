#! /usr/bin/env python3

import rclpy
import time
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
    def __init__(self):
        super().__init__('trash_table_finder')

        self.get_parameters()
        self.get_logger().warn("Robot NAME="+str(self.robot_name))

        self.callback_group = MutuallyExclusiveCallbackGroup()

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
        self.MIN_DIST_BODY = 0.20
        self.CHK_LEG_DIST = 0.35  
        self.CENTER_ERR = 0.05      # 0.05 -> 2.87'
        self.angle_increment = None
        self.left_leg_idx = None
        self.right_leg_idx = None
        self.front_leg_delta_idx = None
        self.find_near_front_leg = False
        self.finding_fg = False
        self.RBot = True
        self.ns = '/' + self.robot_name
        self.P = 0.9
        self.D = 0.05
        self.angular_controller = Controller(P=self.P, D=self.D)

        if self.target_env_ == 'sim':
            self.RBot = False
            self.ns = ""
            self.Table_Width = 0.55     # 0.58
            self.front_range_ix = 510   # 1081/2
        else:
            self.Table_Width = 0.45
            self.front_range_ix = 360   # 720/2
 
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

    def get_parameters(self):
        self.declare_parameter('target', 'sim')
        self.declare_parameter('robot', 'rb1_robot')
        self.target_env_ = self.get_parameter('target').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot').get_parameter_value().string_value


    def scan_callback(self, msg):
        fmsg = copy.deepcopy(msg)
        for i in range(len(fmsg.ranges)):
            if fmsg.ranges[i] < self.MIN_DIST_BODY:
                fmsg.ranges[i] = self.MAX_DIST_FROM_TABLE

        self.laser_length = len(msg.ranges)
        self.angle_increment = msg.angle_increment

        # We get the left closest reading, x[:int(x.size/2)]
        # self.left_min_dist_index = np.nanargmin(msg.ranges[:int(len(msg.ranges)/2)])
        self.left_min_dist_index = np.nanargmin(fmsg.ranges[:self.front_range_ix])

        self.left_min_value = msg.ranges[self.left_min_dist_index]

        front_idx = self.front_range_ix
        front_leg_fg = 0
        front_range = int(1.5707/msg.angle_increment)
        for i in range(front_idx, front_idx - front_range, -1):
            if fmsg.ranges[i] < self.CHK_LEG_DIST:   # 0.35
                self.left_leg_idx = front_idx - i
                front_leg_fg += 1
                break

        # We get the right closest reading, x[int(x.size/2):]
        self.right_min_dist_index = np.nanargmin(fmsg.ranges[self.front_range_ix:]) + self.front_range_ix

        self.right_min_value = msg.ranges[self.right_min_dist_index]

        for i in range(front_idx+1, front_idx + front_range):
            if fmsg.ranges[i] < self.CHK_LEG_DIST:   # 0.35
                self.right_leg_idx = i - front_idx
                front_leg_fg += 1
                break

        # We calculate the angle in radians between both detections
        angle_left = msg.angle_increment * self.left_min_dist_index
        angle_right = msg.angle_increment * self.right_min_dist_index
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

    def detection_table_side(self):
        S = self.laser_length

        if self.target_env_ == "sim":
            idx = 60                    # 60 x 0.004364 = 0.26184(+- 15' ),=> 15' + (180-135) 40-> 10'
        else:
            idx = 120                   # (120 * 0.008714 / 3.141592)*180 = 60'
         
        if self.left_min_dist_index <= idx and self.right_min_dist_index >= (S-idx):
            # Table legs on the back of the robot1
            self.table_pos = "back"
        else:
            # Table Forwards or just in the middle of the legs
            self.table_pos = "front"

        # if self.target_env_ == "sim":
        #     idx = int(( 60 *3.14)/(180*0.004364))                   # (60'*3.14) / (180*0.004364) = 240
        # else:
        #     idx = int(( 60 *3.14)/(180*0.008714))                   # (60'*3.14) / (180*0.008714) = 120

        # if self.left_min_dist_index > self.front_range_ix - idx and self.right_min_dist_index < self.front_range_ix + idx:
        #     # Table legs are between +- 60' in the forward direction of the robot.
        #     self.table_pos = "front"
        # else:
        #     # Table legs on the back of the robot1
        #     self.table_pos = "back"

    
    def check_if_table(self, error = 0.05):

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
                # We need to check that its INFRONT, not on the back
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
        if self.is_table:
            print("- detected table({}) - Actual with {}".format(self.Table_Width, w))
        if self.finding_fg:
            print("Find the table in front : left: {}, right: {}, w: {}".format(d2, d1, w))
            print("Status: {}, left leg idx: {}, right leg idx: {}".format(self.table_status, self.left_leg_idx, self.right_leg_idx))
            self.finding_fg = False


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

    def align_to_front_leg(self, center=False):
        if not self.find_near_front_leg:
            print("Align to front leg - Not found near front leg...")
            return False
        rate = self.create_rate(10)
        aligned_fg = False
        cnt = 0
        while not aligned_fg:
            control_variable = (self.front_leg_delta_idx) / 100.0
            if self.target_env_ == 'sim':
                control_variable *= -1

            vel_w = self.angular_controller.update(control_variable)
            if cnt % 5 == 0:
                print("Aligning to the center of the leg delta({}), Rotate the robot... - w : {}".format(self.front_leg_delta_idx, vel_w))
            self.rotate(direction=None, w=vel_w*0.8, x=0.0)     # too fast to rotate, so w = vel_w * 0.5
            rclpy.spin_once(self)
            # rate.sleep()
            if abs(self.front_leg_delta_idx) < 3 and 20 < (self.left_leg_idx + self.right_leg_idx):     # sim: 5 * 0.004364 / 3.141592 * 180 = 
                aligned_fg = True
            cnt += 1
        self.stop()
        if aligned_fg and center :
            f_deg = (self.left_leg_idx + self.right_leg_idx) * self.angle_increment
            diff = f_deg - 1.5707
            if abs(diff) > self.CENTER_ERR:
                if diff > 0:
                    dir = -1
                else:
                    dir = 1
                print("Centered but too close to front...")
                cnt = int(abs(diff) * 180 / 3.141592)
                while cnt > 0:
                    if cnt % 5 == 0:
                        if dir < 0:
                            print("move back... {}".format(cnt))
                        else:
                            print("move front... {}".format(cnt))
                    x = 0.06
                    if self.target_env_ == 'sim':
                        x = 0.02
                    self.rotate(direction=None, w=0.0, x=x*dir)
                    time.sleep(0.1)
                    cnt -= 1
                self.stop()

        print("Aligned to the center of the leg : {}".format(self.front_leg_delta_idx))
        self.get_logger().info("Robot aligned to the front legs")
        return True 

def main(args=None):
    rclpy.init(args=args)

    try:
        # cleaner2_finder = TrashTableFinder(args.robot_name) 
        cleaner2_finder = TrashTableFinder() 
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
