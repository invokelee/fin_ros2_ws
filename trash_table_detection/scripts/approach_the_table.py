#! /usr/bin/env python3

import argparse, sys
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from table_find_interface.action import GoUnderTable

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import ReliabilityPolicy

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import copy

class PID:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kp_err = 0.0
        self.Ki_err = 0.0
        self.Kd_err = 0.0

    def calcPID(self, current_value):
        self.Kd_err = current_value - self.Kp_err
        self.Kp_err = current_value
        self.Ki_err += current_value
        return self.Kp*self.Kp_err + self.Ki*self.Ki_err + self.Kd*self.Kd_err

    def setPID(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

class ApproachTable(Node):
    def __init__(self):
        super().__init__('trash_table_go_under')

        self.get_parameters()
        self.get_logger().warn("Robot NAME="+str(self.robot_name))

        self.callback_group = ReentrantCallbackGroup()

        if self.target_env_ == "sim":
            self.front_range_ix = 510   # 1081/2
        else:
            self.front_range_ix = 360   # 720/2
        
        self.angular_pid = PID(Kp=0.4, Ki=0.001, Kd=0.05)

        # We init the positional values
        self.left_min_dist_index = None
        self.leftmost_inner_index = None
        self.left_min_value = None
        self.right_min_dist_index = None
        self.rightmost_inner_index = None
        self.right_min_value = None
        self.laser_length = None
        self.angle_between_left_right = None
        self.is_table = False
        self.MAX_DIST_FROM_TABLE = 1.5
        self.MEAN_DIST_FROM_TABLE = 1.0
        self.MIN_DIST_FROM_TABLE = 0.2
        self.BOUNDARY_DEPTH = 0.05
        self.MIN_DIST_BODY = 0.20
        self.left_leg_idx = None
        self.right_leg_idx = None
        self.front_leg_delta_idx = None
        self.find_near_front_leg = False

        self.robot_stat = "ready_to_action"
        self.RBot = True
        self.ns = '/' + self.robot_name
        if self.target_env_ == "sim":
            self.RBot = False
            self.ns = ""
            self.Table_Width = 0.58
        else:
            self.Table_Width = 0.45
 
        self.scan_sub_ = self.create_subscription(LaserScan, '{}/scan'.format(self.ns), self.scan_callback, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.callback_group
        )

        self.table_pos = "nothing"
        self.SCAN_BIG_DIST = 100.0

        if self.target_env_ == 'sim':
            self.action_gu_server_name = '/go_under_table'
        else:
            self.action_gu_server_name = self.ns + '/go_under_table'

        self._action_gu_server = ActionServer(self, GoUnderTable, self.action_gu_server_name, self.go_under_table, callback_group=self.callback_group)

        if self.target_env_ == 'sim':
            self.vel_pub_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped',
                                    10, callback_group=self.callback_group)
        else:
            self.vel_pub_ = self.create_publisher(Twist, '{}/cmd_vel'.format(self.ns), 
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

        # We get the left closest reading, x[:int(x.size/2)]
        # self.left_min_dist_index = np.nanargmin(msg.ranges[:int(len(msg.ranges)/2)])
        self.left_min_dist_index = np.nanargmin(fmsg.ranges[:self.front_range_ix])

        self.left_min_value = msg.ranges[self.left_min_dist_index]

        # We get the right closest reading, x[int(x.size/2):]
        self.right_min_dist_index = np.nanargmin(fmsg.ranges[self.front_range_ix:]) + self.front_range_ix

        self.right_min_value = msg.ranges[self.right_min_dist_index]

        # Find left leg most right boundary 
        self.leftmost_inner_index = self.left_min_dist_index
        bl = self.left_min_value
        for i in range(self.left_min_dist_index+1, self.front_range_ix):
            if abs(fmsg.ranges[i] - bl) > self.BOUNDARY_DEPTH:
                self.leftmost_inner_index = i-1
                break
            bl = fmsg.ranges[i]
            
        # Find right leg most left boundary 
        self.rightmost_inner_index = self.right_min_dist_index
        br = self.right_min_value
        for i in range(self.right_min_dist_index-1, self.front_range_ix, -1):
            if abs(fmsg.ranges[i] - br) > self.BOUNDARY_DEPTH:
                self.rightmost_inner_index = i+1
                break
            br = fmsg.ranges[i]

        # We calculate the angle in radians between both detections
        angle_left = msg.angle_increment * self.left_min_dist_index
        angle_right = msg.angle_increment * self.right_min_dist_index
        self.angle_between_left_right = angle_right - angle_left
        
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

    
    def check_if_table(self, error = 0.05):
        d1 = self.right_min_value
        d2 = self.left_min_value

        beta = self.angle_between_left_right
        aux = np.square(d2) + np.square(d1) - 2*d1*d2*np.cos(beta)
        w = np.sqrt(aux)
        
        width_ok = w >= self.Table_Width - error and w <= self.Table_Width + error
        if width_ok:
            distance_ok = d1 <= self.MAX_DIST_FROM_TABLE and d2 <= self.MAX_DIST_FROM_TABLE
            if not distance_ok:
                self.get_logger().debug("TABLE TOO FAR AWAY=d1="+str(d1)+",d2="+str(d2))
                self.is_table = False
            else:
                # We nee dto check that its INFRONT, not on the back
                table_is_front = self.table_pos == "front"
                if not table_is_front:
                    self.get_logger().debug("NOT IN FRONT="+str(self.table_pos))
                else:
                    self.get_logger().debug("IS TABLE IN FRONT =d1="+str(d1)+",d2="+str(d2)+"w="+str(w))
                    self.is_table = True
        else:
            self.get_logger().debug("WIDTH TABLE WRONG=w="+str(w))
            self.is_table = False

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

    def go_under_table(self,goal_handle):

        self.robot_stat = "searching_table"
        feedback_msg = GoUnderTable.Feedback()        

        # We do the Three fases: FIND, GO UNDER and CENTER
        if goal_handle.request.start:
            feedback_msg.phase = "searching_for_table"
        else:
            feedback_msg.phase = "skip searching_for_table"
                
        goal_handle.publish_feedback(feedback_msg)

        if goal_handle.request.start:
            self.find_table()
        
        feedback_msg.phase = "moving_under_table"
        goal_handle.publish_feedback(feedback_msg)
        self.robot_stat = "approaching_table"
        self.get_under_table()

        feedback_msg.phase = "centering_under_table"
        goal_handle.publish_feedback(feedback_msg)
        self.robot_stat = "centering_under_table"
        self.center()
        
        feedback_msg.phase = "finished_go_under_table"
        goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info("Robot Found Table")
        goal_handle.succeed()
        result = GoUnderTable.Result()
        result.complete = True
        self.robot_stat = "ready_to_action"
        return result 

    def find_table(self):
        rate = self.create_rate(10)

        while not self.is_table:
            self.rotate(direction="left", w=0.5, x=0.0)
            rate.sleep()

        self.stop()
        self.get_logger().info("Robot Found Table")

        return True 
    
    def get_under_table(self):

        rate = self.create_rate(10)
        while self.table_pos != "back":
            # we want to make the angle of right and left min the closest as possible
            # delta_left = int(self.laser_length / 2) - self.left_min_dist_index
            delta_left = self.front_range_ix - self.leftmost_inner_index

            # delta_right = self.right_min_dist_index - int(self.laser_length / 2)
            delta_right = self.rightmost_inner_index - self.front_range_ix
            
            control_variable = (delta_left - delta_right) / 100.0
            if self.target_env_ == 'real':
                control_variable *= -1

            vel_w = self.angular_pid.calcPID(control_variable)

            # print(delta_left, "(",self.laser_length/2-self.left_min_dist_index,") ",
            #      delta_right,"(",self.right_min_dist_index-self.laser_length/2,") ")
            # print("--", control_variable, vel_w, self.right_min_value, self.left_min_value)

            self.rotate(direction=None, w=vel_w, x=0.05)
            rate.sleep()

        # keep going forward for a while
        self.went_front_num = 0
        print("Keep going forward loop to deep inside table..")
        cnt = 70
        if self.target_env_ == 'real':
            cnt = 20
        while self.went_front_num <= cnt:
            self.rotate(direction=None, w=0.0, x=0.05)
            if self.went_front_num % 5 == 0:
                self.get_logger().warn("went_front_num="+str(self.went_front_num))
            rate.sleep()
            self.went_front_num += 1
        print("Exit from keep going forward loop")

        self.stop()
        self.get_logger().info("Robot Under Table")

        return True 

    def center(self):
        # We Start from detectin on the back
        rate = self.create_rate(10)
        self.went_front_num = 0

        while self.table_pos == "back" or self.went_front_num <= 10:
            # we want to make the angle of right and left min the closest as possible
            # delta_left = int(self.laser_length / 2) - self.left_min_dist_index
            delta_left = self.front_range_ix - self.leftmost_inner_index
            
            # delta_right = self.right_min_dist_index - int(self.laser_length / 2)
            delta_right = self.rightmost_inner_index - self.front_range_ix

            # control_variable = -1*(delta_left - delta_right) / 100.0
            control_variable = (delta_left - delta_right) / 100.0
            if self.target_env_ == 'real':
                control_variable *= -1

            vel_w = self.angular_pid.calcPID(control_variable)

            # self.rotate(direction=None, w=vel_w, x=0.02)
            self.rotate(direction=None, w=0.0, x=0.03)
            
            rate.sleep()
            self.get_logger().warn("went_front_num="+str(self.went_front_num))
            if self.table_pos == "front":
                self.went_front_num += 1

        self.stop()
        self.get_logger().info("Robot is Centered under the Table")

        return True

# parser = argparse.ArgumentParser()
# parser.add_argument('-target', default='sim', help='Please set the Target Env(sim or real)') 
# parser.add_argument('-robot_name', default='rb1_robot', help='Please set the Target robot name for sim or real') 
# args = parser.parse_args()

def main(args=None):
    rclpy.init(args=args)

    try:
        # cleaner2_approach = ApproachTable(args.target, args.robot_name) 
        cleaner2_approach = ApproachTable() 
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(cleaner2_approach)

        try:
            executor.spin()

        finally:
            executor.shutdown()
            cleaner2_approach.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
