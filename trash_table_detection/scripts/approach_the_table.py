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

class ApproachTable(Node):
    def __init__(self, env_, name_):
        super().__init__('trash_table_go_under')

        if env_ == 'sim':
            self.target_env_ = 'sim'
        else:
            self.target_env_ = 'real'

        # callback group for _sub and _action_server callbacks
        self.callback_group = ReentrantCallbackGroup()

        # declare parameters
        self.robot_name = name_
        # self.declare_parameter('robot_name', "cleaner_2")
        # self.get_parameters()
        self.get_logger().warn("Robot NAME="+str(self.robot_name))

        self.ranges = []
        #        1081 (540)    (500) 1000
        # self.index_a = 580    # 520
        # self.index_b = 500    # 480
        # self.index_x = 240    # 350
        # self.index_y = 0      # 150(174')
        # self.desired_range_a = 0.85
        # self.desired_range_b = 0.85
        # self.desired_right_range = 0.40
        # self.thresh = 0.02
        self.P = 0.9
        self.D = 0.05
        if self.target_env_ == "sim":
            self.front_range_ix = 540   # 1081/2
        else:
            self.front_range_ix = 360   # 720/2
        
        self.angular_controller = Controller(P=self.P, D=self.D)

        # self.aligning = False

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
        self.RBot = True
        self.ns = self.robot_name
        if self.target_env_ == "sim":
            self.RBot = False
            self.ns = ""
            self.Table_Width = 0.58
        else:
            self.Table_Width = 0.42
 
        self.scan_sub_ = self.create_subscription(LaserScan, '{}/scan'.format(self.ns), self.scan_callback, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.callback_group
        )

        self.table_pos = "nothing"
        self.SCAN_BIG_DIST = 100.0
        qos_profile_publisher = QoSProfile(depth=1)
        qos_profile_publisher.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.scan_pub_ = self.create_publisher(msg_type=LaserScan,
                                                topic='{}/scan_filtered'.format(self.ns),
                                                qos_profile=qos_profile_publisher,
                                                callback_group=self.callback_group)


        if self.RBot:
            self.action_gu_server_name = self.robot_name + '/go_under_table'
        else:
            self.action_gu_server_name = '/go_under_table'

        self._action_gu_server = ActionServer(self, GoUnderTable, self.action_gu_server_name, self.go_under_table, callback_group=self.callback_group)

        if self.RBot:
            self.vel_pub_ = self.create_publisher(Twist, '{}/cmd_vel'.format(self.robot_name), 10, callback_group=self.callback_group)
        else:
            self.vel_pub_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped',
                                    10, callback_group=self.callback_group)
        
        self.vel_cmd = Twist()



    def get_parameters(self):
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

    def scan_callback(self, msg):


        # We publish the filtered data hat we are going to use
        f_scan_msg = copy.deepcopy(msg)
        f_scan_msg.ranges = [self.SCAN_BIG_DIST] * len(msg.ranges)

        self.laser_length = len(f_scan_msg.ranges)

        # We get the left closest reading, x[:int(x.size/2)]
        self.left_min_dist_index = np.nanargmin(msg.ranges[:int(len(msg.ranges)/2)])
        self.left_min_value = msg.ranges[self.left_min_dist_index]
        f_scan_msg.ranges[self.left_min_dist_index] = self.left_min_value

        # Find left leg most right boundary 
        self.leftmost_inner_index = self.left_min_dist_index
        bl = self.left_min_value
        for i in range(self.left_min_dist_index+1, int(len(msg.ranges)/2)):
            if abs(msg.ranges[i] - bl) > self.BOUNDARY_DEPTH:
                self.leftmost_inner_index = i
                break
            bl = msg.ranges[i]
            
        # We get the right closest reading, x[int(x.size/2):]
        self.right_min_dist_index = np.nanargmin(msg.ranges[int(len(msg.ranges)/2):]) + int(len(msg.ranges)/2)
        self.right_min_value = msg.ranges[self.right_min_dist_index]
        f_scan_msg.ranges[self.right_min_dist_index] = self.right_min_value

        # Find right leg most left boundary 
        self.rightmost_inner_index = self.right_min_dist_index
        br = self.right_min_value
        for i in range(self.right_min_dist_index-1, int(len(msg.ranges)/2)):
            if abs(msg.ranges[i] - br) > self.BOUNDARY_DEPTH:
                self.rightmost_inner_index = i
                break
            br = msg.ranges[i]

        # We calculate the angle in radians between both detections
        angle_left = f_scan_msg.angle_increment * self.left_min_dist_index
        angle_right = f_scan_msg.angle_increment * self.right_min_dist_index
        self.angle_between_left_right = angle_right - angle_left
        

        self.check_if_table()
        self.detection_table_side()
        
        self.scan_pub_.publish(f_scan_msg)
        

    def detection_table_side(self):
        S = self.laser_length
        if self.target_env_ == "sim":
            idx = 60                    # 80 x 0.004364 = 0.34912(+- 20' ), 40-> 10'
        else:
            idx = 130                   # S/2 - 2.007 * 0.008714
         
        # if self.left_min_dist_index <= (3.0/16.0)*S and self.right_min_dist_index >= (13.0/16.0)*S:
        if self.left_min_dist_index <= idx and self.right_min_dist_index >= (S-idx):
            # Table legs on the back of the robot1
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

        feedback_msg = GoUnderTable.Feedback()        

        # We do the Three fases: FIND, GO UNDER and CENTER
        feedback_msg.phase = "searching_for_table"
        goal_handle.publish_feedback(feedback_msg)
        self.find_table()

        feedback_msg.phase = "moving_under_table"
        goal_handle.publish_feedback(feedback_msg)
        self.get_under_table()

        feedback_msg.phase = "centering_under_table"
        goal_handle.publish_feedback(feedback_msg)
        self.center()

        # TODO: Debug to make it work
        # if "barista" in self.robot_name:
        #     feedback_msg.phase = "centering_barista_under_table"
        #     goal_handle.publish_feedback(feedback_msg)
        #     self.center_barista()
        
        feedback_msg.phase = "finished_go_under_table"
        goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info("Robot Found Table")
        goal_handle.succeed()
        result = GoUnderTable.Result()
        result.complete = True
        return result 

    def find_table(self):
        """
        Start rotating looking for detecting objects in boh sides at a maximum distance
        When done, it check if its a possible table
        If not then continues
        """
        
        rate = self.create_rate(10)

        while not self.is_table:
            
            self.rotate(direction="left", w=0.5, x=0.0)
            rate.sleep()

        self.stop()
        self.get_logger().info("Robot Found Table")

        return True 
    



    def get_under_table(self):
        """
        Gets Under the table until the front legs are 
        behind the robot, but close than the other legs
        """

        rate = self.create_rate(10)
        self.angular_controller.setPoint(0.36)
        pass_front_leg = False
        while self.table_pos != "back":
        # while not pass_front_leg:

            # we want to make the angle of right and left min the closest as possible
            delta_left = int(self.laser_length / 2) - self.left_min_dist_index
            dl = int(self.laser_length / 2) - self.left_min_dist_index
            # delta_left = int(self.laser_length / 2) - self.leftmost_inner_index

            delta_right = self.right_min_dist_index - int(self.laser_length / 2)
            dr = self.right_min_dist_index - int(self.laser_length / 2)
            # delta_right = self.rightmost_inner_index - int(self.laser_length / 2)

            # if delta_left >= 440 and delta_right >= 440:    # 360 -> 90' 
            #     control_variable = 1*(delta_left - delta_right) / 100.0
            # else:
            #     control_variable = -1*(delta_left - delta_right) / 100.0

            control_variable = -1*(delta_left - delta_right) / 100.0
            vel_w = self.angular_controller.update(control_variable)
            print(delta_right, delta_left, control_variable, vel_w)
            print("--", self.right_min_value, self.left_min_value)

            # self.rotate(direction=None, w=vel_w, x=0.05)
            self.rotate(direction=None, w=vel_w, x=0.05)

            if delta_left >= 480 and delta_right >= 480:    # 340 -> 85' 
                pass_front_leg = True
            rate.sleep()

        # keep going forward for a while
        self.went_front_num = 0
        print("Keep going forward loop to deep inside table..")
        while self.went_front_num <= 70:
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
        """
        Mves forwards until boh four legs are at the same distance
        """
        # We Start from detectin on the back

        rate = self.create_rate(10)
        self.went_front_num = 0
        self.angular_controller.setPoint(0.3)
        while self.table_pos == "back" or self.went_front_num <= 20:

            # we want to make the angle of right and left min the closest as possible
            delta_left = int(self.laser_length / 2) - self.left_min_dist_index
            delta_right = self.right_min_dist_index - int(self.laser_length / 2)

            # control_variable = -1*(delta_right - delta_left) / 100.0

            control_variable = -1*(delta_left - delta_right) / 100.0
            
            vel_w = self.angular_controller.update(control_variable)

            self.rotate(direction=None, w=vel_w, x=0.03)
            rate.sleep()
            self.get_logger().warn("went_front_num="+str(self.went_front_num))
            if self.table_pos == "front":
                self.went_front_num += 1

        self.stop()
        self.get_logger().info("Robot is Centered under the Table")

        return True

    def center_barista(self):
        """
        Mves forwards until the legs in fornt of him
        are at a distance of self.MIN_DIST_FROM_TABLE
        """
        # We Start from detectin on the back

        rate = self.create_rate(10)

        d1 = self.right_min_value
        d2 = self.left_min_value
        close_enough = d1 <= self.MIN_DIST_FROM_TABLE and d2 <= self.MIN_DIST_FROM_TABLE
        far_enough = d1 >= self.MIN_DIST_FROM_TABLE+0.1 and d2 <= self.MIN_DIST_FROM_TABLE+0.1

        self.get_logger().warn("[d1,d2]=["+str(d1)+","+str(d2)+"],"+str(self.table_pos)+",c="+str(close_enough)+",f="+str(far_enough))
        while not close_enough:
            d1 = self.right_min_value
            d2 = self.left_min_value
            close_enough = d1 <= self.MIN_DIST_FROM_TABLE and d2 <= self.MIN_DIST_FROM_TABLE
            self.get_logger().warn("[d1,d2]=["+str(d1)+","+str(d2)+"],"+str(self.table_pos)+","+str(close_enough))
            # we want to make the angle of right and left min the closest as possible
            delta_left = int(self.laser_length / 2) - self.left_min_dist_index
            delta_right = self.right_min_dist_index - int(self.laser_length / 2)

            control_variable = -1*(delta_right - delta_left) / 100.0
            vel_w = self.angular_controller.update(control_variable)

            self.rotate(direction=None, w=vel_w, x=0.02)
            rate.sleep()

        self.stop()
        self.get_logger().warn("Robot Type Barista is Centered under the Table")

        return True

parser = argparse.ArgumentParser()
parser.add_argument('-target', default='sim', help='Please set the Target Env(sim or real)') 
parser.add_argument('-robot_name', default='rb1_robot', help='Please set the Target robot name for sim or real') 
args = parser.parse_args()

def main(rgv, args):
    rclpy.init()

    try:
        cleaner2_approach = ApproachTable(args.target, args.robot_name) 
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
    argv = sys.argv
    main(argv, args)
