#! /usr/bin/env python3

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from attach_shelf.srv import GoToLoading
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RobotUtilsNode(Node):

    def __init__(self):
        super().__init__('robot_utils_node')
        self.declare_parameter('target', 'sim')

        self.cli = self.create_client(GoToLoading, '/approach_shelf')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.req = GoToLoading.Request()
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.elevator_down_ = self.create_publisher(String, '/elevator_down', 1)
        self.loc_fprint_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.glb_fprint_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.attached_shelf_stat = 0
        self.pub_fprint_cnt = 0
        self.loaded_robot_shape = Polygon()
        self.loaded_robot_shape.points = [
            Point32(x=0.50,  y=0.40,  z=0.0),
            Point32(x=0.50,  y=-0.40, z=0.0),
            Point32(x=-0.50, y=-0.40, z=0.0),
            Point32(x=-0.50, y=0.40,  z=0.0)
        ] 
        self.normal_robot_shape = Polygon()
        self.normal_robot_shape.points = [
            Point32(x=0.13,  y=0.13,  z=0.0),
            Point32(x=0.13,  y=-0.13, z=0.0),
            Point32(x=-0.13, y=-0.13, z=0.0),
            Point32(x=-0.13, y=0.13,  z=0.0)
        ] 
        self.env_ = self.get_parameter('target')
        self.elev_msg = String()
        self.shelf_down_cnt = 0
        self.move_robot_cnt = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0, self.timer_callback, callback_group=timer_cb_group)
        self._loop_rate = self.create_rate(2.0, self.get_clock()) # 2Hz


    def send_attach_shelf_request(self):
        self.req.attach_to_shelf = True;
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def timer_callback(self):
        if self.move_robot_cnt > 0:
            twist = Twist()
            twist.linear.x = self.linear_vel
            twist.angular.z = self.angular_vel
            if(self.move_robot_cnt < 2):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.cmd_vel_pub_.publish(twist)
            self.move_robot_cnt -= 1

        if self.shelf_down_cnt > 0:
            self.elev_msg.data = 'down'
            self.elevator_down_.publish(self.elev_msg)
            self.shelf_down_cnt -= 1

        if self.pub_fprint_cnt > 0:
            if self.attached_shelf_stat == 1:
                self.loc_fprint_.publish(self.loaded_robot_shape)
                self.glb_fprint_.publish(self.loaded_robot_shape)
            elif self.attached_shelf_stat == 2:
                self.loc_fprint_.publish(self.normal_robot_shape)
                self.glb_fprint_.publish(self.normal_robot_shape)
            self.pub_fprint_cnt -= 1


# robot positions for moving
move_positions = {
    "init_position":        [-0.2, 0.0, 0.0],
    "loading_position":      [5.8, 0.0, -1.7],  # -1.57
    "shipping_position":    [2.4, 1.4, 1.37]    # 1.57
}


def main():
    rclpy.init()

    robot_utils = RobotUtilsNode()

    if robot_utils.env_ == 'sim':
        print('Robot will run at simulated env')
    elif robot_utils.env_ == 'real':
        print('Robot will run at real env')
    else:
        print('Do not know where is the place the robot is running env')

    navigator = BasicNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(robot_utils)
    executor.add_node(navigator)

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = move_positions["init_position"][0]
    initial_pose.pose.position.y = move_positions["init_position"][1]
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # initial_pose.pose.orientation.w = move_positions["init_position"][2]

    r = R.from_euler('xyz',[0,0,move_positions["init_position"][2]])
    initial_pose.pose.orientation.x = r.as_quat()[0]
    initial_pose.pose.orientation.y = r.as_quat()[1]
    initial_pose.pose.orientation.z = r.as_quat()[2]
    initial_pose.pose.orientation.w = r.as_quat()[3]

    navigator.setInitialPose(initial_pose)
    print('Set the Initial position at Home position.')

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    move_to_pose = PoseStamped()
    move_to_pose.header.frame_id = 'map'
    move_to_pose.header.stamp = navigator.get_clock().now().to_msg()
    move_to_pose.pose.position.x = move_positions["loading_position"][0]
    move_to_pose.pose.position.y = move_positions["loading_position"][1]
    # move_to_pose.pose.orientation.z = 1.0
    # move_to_pose.pose.orientation.w = move_positions["loading_position"][2]
    r = R.from_euler('xyz',[0,0,move_positions["loading_position"][2]])
    move_to_pose.pose.orientation.x = r.as_quat()[0]
    move_to_pose.pose.orientation.y = r.as_quat()[1]
    move_to_pose.pose.orientation.z = r.as_quat()[2]
    move_to_pose.pose.orientation.w = r.as_quat()[3]

    print('Request to move at loading position x(%f), y(%f), theta(%f)' 
            % (move_positions['loading_position'][0], 
                move_positions['loading_position'][1], 
                move_positions['loading_position'][2]))

    navigator.goToPose(move_to_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at loading position: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Arrived at loading position, now approaching to underneath the shelf.')

        response = robot_utils.send_attach_shelf_request()
        if (response.complete == True):
            robot_utils.get_logger().info('Successfuly attached')
            robot_utils.attached_shelf_stat = 1     # load the shelf, need to publish new footprint
            robot_utils.pub_fprint_cnt = 3

            while rclpy.ok():
                if robot_utils.pub_fprint_cnt <= 0:
                    break
                print('Publish new footprint for the loaded robot shape(%d)...' %(robot_utils.pub_fprint_cnt))
                rclpy.spin_once(robot_utils)

        else:
            robot_utils.get_logger().info('Failed to attach the shelf')
            robot_utils.destroy_node()
            exit(-1)

        # move back robot to 1.5 m
        robot_utils.linear_vel = -0.3
        robot_utils.angular_vel = 0.0
        robot_utils.move_robot_cnt = 10

        while rclpy.ok():
            if robot_utils.move_robot_cnt <= 0:
                break
            print('Move back to safe position(%d)...' %(robot_utils.move_robot_cnt))
            rclpy.spin_once(robot_utils)

    elif result == TaskResult.CANCELED:
        print('Task at moving to loading position was canceled. Returning to Home position...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        while not navigator.isTaskComplete():
            pass
        robot_utils.destroy_node()
        exit(-1)

    elif result == TaskResult.FAILED:
        print('Task at moving to loading position was failed!')
        robot_utils.destroy_node()
        exit(-1)

    move_to_pose = PoseStamped()
    move_to_pose.header.frame_id = 'map'
    move_to_pose.header.stamp = navigator.get_clock().now().to_msg()
    move_to_pose.pose.position.x = move_positions["shipping_position"][0]
    move_to_pose.pose.position.y = move_positions["shipping_position"][1]
    # move_to_pose.pose.orientation.z = 1.0
    # move_to_pose.pose.orientation.w = 0.0
    r = R.from_euler('xyz',[0,0,move_positions["shipping_position"][2]])
    move_to_pose.pose.orientation.x = r.as_quat()[0]
    move_to_pose.pose.orientation.y = r.as_quat()[1]
    move_to_pose.pose.orientation.z = r.as_quat()[2]
    move_to_pose.pose.orientation.w = r.as_quat()[3]

    print('Request to move at shipping position x(%f), y(%f), theta(%f)' 
            % (move_positions['shipping_position'][0], 
                move_positions['shipping_position'][1], 
                move_positions['shipping_position'][2]))

    navigator.goToPose(move_to_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at shipping position: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Arrived at shipping position, now put down the shelf and will get out from the underneath of the shelf.')
        robot_utils.shelf_down_cnt = 2
        while rclpy.ok():
            if robot_utils.shelf_down_cnt <= 0:
                break
            rclpy.spin_once(robot_utils)
            # robot_utils._loop_rate.sleep()
        print('elevator down.')

        robot_utils.attached_shelf_stat = 2     # unload the shelf, need to publish normal footprint
        robot_utils.pub_fprint_cnt = 3

        while rclpy.ok():
            if robot_utils.pub_fprint_cnt <= 0:
                break
            print('Publish normal footprint for the unloaded robot shape(%d)...' %(robot_utils.pub_fprint_cnt))
            rclpy.spin_once(robot_utils)

        # move back robot to 1.5 m
        robot_utils.linear_vel = -0.3
        robot_utils.angular_vel = 0.0
        robot_utils.move_robot_cnt = 10
        while rclpy.ok():
            if robot_utils.move_robot_cnt <= 0:
                break;
            print('Move back to safe position(%d)...' %(robot_utils.move_robot_cnt))
            rclpy.spin_once(robot_utils)
            # robot_utils._loop_rate.sleep()

        print('got out from underneath of the shelf.')

    elif result == TaskResult.CANCELED:
        print('Task at moving to shipping position was canceled. Returning to Home position...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
        while not navigator.isTaskComplete():
            pass
        robot_utils.destroy_node()
        exit(-1)

    elif result == TaskResult.FAILED:
        print('Task at moving to shipping position was failed!')
        robot_utils.destroy_node()
        exit(-1)

    print('Request to move at Home position x(%f), y(%f), theta(%f)' 
            % (move_positions['init_position'][0], 
                move_positions['init_position'][1], 
                move_positions['init_position'][2]))

    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at Init position: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
       print('Arrived at Init position, all tasks are done')

    elif result == TaskResult.CANCELED:
        print('Task at moving to Init position was canceled.')
        robot_utils.destroy_node()
        exit(-1)

    elif result == TaskResult.FAILED:
        print('Task at moving to shipping position was failed!')
        robot_utils.destroy_node()
        exit(-1)

    robot_utils.destroy_node()
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()