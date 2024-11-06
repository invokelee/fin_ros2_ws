#! /usr/bin/env python3

import time
import argparse, sys
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionServer
from rclpy.action import ActionClient
from table_find_interface.action import GoUnderTable
from table_find_interface.action import CleanTable

from trash_table_detection.trash_table_finder import TrashTableFinder

class RobotUtilsNode(Node):

    def __init__(self):
        super().__init__('robot_utils_node')

        self.get_parameters()

        print('Target robot running env is ' + self.target_env_)
        self.get_logger().info('Target robot running env is %s' %(self.target_env_))

        if self.target_env_ == 'sim':
            self.cmd_vel_pub_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        else:
            self.cmd_vel_pub_ = self.create_publisher(Twist, '/cleaner_2/cmd_vel', 10)

        self.group_ac = MutuallyExclusiveCallbackGroup()
        self.table_finder_phase = "nothing"
        self.robot_state = "iddle"
        if self.target_env_ == 'sim':
            self.ac_name = "/go_under_table"
        else:
            self.ac_name = "/"+self.robot_name + "/go_under_table"

        self._table_finder_ac = ActionClient(self, GoUnderTable, self.ac_name, callback_group=self.group_ac)

        self.elevator_up_ = self.create_publisher(String, '/elevator_up', 1)
        self.elevator_down_ = self.create_publisher(String, '/elevator_down', 1)
        self.loc_fprint_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.glb_fprint_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.attached_table_stat = 2
        self.pub_fprint_cnt = 0
        self.find_rb_cnt = 0
        self.rb_direction = None
        self.loaded_robot_shape = Polygon()
        self.loaded_robot_shape.points = [
            Point32(x=0.50,  y=0.5,  z=0.0),
            Point32(x=0.50,  y=-0.5, z=0.0),
            Point32(x=-0.50, y=-0.5, z=0.0),
            Point32(x=-0.50, y=0.5,  z=0.0)
        ] 
        self.normal_robot_shape = Polygon()
        self.normal_robot_shape.points = [
            Point32(x=0.3,  y=0.3,  z=0.0),
            Point32(x=0.3,  y=-0.3, z=0.0),
            Point32(x=-0.3, y=-0.3, z=0.0),
            Point32(x=-0.3, y=0.3,  z=0.0)
        ] 

        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0, self.timer_callback, callback_group=timer_cb_group)
        self._loop_rate = self.create_rate(2.0, self.get_clock()) # 2Hz

    def get_parameters(self):
        self.declare_parameter('target', 'sim')
        self.declare_parameter('robot', 'rb1_robot')
        self.target_env_ = self.get_parameter('target').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot').get_parameter_value().string_value

    def timer_callback(self):
        if self.find_rb_cnt > 0:
            self.find_rb_cnt -= 1
            if self.find_rb_cnt <= 0:
                self.rb_stop()
            else:
                self.rb_rotate(self.rb_direction, 0.5, 0.0)

        if self.pub_fprint_cnt > 0:
            if self.attached_table_stat == 1:
                self.loc_fprint_.publish(self.loaded_robot_shape)
                self.glb_fprint_.publish(self.loaded_robot_shape)
            elif self.attached_table_stat == 2:
                self.loc_fprint_.publish(self.normal_robot_shape)
                self.glb_fprint_.publish(self.normal_robot_shape)
            self.pub_fprint_cnt -= 1

    #------------------------
    def send_goal(self, search=True):
        goal_msg = GoUnderTable.Goal()
        goal_msg.start = search

        self._table_finder_ac.wait_for_server()
        self._send_goal_future = self._table_finder_ac.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Table finder Result: {0}'.format(result.complete))
        self.robot_state = "trash-table_search_finished"


    def feedback_callback(self, feedback_msg):
        self.table_finder_phase = feedback_msg.feedback.phase
        self.robot_state = "trash-"+self.table_finder_phase
        self.get_logger().info(
            'Table Finder Feeback Recieved: {0}'.format(self.robot_state))
    #------------------------
    
    def rb_stop(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.cmd_vel_pub_.publish(vel_cmd)

    def rb_rotate(self, direction, w=0.5, x=0.0):
        vel_cmd = Twist()
        if direction == "left":
            vel_cmd.angular.z = w
            vel_cmd.linear.x = x
        elif direction == "right":
            vel_cmd.angular.z = -w
            vel_cmd.linear.x = x
        elif direction is None:
            vel_cmd.angular.z = w
            vel_cmd.linear.x = x
        self.cmd_vel_pub_.publish(vel_cmd)

    def rb_move_out(self, direction='forward', d_rotate='forward', t=5.5):
        # Start time
        start_time = time.time()
        tm_duration = t

        # Publish a message to move the robot backwards
        msg = Twist()
        if direction == 'backward':
            msg.linear.x = -0.25      # Move backwards
            mesg = 'Moving the robot backwards'
        else:
            msg.linear.x = 0.25      # Move forwards
            mesg = 'Moving the robot forwards'
        if d_rotate == 'left':
            msg.angular.z = 0.1
        elif d_rotate == 'right':
            msg.angular.z = -0.1
            
        self.cmd_vel_pub_.publish(msg)
        self.get_logger().info(mesg)

        while time.time() - start_time < tm_duration:
            self.cmd_vel_pub_.publish(msg)
            self.get_logger().info(mesg)
            time.sleep(0.1)  # Adjust the sleep time as needed for responsiveness

        # Stop the robot by publishing zero velocities
        self.cmd_vel_pub_.publish(Twist())
        self.get_logger().info('Stopping the robot')

    def rb_elevator_up(self):
        elev_msg = String()
        elev_msg.data = "up"  
        self.elevator_up_.publish(elev_msg)

    def rb_elevator_down(self):
        elev_msg = String()
        elev_msg.data = "down"  
        self.elevator_down_.publish(elev_msg)

#--------------------------

class CleanTrashTableAS(Node):

    def __init__(self):
        super().__init__('clean_trash_table_as')

        self.get_parameters()

        print('Target robot running env is ' + self.target_env_)
        self.get_logger().info('Target robot running env is %s' %(self.target_env_))

        self.group_as = MutuallyExclusiveCallbackGroup()
        if self.target_env_ == 'sim':
            self.as_name = "/clean_table"
        else:
            self.as_name = "/"+self.robot_name + "/clean_table"

        self.init_variable()

        if self.target_env_ == 'sim':
            self.move_positions = self.sim_move_positions
            self.check_positions = self.sim_check_positions

            self.table1_wp = self.sim_table_1_wp
            self.table2_wp = self.sim_table_2_wp
            self.table2_cor_wp = self.sim_table_2_cor_wp
            self.discovery_wp = self.sim_discovery_wp
            print('Target Env is Simulation')
        else:
            self.move_positions = self.real_move_positions
            print('Target Env is Real robot')

        self.set_initial_pose(navigator, self.move_positions['home_position'])
        # Wait for navigation to activate fully
        navigator.waitUntilNav2Active()

        self._action_ct_server = ActionServer(self, CleanTable, self.as_name, self.as_clean_table, callback_group=self.group_as)

        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0, self.timer_callback, callback_group=timer_cb_group)
        self._loop_rate = self.create_rate(2.0, self.get_clock()) # 2Hz

    def get_parameters(self):
        self.declare_parameter('target', 'sim')
        self.declare_parameter('robot', 'rb1_robot')
        self.target_env_ = self.get_parameter('target').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot').get_parameter_value().string_value

    def init_variable(self):
        self.requested_cmd = None

        # robot positions for moving
        self.sim_move_positions = {
            "home_position":    [0.0, 0.0, 0.0],
            "loading_pos1":     [ 0.1,   1.29, 0.0],
            "corridor_pos1":    [ 0.5,  -0.9, 0.0],
            "corridor_pos2":    [ 4.74,  -0.29, 0.0],
            "loading_pos2":     [ 4.3,  -1.31, 0.0],
            "put_down_pos1":    [ 7.22,  0.926, 0.0],
            "put_down_pos2":    [ 7.22, -1.926, 0.0],
        }

        self.sim_table_1_wp = {
            "loading_pos":      [ 4.3, -1.31, 0.0],
            "corridor_pos1":    [ 8.0,  0.0, -1.5707],
            "put_down_pos":     [ 8.0, -2.1, -1.5707],
        }

        self.sim_table_2_wp = {
            # "loading_pos":      [ 0.1,   1.4,  0.0],
            "loading_pos":      [ 1.1,   0.3,   1.5707],
            "corridor_pos1":    [ 0.5,  -0.9,   -0.5],
            "corridor_pos2":  [ 1.45,  -1.4,   0.0],
            "corridor_pos3":    [ 4.0,  -1.0,   0.8535],
            "corridor_pos4":    [ 5.0,   -0.2,   -0.2],
            "corridor_pos5":    [ 8.0,  0.0,   1.5707],
            "put_down_pos":     [ 8.0,  1.2, 1.5707],
        }

        self.sim_table_2_cor_wp = {
            "corridor_pos1":    [ 0.5,  -0.9,   -0.5],
            "corridor_pos2":    [ 1.45,  -1.4,   0.0],
            "corridor_pos3":    [ 4.0,  -1.0,   0.8535],
            "corridor_pos4":    [ 5.0,   -0.2,   -0.2],
            "corridor_pos5":    [ 8.0,  0.0,   1.5707],
        }

        self.sim_check_positions = {
            "cp_01":             [ 1.1,   0.3,   1.5707],
            # "cp_01":             [-0.5,   1.0,   1.5707],
            "cp_02":             [-1.79, -1.22, 3.14159],
            "cp_03":             [-0.784,-2.44, -1.5707],
            "cp_04":             [ 1.36, -1.42, 0.0],
            "cp_05":             [ 1.36, -3.02, -1.5707],
            "cp_06":             [ 1.36, -1.42, 1.5707],
            "cp_07":             [ 4.3,  -1.31, 0.0],
            "cp_08":             [ 5.09,  0.512, 1.5707],
        }

        self.sim_discovery_wp = {
            "corridor_pos1":    [ 0.5,  -0.9, -0.5],
            "corridor_pos2":    [ 1.45, -1.4,  0.0],
            "corridor_pos3":    [ 4.0,  -1.0,  0.8535],
            "corridor_pos4":    [ 5.0,  -0.2, -0.2],
            "corridor_pos5":    [ 8.0,   0.0,  1.5707],
            "put_down_pos":     [ 8.0,   1.2,  1.5707],
        }

        self.real_move_positions = {
            "home_position":  [ 0.018,  0.005,  0.0], # 0.018 0.005 -0.304
            "loading_pos":    [ 4.2,   -0.79,  -1.7],  # -1.57
            "corridor_pos1":  [ 0.5,   -0.9,    0.0],
            "put_down_pos":   [ 1.9,    0.9,    1.37]    # 1.57
        }

        self.real_table_1_wp = {
            "loading_pos":    [ 4.2, -0.79, -1.7],
            "corridor_pos1":  [ 8.0,  0.0,  -1.5707],
            "put_down_pos":   [ 8.0, -2.1,  -1.5707],
        }

        self.real_check_positions = {
            "cp_01":             [ 1.1,   0.3,   1.5707],
            # "cp_01":             [-0.5,   1.0,   1.5707],
            "cp_02":             [-1.79, -1.22, 3.14159],
            "cp_03":             [-0.784,-2.44, -1.5707],
            "cp_04":             [ 1.36, -1.42, 0.0],
            "cp_05":             [ 1.36, -3.02, -1.5707],
            "cp_06":             [ 1.36, -1.42, 1.5707],
            "cp_07":             [ 4.3,  -1.31, 0.0],
            "cp_08":             [ 5.09,  0.512, 1.5707],
        }

    def timer_callback(self):
        if self.requested_cmd != None:
            print("Clean Table AS: Requested - {}".format(self.requested_cmd))
            self.requested_cmd = None

    def set_initial_pose(self, navigator, m_p):
        initial_pose = PoseStamped()

        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = m_p[0]
        initial_pose.pose.position.y = m_p[1]

        r = R.from_euler('xyz',[0, 0, m_p[2]])
        initial_pose.pose.orientation.x = r.as_quat()[0]
        initial_pose.pose.orientation.y = r.as_quat()[1]
        initial_pose.pose.orientation.z = r.as_quat()[2]
        initial_pose.pose.orientation.w = r.as_quat()[3]

        navigator.setInitialPose(initial_pose)
        print('Set the Initial position at Home position.')

    def navi_go_through_pose(self, navigator, wp):
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for loc, pt in wp.items():
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))

        print(route_poses)

        navigator.goThroughPoses(route_poses)
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at loading position: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

            rclpy.spin_once(robot_utils)
            rclpy.spin_once(table_finder)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Success to go through poses')
            ret = True
        elif result == TaskResult.CANCELED:
            print('go through poses was canceled')
            ret = False
        elif result == TaskResult.FAILED:
            print('Fail to go through poses')
            ret = False
        return ret


    def navi_goto_pose(self, navigator, m_p, retry=True):
        move_to_pose = PoseStamped()

        move_to_pose.header.frame_id = 'map'
        move_to_pose.header.stamp = navigator.get_clock().now().to_msg()
        move_to_pose.pose.position.x = m_p[0]
        move_to_pose.pose.position.y = m_p[1]

        r = R.from_euler('xyz',[0, 0, m_p[2]])
        move_to_pose.pose.orientation.x = r.as_quat()[0]
        move_to_pose.pose.orientation.y = r.as_quat()[1]
        move_to_pose.pose.orientation.z = r.as_quat()[2]
        move_to_pose.pose.orientation.w = r.as_quat()[3]

        print('Request to move to target position x(%f), y(%f), theta(%f)' 
                % (m_p[0], m_p[1], m_p[2]))

        ret = False
        for n in range(5):
            navigator.goToPose(move_to_pose)
            i = 0
            while not navigator.isTaskComplete():
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival at loading position: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')
                rclpy.spin_once(robot_utils)
                rclpy.spin_once(table_finder)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                ret = True
                break
            elif result == TaskResult.FAILED:
                print('Task at request_item_location, failed on its {} attempt.'.format(n+1))
                if n + 1 == 5 or retry == False:
                    ret = False
                    break 
                time.sleep(1.0)
            else:
                break
        return ret

    def navi_goto_pose_n_check_table(self, navigator, m_p, retry=True):
        move_to_pose = PoseStamped()

        move_to_pose.header.frame_id = 'map'
        move_to_pose.header.stamp = navigator.get_clock().now().to_msg()
        move_to_pose.pose.position.x = m_p[0]
        move_to_pose.pose.position.y = m_p[1]

        r = R.from_euler('xyz',[0, 0, m_p[2]])
        move_to_pose.pose.orientation.x = r.as_quat()[0]
        move_to_pose.pose.orientation.y = r.as_quat()[1]
        move_to_pose.pose.orientation.z = r.as_quat()[2]
        move_to_pose.pose.orientation.w = r.as_quat()[3]

        print('Request to move to target position x(%f), y(%f), theta(%f)' 
                % (m_p[0], m_p[1], m_p[2]))

        ret = False
        for n in range(5):
            navigator.goToPose(move_to_pose)
            i = 0
            while not navigator.isTaskComplete():
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival at loading position: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')
                if table_finder.is_table:
                    print("=%d= Detected table in the way of target position x(%f), y(%f), theta(%f)" 
                    % (i, m_p[0], m_p[1], m_p[2]))
                rclpy.spin_once(robot_utils)
                rclpy.spin_once(table_finder)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Arrived at the target position x(%f), y(%f), theta(%f) and Check the table exist' % (m_p[0], m_p[1], m_p[2]))
                ret = True
                break
            elif result == TaskResult.CANCELED:
                print('To move to target position was canceled and Check the table exist')
                ret = False
                break
            elif result == TaskResult.FAILED:
                print('Task at request_item_location, failed on its {} attempt, but check the table exist'.format(n+1))
                if n + 1 == 5 or retry == False: 
                    ret = False
                    break
                    # exit(-1)
                time.sleep(1.0)
            else:
                break

        table_finder.check_if_table()
        if table_finder.is_table == True:
            print("Found the table ! ")
        else:
            print("Not Found the table... ")
        print("table status ({})".format(table_finder.table_status))

        return table_finder.is_table

    def find_trash_table(self):
        print("Finding table - rotate right 60'")
        self.mover_rotate_robot("right", 70)
        time.sleep(1)
        table_finder.is_table = False
        ret = False
        cnt = 20
        while cnt > 0:
            robot_utils.rb_rotate("left", 0.3, 0.0)
            if cnt % 5 == 0:
                print("Finding table ... {}".format(cnt))

            if table_finder.is_table and table_finder.table_pos == "front":
                ret = True
                break
            rclpy.spin_once(robot_utils)
            rclpy.spin_once(table_finder)
            time.sleep(0.1)
            cnt -= 1

        robot_utils.rb_stop()
        return ret 

    def mover_rotate_robot(self, direction, deg=90):
        dt_const = 0
        turn_cnt = int(deg / 30 * 10) + dt_const    
        w = 0.19           
        while turn_cnt > 0:
            robot_utils.rb_rotate(direction, w=w, x=0.0)
            if turn_cnt % 5 == 0:
                print("Rotating {} degree...count {}".format(deg, turn_cnt))
            rclpy.spin_once(robot_utils)
            rclpy.spin_once(table_finder)
            time.sleep(0.1)        
            turn_cnt -= 1

    def mover_align_to_front_leg(self):
        ret = table_finder.align_to_front_leg()
        return ret

    def mover_elevator_up(self):
        cmd_cnt = 20    # 4 second 
        robot_utils.rb_elevator_up()
        print("Elevator up {}".format(cmd_cnt))
        while cmd_cnt > 0:
            rclpy.spin_once(robot_utils)
            time.sleep(0.1)        
            cmd_cnt -= 1
        robot_utils.attached_table_stat = 1

    def mover_elevator_down(self):
        cmd_cnt = 20    # 4 second   
        robot_utils.rb_elevator_down()
        print("Elevator down {}".format(cmd_cnt))
        while cmd_cnt > 0:
            rclpy.spin_once(robot_utils)
            time.sleep(0.1)        
            cmd_cnt -= 1
        robot_utils.attached_table_stat = 2

    def mover_init_robot(self):
        robot_utils.pub_fprint_cnt = 1
        self.mover_elevator_down()

    def send_feedback_msg(self, goal_handle, msg):
        feedback_msg = CleanTable.Feedback() 
        feedback_msg.phase = msg
        goal_handle.publish_feedback(feedback_msg)

    def cleaner_move_table_1(self, goal_handle):
        find_fg = False
        job_stat = False
        self.mover_init_robot()
        for loc, pos in self.table1_wp.items():
            print("> Go to the {}...".format(loc))
            self.send_feedback_msg(goal_handle, "move robot to "+ loc)
            if loc == 'loading_pos':
                ret = self.navi_goto_pose_n_check_table(navigator, pos)
                if ret:
                    print('Found the table in front at the {} !'.format(loc))
                    find_fg = True
                else:
                    print('Not found the table at the {}, rotating around to find the table...'.format(loc))
                    if self.find_trash_table():
                        print('Found the table around of robot!')
                        find_fg = True
                    else:
                        print('Not found the table around of robot!')
                        job_stat = True
                        break

                if find_fg:
                    print("Send the goal to 'Approch to trash table action server'!")
                    self.send_feedback_msg(goal_handle, "Call AS to approach under the trash table")
                    robot_utils.send_goal(search=False)
                    robot_utils.robot_state = "moving-under_table"
                    while robot_utils.robot_state != "trash-table_search_finished":
                        rclpy.spin_once(robot_utils)
                        rclpy.spin_once(table_finder)

                    find_fg = False   
                    if robot_utils.robot_state == "trash-table_search_finished":
                        print("-- Found trash table and Complete to go under table --")
                        find_fg = True

                        self.send_feedback_msg(goal_handle, "Rotate robot and lift the table")

                        print("Rotate 90 degree left for going forward")
                        self.mover_rotate_robot("left", 90)

                        print("Align to the table leg for lift")
                        if self.mover_align_to_front_leg() == False:
                            print("-- Fail to align robot to the leg")
                        time.sleep(1)

                        print("Lift the trash table")
                        self.mover_elevator_up()

                        print("Start to publish new footprint for the loaded robot shape")
                        robot_utils.pub_fprint_cnt = 1
                        while robot_utils.pub_fprint_cnt > 0:
                            rclpy.spin_once(robot_utils)
                            rclpy.spin_once(table_finder)

                        self.send_feedback_msg(goal_handle, "Publish loaded footprint and go out from the loading position")

                        print("Moving forward to get out of loading position...")
                        robot_utils.rb_move_out(direction='forward', d_rotate='left')

                        print("Rotate 110 degree right for going to put down position")
                        self.mover_rotate_robot("right", 145)

            elif loc == 'put_down_pos':
                ret = self.navi_goto_pose(navigator, pos)
                if not ret:
                    print('Fail to move to the {}...'.format(loc))
                else:
                    self.send_feedback_msg(goal_handle, "put down the table and going out from underneath table")
                    print('Arrived at the {} !'.format(loc))
                    print("Drop the trash table")
                    self.mover_elevator_down()

                    print("Start to publish new footprint for the unloaded robot shape")
                    robot_utils.pub_fprint_cnt = 1
                    while robot_utils.pub_fprint_cnt > 0:
                        rclpy.spin_once(robot_utils)
                        rclpy.spin_once(table_finder)

                    if self.mover_align_to_front_leg() == False:
                        print("-- Fail to align robot to the leg")

                    time.sleep(1)

                    print("Rotate 180 degree right for going to put down position")
                    # self.mover_rotate_robot("right", 180)
                    self.mover_rotate_robot("right", 210)

                    print("Align to the table leg for getting out")
                    if self.mover_align_to_front_leg() == False:
                        print("-- Fail to align robot to the leg")
    
                    time.sleep(1)

                    print("Moving forwardward to get out of put down position...")
                    robot_utils.rb_move_out(direction='forward', d_rotate='forward', t=4)
                    job_stat = True
            else:
                # self.send_feedback_msg(goal_handle, "move robot to " + loc)
                print("Moving forward to the {}...".format(loc))
                ret = self.navi_goto_pose(navigator, pos)
                if ret:
                    print('Arrived at the {} !'.format(loc))
                else:
                    print('Fail to move to the {}...'.format(loc))
        if job_stat:
            self.send_feedback_msg(goal_handle, "going back to home position")
            print(">> Going back to home...")
            ret = self.navi_goto_pose(navigator, self.move_positions['home_position'])
            if ret:
                print('Arrived at home position !')
            else:
                print('Fail to go back home position ...')
        return job_stat

    def cleaner_move_table_2(self, goal_handle):
        find_fg = False
        job_stat = False
        self.mover_init_robot()
        for loc, pos in self.table2_wp.items():
            print("> Go to the {}...".format(loc))
            self.send_feedback_msg(goal_handle, "move robot to "+ loc)
            if loc == 'loading_pos':
                ret = self.navi_goto_pose_n_check_table(navigator, pos)
                if ret:
                    print('Found the table in front at the {} !'.format(loc))
                    find_fg = True
                else:
                    print('Not found the table at the {}, rotating around to find the table...'.format(loc))
                    if self.find_trash_table():
                        print('Found the table around of robot!')
                        find_fg = True
                    else:
                        print('Not found the table around of robot!')
                        job_stat = True
                        break

                if find_fg:
                    print("Send the goal to 'Approch to trash table action server'!")
                    self.send_feedback_msg(goal_handle, "Call AS to approach under the trash table")
                    robot_utils.send_goal(search=False)
                    robot_utils.robot_state = "moving-under_table"
                    while robot_utils.robot_state != "trash-table_search_finished":
                        rclpy.spin_once(robot_utils)
                        rclpy.spin_once(table_finder)

                    find_fg = False   
                    if robot_utils.robot_state == "trash-table_search_finished":
                        print("-- Found trash table and Complete to go under table --")
                        find_fg = True

                        self.send_feedback_msg(goal_handle, "Rotate robot and lift the table")

                        print("Rotate 180 degree left for going forward")
                        self.mover_rotate_robot("right", 180)

                        print("Align to the table leg for lift")
                        if self.mover_align_to_front_leg() == False:
                            print("-- Fail to align robot to the leg")
                        
                        print("Lift the trash table")
                        self.mover_elevator_up()

                        print("Start to publish new footprint for the loaded robot shape")
                        robot_utils.pub_fprint_cnt = 1
                        while robot_utils.pub_fprint_cnt > 0:
                            rclpy.spin_once(robot_utils)
                            rclpy.spin_once(table_finder)
                        
                        self.send_feedback_msg(goal_handle, "Publish loaded footprint and go out from the loading position")

                        print("Moving forward to get out of loading position...")
                        robot_utils.rb_move_out(direction='forward', d_rotate='right')

            elif loc == 'put_down_pos':
                ret = self.navi_goto_pose(navigator, pos)
                if not ret:
                    print('Fail to move to the {}...'.format(loc))
                else:
                    self.send_feedback_msg(goal_handle, "put down the table and going out from underneath table")
                    print('Arrived at the {} !'.format(loc))
                    print("Drop the trash table")
                    self.mover_elevator_down()

                    print("Start to publish new footprint for the unloaded robot shape")
                    robot_utils.pub_fprint_cnt = 1
                    while robot_utils.pub_fprint_cnt > 0:
                        rclpy.spin_once(robot_utils)
                        rclpy.spin_once(table_finder)

                    if self.mover_align_to_front_leg() == False:
                        print("-- Fail to align robot to the leg")

                    print("Rotate 90 degree left for going to put down position")
                    self.mover_rotate_robot("left", 90)

                    print("Align to the table leg for getting out")
                    if self.mover_align_to_front_leg() == False:
                        print("-- Fail to align robot to the leg")
    
                    print("Moving forwardward to get out of put down position...")
                    robot_utils.rb_move_out(direction='forward', d_rotate='forward', t=4)
                    job_stat = True
            else:
                print("Moving forward to the {}...".format(loc))
                ret = self.navi_goto_pose(navigator, pos)
                if ret:
                    print('Arrived at the {} !'.format(loc))
                else:
                    print('Fail to move to the {}...'.format(loc))
        if job_stat:
            self.send_feedback_msg(goal_handle, "going back to home position")
            print(">> Going back to home...")
            ret = self.navi_goto_pose(navigator, self.move_positions['home_position'])
            if ret:
                print('Arrived at home position !')
            else:
                print('Fail to go back home position ...')
        return job_stat

###-------------------------
    def cleaner_move_table_2_2(self, goal_handle):
        find_fg = False
        job_stat = False
        self.mover_init_robot()

        # pos = self.table2_wp["loading_pos"]
        print("> Go to the {}...".format("loading_pos"))
        self.send_feedback_msg(goal_handle, "move robot to loading_pos")

        ret = self.navi_goto_pose_n_check_table(navigator, self.table2_wp["loading_pos"])
        if ret:
            print('Found the table in front at the loading_pos !')
            find_fg = True
        else:
            print('Not found the table at the loading_pos, rotating around to find the table...')
            if self.find_trash_table():
                print('Found the table around of robot!')
                find_fg = True
            else:
                print('Not found the table around of robot!')
                job_stat = True

        if find_fg:
            print("Send the goal to 'Approch to trash table action server'!")
            self.send_feedback_msg(goal_handle, "Call AS to approach under the trash table")
            robot_utils.send_goal(search=False)
            robot_utils.robot_state = "moving-under_table"
            while robot_utils.robot_state != "trash-table_search_finished":
                rclpy.spin_once(robot_utils)
                rclpy.spin_once(table_finder)

            find_fg = False   
            if robot_utils.robot_state == "trash-table_search_finished":
                print("-- Found trash table and Complete to go under table --")
                find_fg = True

                self.send_feedback_msg(goal_handle, "Rotate robot and lift the table")

                print("Rotate 180 degree left for going forward")
                self.mover_rotate_robot("right", 180)

                print("Align to the table leg for lift")
                if self.mover_align_to_front_leg() == False:
                    print("-- Fail to align robot to the leg")
                
                print("Lift the trash table")
                self.mover_elevator_up()

                print("Start to publish new footprint for the loaded robot shape")
                robot_utils.pub_fprint_cnt = 1
                while robot_utils.pub_fprint_cnt > 0:
                    rclpy.spin_once(robot_utils)
                    rclpy.spin_once(table_finder)
                
                self.send_feedback_msg(goal_handle, "Publish loaded footprint and go out from the loading position")

                print("Moving forward to get out of loading position...")
                robot_utils.rb_move_out(direction='forward', d_rotate='right')
        else:
            return False
        
        if self.navi_go_through_pose(navigator, self.table2_cor_wp) == False:
            return False

        if self.navi_goto_pose(navigator, self.table2_wp["put_down_pos"]):
            self.send_feedback_msg(goal_handle, "put down the table and going out from underneath table")
            print('Arrived at the put_down_pos')
            print("Drop the trash table")
            self.mover_elevator_down()

            print("Start to publish new footprint for the unloaded robot shape")
            robot_utils.pub_fprint_cnt = 1
            while robot_utils.pub_fprint_cnt > 0:
                rclpy.spin_once(robot_utils)
                rclpy.spin_once(table_finder)

            if self.mover_align_to_front_leg() == False:
                print("-- Fail to align robot to the leg")

            print("Rotate 90 degree left for going to put down position")
            self.mover_rotate_robot("left", 90)

            print("Align to the table leg for getting out")
            if self.mover_align_to_front_leg() == False:
                print("-- Fail to align robot to the leg")

            print("Moving forwardward to get out of put down position...")
            robot_utils.rb_move_out(direction='forward', d_rotate='forward', t=4)
            job_stat = True
        else:
            print('Fail to move to the put_down_pos')
            job_stat = False

        if job_stat:
            self.send_feedback_msg(goal_handle, "going back to home position")
            print(">> Going back to home...")
            ret = self.navi_goto_pose(navigator, self.move_positions['home_position'])
            if ret:
                print('Arrived at home position !')
            else:
                print('Fail to go back home position ...')
        return job_stat
###-------------------------

    def discovery_and_cleaner_move(self, goal_handle):
        self.send_feedback_msg(goal_handle, "discovering the trash table in the cafe")
        find_fg = False
        job_stat = False
        self.mover_init_robot()
        kl = sorted(self.check_positions.keys())
        for idx, cp in enumerate(kl):
            loc = cp
            pos = self.check_positions[loc]

            print("> Go to check point {} and finding table...".format(loc))
            self.send_feedback_msg(goal_handle, "move robot to check point "+ loc)

            ret = self.navi_goto_pose_n_check_table(navigator, pos, False)
            if ret:
                print('Found the table in front at the {} !'.format(loc))
                find_fg = True
            else:
                print('Arrived at the {}, rotating around to find the table...'.format(loc))
                if self.find_trash_table():
                    print('Found the table around of robot!')
                    find_fg = True
                else:
                    print('Not found the table around of robot!')
                    job_stat = True
                    continue

            if find_fg:
                print("Send the goal to 'Approch to trash table action server'!")
                self.send_feedback_msg(goal_handle, "Find the table and Call AS to approach under the trash table")

                robot_utils.send_goal(search=False)
                robot_utils.robot_state = "moving-under_table"
                
                while robot_utils.robot_state != "trash-table_search_finished":
                    rclpy.spin_once(robot_utils)
                    rclpy.spin_once(table_finder)

                print("-- Found trash table and Complete to go under table --")
                self.send_feedback_msg(goal_handle, "Rotate robot and lift the table")

                if idx in [0, 1, 2, 4, 7]:
                    print("Rotate 180 degree left for going forward")
                    self.mover_rotate_robot("right", 180)
                elif idx == 6:
                    print("Rotate 90 degree left for going forward")
                    self.mover_rotate_robot("left", 90)

                print("Align to the table leg for lift")
                if self.mover_align_to_front_leg() == False:
                    print("-- Fail to align robot to the leg")
                
                print("Lift the trash table")
                self.mover_elevator_up()

                print("Start to publish new footprint for the loaded robot shape")
                robot_utils.pub_fprint_cnt = 1
                while robot_utils.pub_fprint_cnt > 0:
                    rclpy.spin_once(robot_utils)
                    rclpy.spin_once(table_finder)
                
                self.send_feedback_msg(goal_handle, "Publish loaded footprint and go out from the loading position")

                print("Moving forward to get out of loading position...")
                if idx == 0:
                    robot_utils.rb_move_out(direction='forward', d_rotate='right')
                elif idx in [1, 2, 3, 4]:
                    robot_utils.rb_move_out(direction='forward', d_rotate='forward')
                elif idx == 6:
                    robot_utils.rb_move_out(direction='forward', d_rotate='left')
                    print("Rotate 110 degree right for going to put down position")
                    self.mover_rotate_robot("right", 110)
                elif idx == 7:
                    robot_utils.rb_move_out(direction='forward', d_rotate='right')
                    print("Rotate 110 degree right for going to put down position")
                    self.mover_rotate_robot("left", 110)
                break

        if find_fg:
            wl = sorted(self.discovery_wp.keys())
            for widx, wp in enumerate(wl):
                wloc = wp
                pos = self.discovery_wp[wloc]
                if idx > 2 and widx < 1:
                    continue
                elif idx in [6, 7] and widx < 4:
                    continue

                print("Moving forward to the {}...".format(wloc))
                self.send_feedback_msg(goal_handle, "move robot through waypoint "+ wloc)

                ret = self.navi_goto_pose(navigator, pos)
                if not ret:
                    print('Fail to move to the {}...'.format(wloc))
                else:
                    if wloc == "put_down_pos":
                        self.send_feedback_msg(goal_handle, "put down the table and going out from underneath table")
                        print('Arrived at the {} !'.format(wloc))
                        print("Drop the trash table")
                        self.mover_elevator_down()

                        print("Start to publish new footprint for the unloaded robot shape")
                        robot_utils.pub_fprint_cnt = 1
                        while robot_utils.pub_fprint_cnt > 0:
                            rclpy.spin_once(robot_utils)
                            rclpy.spin_once(table_finder)

                        if self.mover_align_to_front_leg() == False:
                            print("-- Fail to align robot to the leg")

                        print("Rotate 90 degree left for going to put down position")
                        self.mover_rotate_robot("left", 90)

                        print("Align to the table leg for getting out")
                        if self.mover_align_to_front_leg() == False:
                            print("-- Fail to align robot to the leg")
        
                        print("Moving forwardward to get out of put down position...")
                        robot_utils.rb_move_out(direction='forward', d_rotate='forward', t=4)
                        job_stat = True
                    else:
                        print('Arrived at the {} !'.format(wloc))
        else:
            print("Not found the trash table anywhere")

        if job_stat:
            self.send_feedback_msg(goal_handle, "Done!! and going back to home position")
        print(">> Going back to home...")
        ret = self.navi_goto_pose(navigator, self.move_positions['home_position'])
        if ret:
            print('Arrived at home position !')
        else:
            print('Fail to go back home position ...')
        return job_stat

    def go_home_position(self, goal_handle):
        self.send_feedback_msg(goal_handle, "going to home position")
        self.mover_init_robot()
        print(">> Going back to home...")
        ret = self.navi_goto_pose(navigator, self.move_positions['home_position'])
        if ret:
            print('Arrived at home position !')
        else:
            print('Fail to go back home position ...')
        return ret

    def as_clean_table(self,goal_handle):

        self.robot_stat = "approaching_table"
        feedback_msg = CleanTable.Feedback()        
        result = CleanTable.Result()

        self.requested_cmd = goal_handle.request.cmd

        feedback_msg.phase = "processing : " + goal_handle.request.cmd
        goal_handle.publish_feedback(feedback_msg)
        
        ret = True
        # We do the Three fases: Approach, Go under table and align CENTER
        if goal_handle.request.cmd == "clean_table1":
            ret = self.cleaner_move_table_1(goal_handle)
        elif goal_handle.request.cmd == "clean_table2":
            ret = self.cleaner_move_table_2(goal_handle)
        elif goal_handle.request.cmd == "clean_table22":
            ret = self.cleaner_move_table_2_2(goal_handle)
        elif goal_handle.request.cmd == "discover_clean":
            ret = self.discovery_and_cleaner_move(goal_handle)
        elif goal_handle.request.cmd == "go_home":
            ret = self.go_home_position(goal_handle)
        else:
            feedback_msg.phase = "unknown_command"
            goal_handle.publish_feedback(feedback_msg)
            result.complete = False
            return result
    
        feedback_msg.phase = "finished action request : " + goal_handle.request.cmd
        goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info("Complete Action Request")

        if ret:
            goal_handle.succeed()
            result.complete = True
        else:
            result.complete = False
        self.robot_stat = "ready_to_action"
        self.requested_cmd = None
        return result 

#---------------------------

# parser = argparse.ArgumentParser()
# parser.add_argument('-target', default='sim', help='Please set the Target Env(sim or real)') 
# parser.add_argument('-robot_name', default='rb1_robot', help='Please set the Target robot name for sim or real') 
# args = parser.parse_args()

def main(args=None):
    rclpy.init(args=args)

    global robot_utils, navigator, table_finder, clean_table_as
    global table1_wp, table2_wp, move_positions

    # robot_utils = RobotUtilsNode(args.target, args.robot_name)
    # table_finder = TrashTableFinder(args.robot_name)
    # clean_table_as = CleanTrashTableAS(args.target, args.robot_name) 

    navigator = BasicNavigator()
    robot_utils = RobotUtilsNode()
    table_finder = TrashTableFinder()
    clean_table_as = CleanTrashTableAS() 


    executor = MultiThreadedExecutor()
    executor.add_node(robot_utils)
    executor.add_node(navigator)
    executor.add_node(table_finder)
    executor.add_node(clean_table_as)

    try:
        executor.spin()

    finally:
        executor.shutdown()
        robot_utils.destroy_node()
        navigator.destroy_node()
        table_finder.destroy_node()

    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()