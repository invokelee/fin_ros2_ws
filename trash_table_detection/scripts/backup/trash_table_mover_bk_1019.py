#! /usr/bin/env python3

import time
import argparse, sys
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
from rclpy.action import ActionClient
from table_find_interface.action import GoUnderTable

from trash_table_detection.trash_table_finder import TrashTableFinder

class RobotUtilsNode(Node):

    def __init__(self, env_, name_):
        super().__init__('robot_utils_node')

        self.cli = self.create_client(GoToLoading, '/approach_shelf')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.req = GoToLoading.Request()
    
        if env_ == 'sim':
            self.target_env_ = 'sim'
        else:
            self.target_env_ = 'real'
        print('Target robot running env is ' + self.target_env_)
        # self.get_parameters()
        self.robot_name = name_
        self.get_logger().info('Target robot running env is %s' %(self.target_env_))
        if self.target_env_ == 'sim':
            self.cmd_vel_pub_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        else:
            self.cmd_vel_pub_ = self.create_publisher(Twist, '/cleaner_2/cmd_vel', 10)

        self.group_ac = MutuallyExclusiveCallbackGroup()
        self.table_finder_phase = "nothing"
        self.robot_state = "iddle"
        if env_ == 'sim':
            self.ac_name = "/go_under_table"
        else:
            self.ac_name = "/"+self.robot_name + "/go_under_table"
        self._table_finder_ac = ActionClient(self, GoUnderTable, self.ac_name, callback_group=self.group_ac)

        self.elevator_up_ = self.create_publisher(String, '/elevator_up', 1)
        self.elevator_down_ = self.create_publisher(String, '/elevator_down', 1)
        self.loc_fprint_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.glb_fprint_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.attached_table_stat = 0
        self.pub_fprint_cnt = 0
        self.find_rb_cnt = 0
        self.rb_direction = None
        self.loaded_robot_shape = Polygon()
        self.loaded_robot_shape.points = [
            Point32(x=0.50,  y=0.45,  z=0.0),
            Point32(x=0.50,  y=-0.45, z=0.0),
            Point32(x=-0.50, y=-0.45, z=0.0),
            Point32(x=-0.50, y=0.45,  z=0.0)
        ] 
        self.normal_robot_shape = Polygon()
        # self.normal_robot_shape.points = [
        #     Point32(x=0.25,  y=0.25,  z=0.0),
        #     Point32(x=0.25,  y=-0.25, z=0.0),
        #     Point32(x=-0.25, y=-0.25, z=0.0),
        #     Point32(x=-0.25, y=0.25,  z=0.0)
        # ] 
        self.normal_robot_shape.points = [
            Point32(x=0.3,  y=0.3,  z=0.0),
            Point32(x=0.3,  y=-0.3, z=0.0),
            Point32(x=-0.3, y=-0.3, z=0.0),
            Point32(x=-0.3, y=0.3,  z=0.0)
        ] 
        self.elev_msg = String()
        self.shelf_down_cnt = 0
        self.move_robot_cnt = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.create_timer(1.0, self.timer_callback, callback_group=timer_cb_group)
        self._loop_rate = self.create_rate(2.0, self.get_clock()) # 2Hz

    # def get_parameters(self):
    #     self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

    def send_attach_shelf_request(self):
        self.req.attach_to_shelf = True;
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def timer_callback(self):
        if self.find_rb_cnt > 0:
            self.find_rb_cnt -= 1
            if self.find_rb_cnt <= 0:
                self.rb_stop()
            else:
                self.rb_rotate(self.rb_direction, 0.5, 0.0)

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
            if self.attached_table_stat == 1:
                self.loc_fprint_.publish(self.loaded_robot_shape)
                self.glb_fprint_.publish(self.loaded_robot_shape)
            elif self.attached_table_stat == 2:
                self.loc_fprint_.publish(self.normal_robot_shape)
                self.glb_fprint_.publish(self.normal_robot_shape)
            self.pub_fprint_cnt -= 1

    # AC stuff
    def send_goal(self, search=True):
        """
        If search=True, means that we wantteh robot to search for the table
        if False, then it will triguer the go out of the table
        """
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
    ###################################################



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

    def rb_move_out(self, direction='forward'):
        # Start time
        start_time = time.time()
        tm_duration = 6

        # Publish a message to move the robot backwards
        msg = Twist()
        if direction == 'backward':
            msg.linear.x = -0.25      # Move backwards
            mesg = 'Moving the robot backwards'
        else:
            msg.linear.x = 0.25      # Move forwards
            mesg = 'Moving the robot forwards'

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


# robot positions for moving
sim_move_positions = {
    "home_position":    [0.0, 0.0, 0.0],
    "loading_pos1":     [ 0.1,   1.29, 0.0],
    "corridor_pos1":    [ 0.5,  -0.9, 0.0],
    "corridor_pos2":    [ 4.74,  -0.29, 0.0],
    "loading_pos2":     [ 4.3,  -1.31, 0.0],
    "put_down_pos1":    [ 7.22,  0.926, 0.0],
    "put_down_pos2":    [ 7.22, -1.926, 0.0],
}
sim_loading_pos = {
    "loading_pos1":     [ 0.1,   1.45, 0.0],
    # "loading_pos1":     [ 0.1,   1.29, 0.0],
    # "loading_pos1":     [ -0.386,   1.29, 0.0],
    "loading_pos2":     [ 4.3,  -1.31, 0.0],
}
sim_put_down_pos = {
    "put_down_pos1":    [ 7.22,  0.926, 0.0],
    "put_down_pos2":    [ 7.22, -1.926, 0.0],
}

sim_check_positions = {
    "cp_01":             [-0.386, 1.29, 0.0],
    "cp_02":             [-0.386, 1.29, 0.0],
    "cp_03":             [-1.79, -1.22, 0.0],
    "cp_04":             [-0.784,-2.44, 0.0],
    "cp_05":             [ 1.36, -1.42, 0.0],
    "cp_06":             [ 1.36, -3.02, 0.0],
    "cp_07":             [ 1.36, -1.42, 0.0],
    "cp_08":             [ 4.3,  -1.31, 0.0],
    "cp_09":             [ 5.09,  0.512, 0.0],
}

real_move_positions = {
    "home_position":        [0.018, 0.005, 0.0], # 0.018 0.005 -0.304
    "loading_position":     [4.2, -0.79, -1.7],  # -1.57
    "shipping_position":    [1.9, 0.9, 1.37]    # 1.57
}

parser = argparse.ArgumentParser()
parser.add_argument('-target', default='sim', help='Please set the Target Env(sim or real)') 
parser.add_argument('-robot_name', default='rb1_robot', help='Please set the Target robot name for sim or real') 
args = parser.parse_args()

def set_initial_pose(navigator, m_p):
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

def navi_goto_pose(navigator, m_p):
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
    # move_to_pose.pose.orientation.z = 0.707
    # move_to_pose.pose.orientation.w = 0.707

    print('Request to move to target position x(%f), y(%f), theta(%f)' 
            % (m_p[0], m_p[1], m_p[2]))

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
        return True
    else:
        return False

def navi_goto_pose_n_check_table(navigator, m_p):
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

    elif result == TaskResult.CANCELED:
        print('To move to target position was canceled and Check the table exist')

    elif result == TaskResult.FAILED:
        print('Task at moving to loading position was failed!, but check the table exist')

    table_finder.check_if_table()
    if table_finder.is_table == True:
        print("Found the table ! ")
    else:
        print("Not Found the table... ")
    print("table status ({})".format(table_finder.table_status))

    return table_finder.is_table

def find_trash_table():
    robot_utils.find_rb_cnt = 30
    robot_utils.rb_direction = "left"
    table_finder.is_table = False
    while robot_utils.find_rb_cnt > 0:
        if table_finder.is_table:
            robot_utils.find_rb_cnt = 0
            robot_utils.rb_stop()
            break
        rclpy.spin_once(robot_utils)
        rclpy.spin_once(table_finder)
        time.sleep(0.1)

    # if table_finder.is_table:
    #     print("Robot Found Table")
    # else:
    #     print("Robot Not Found Table")

    return table_finder.is_table 

def mover_rotate_robot(direction, deg=90):
    # turn_cnt = deg / 30 * 10    
    turn_cnt = 30    
    w = 0.17                 # 0.5236 -> 30'
    while turn_cnt > 0:
        robot_utils.rb_rotate(direction, w=w, x=0.0)
        if turn_cnt % 5 == 0:
            print("Rotating {} degree...count {}".format(deg, turn_cnt))
        rclpy.spin_once(robot_utils)
        rclpy.spin_once(table_finder)
        time.sleep(0.1)        
        turn_cnt -= 1

def mover_elevator_up():
    cmd_cnt = 20    # 2 second 
    while cmd_cnt > 0:
        robot_utils.rb_elevator_up()
        if cmd_cnt % 5 == 0:
            print("Elevator up {}".format(cmd_cnt))
        rclpy.spin_once(robot_utils)
        rclpy.spin_once(table_finder)
        time.sleep(0.1)        
        cmd_cnt -= 1
    robot_utils.attached_table_stat = 1

def mover_elevator_down():
    cmd_cnt = 20    # 2 second   
    while cmd_cnt > 0:
        robot_utils.rb_elevator_down()
        if cmd_cnt % 5 == 0:
            print("Elevator down {}".format(cmd_cnt))
        rclpy.spin_once(robot_utils)
        rclpy.spin_once(table_finder)
        time.sleep(0.1)        
        cmd_cnt -= 1
    robot_utils.attached_table_stat = 2

def main(argv, args):
    rclpy.init()

    global robot_utils, navigator, table_finder
    robot_utils = RobotUtilsNode(args.target, args.robot_name)
    navigator = BasicNavigator()
    table_finder = TrashTableFinder(args.robot_name) 

    executor = MultiThreadedExecutor()
    executor.add_node(robot_utils)
    executor.add_node(navigator)
    executor.add_node(table_finder)


    if robot_utils.target_env_ == 'sim':
        move_positions = sim_move_positions
        load_positions = sim_loading_pos
        check_positions = sim_check_positions
        put_positons = sim_put_down_pos
        print('Target Env is Simulation')
    else:
        move_positions = real_move_positions
        print('Target Env is Real robot')

    set_initial_pose(navigator, move_positions['home_position'])
    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Move to the loading position and check if the trash table
    # rate = aligner.create_rate(10)
    print(">> Goto the loading position for the table.")

    # # testing
    # ret = navi_goto_pose(navigator, load_positions["loading_pos1"])
    # input("Press Enter to next...")    
    # ret = navi_goto_pose(navigator, move_positions['home_position'])
    # input("Press Enter to next...")    corridor_pos1

    find_fg = False
    for loc, pos in load_positions.items():
        input("Press Enter to go to the {}...".format(loc))
        ret = navi_goto_pose_n_check_table(navigator, pos)
        if ret:
            print('Found the table in front at the {} !'.format(loc))
            find_fg = True
        else:
            print('Not found the table at the {}, rotating around to find the table...'.format(loc))
            if find_trash_table():
                print('Found the table around of robot!')
                find_fg = True
            else:
                print('Not found the table around of robot!')

        # find_fg = False

        if find_fg:
            print("Send the goal to 'Approch to trash table action server'!")
            robot_utils.send_goal(search=True)
            robot_utils.robot_state = "dispenser-searching_for_table"
            while robot_utils.robot_state != "trash-table_search_finished":
                rclpy.spin_once(robot_utils)
                rclpy.spin_once(table_finder)
                # time.sleep(0.1)
            find_fg = False   
            if robot_utils.robot_state == "trash-table_search_finished":
                print("-- Found trash table and Complete to go under table --")
                find_fg = True

                print("Rotate 90 degree for going forward")
                mover_rotate_robot("right", 90)

                input("Press Enter to next...")

                print("Lift the trash table")
                mover_elevator_up()

                print("Start to publish new footprint for the loaded robot shape")
                robot_utils.pub_fprint_cnt = 3
                while robot_utils.pub_fprint_cnt > 0:
                    rclpy.spin_once(robot_utils)
                    rclpy.spin_once(table_finder)

                print("Moving forward to get out of loading position...")
                robot_utils.rb_move_out(direction='forward')

                print("Moving forward to corridor position 1...")
                ret = navi_goto_pose(navigator, move_positions['corridor_pos1'])
                if ret:
                    print('Arrived at the {} !'.format("corridor_pos1"))
                else:
                    print('Fail to move to the {}...'.format("corridor_pos1"))

                # print("Moving forward to corridor position 2...")
                # ret = navi_goto_pose(navigator, move_positions['corridor_pos2'])
                # if ret:
                #     print('Arrived at the {} !'.format("corridor_pos1"))
                # else:
                #     print('Fail to move to the {}...'.format("corridor_pos1"))

                print("Move the trash table to back room putdown position")
                input("Press Enter to go to the {}...".format(list(put_positons.keys())[1]))
                ret = navi_goto_pose(navigator, put_positons["put_down_pos2"])
                if ret:
                    print('Arrived at the {} !'.format("put_dwon_pos2"))
                else:
                    print('Fail to move to the {}...'.format("put_dwon_pos2"))

                break


    # print(">> Travel to check points for looking for the table.")
    # for loc, pos in check_positions.items():
    #     input("Press Enter to go to the {}...".format(loc))
    #     ret = navi_goto_pose_n_check_table(navigator, pos)
    #     if ret:
    #         print('Found the table at the {} !'.format(loc))
    #     else:
    #         print('Not found the table at the {}, rotating around to find the table...'.format(loc))
    #         if find_trash_table():
    #             print('Found the table around of robot!')
    #         else:
    #             print('Not found the table around of robot!')


    input(">> Press Enter to go home...")

    ret = navi_goto_pose(navigator, move_positions['home_position'])
    if ret:
        print('Placed at home position !')
    else:
        print('Fail to go home position ...')

    input("Press Enter to End...")
    robot_utils.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    argv = sys.argv
    main(argv, args)