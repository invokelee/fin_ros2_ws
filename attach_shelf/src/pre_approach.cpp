#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {
    auto p1_obstacle_desc = rcl_interfaces::msg::ParameterDescriptor{};
    auto p2_degrees_desc = rcl_interfaces::msg::ParameterDescriptor{};
    p1_obstacle_desc.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    p2_degrees_desc.description =
        "Number of degrees for the rotation of the robot after stopping.";
    this->declare_parameter<double>("obstacle", 0.5, p1_obstacle_desc);
    this->declare_parameter<int>("degrees", 90, p2_degrees_desc);

    robot_stat_ = 0;

    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::timer_callback, this));

    cmd_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&PreApproach::scan_callback, this, _1));
  }

  void timer_callback() {
    static int t_cnt;
    auto move = geometry_msgs::msg::Twist();

    if (robot_stat_ == 0)
      return;
    if (robot_stat_ == 1) { // get obstacle distance and go to forward
      this->get_parameter("obstacle", obstacle_dist_);
      move.linear.x = 0.5;
      move.angular.z = 0.0;
      robot_stat_++;
      RCLCPP_INFO(this->get_logger(), "Get obstacle param : %f",
                  obstacle_dist_);
      t_cnt = 0;
    } else if (robot_stat_ == 2) {
      move.linear.x = 0.5;
      move.angular.z = 0.0;
      if (t_cnt++ % 10 == 0)
        RCLCPP_INFO(this->get_logger(), "Go forward until to meet the wall...");
    } else if (robot_stat_ == 3) { // get turn degrees param
      this->get_parameter("degrees", turn_degree_);
      RCLCPP_INFO(this->get_logger(), "Get degrees param : %d", turn_degree_);

      turn_degree_ = turn_degree_ % 360;
      if (turn_degree_ >= 0) { // convert degree to radian
        if (turn_degree_ <= 180)
          turn_radian_ = turn_degree_ * 0.0174533;
        else {
          turn_radian_ = (360 - turn_degree_) * -0.0174533;
          turn_dir_ = -1.0;
        }
      } else {
        if (turn_degree_ >= -180)
          turn_radian_ = turn_degree_ * 0.0174533;
        else
          turn_radian_ = (360 + turn_radian_) * 0.0174533;
      }
      turn_dir_ = 1.0;
      if (turn_radian_ < 0)
        turn_dir_ = -1.0;

      robot_stat_ = 4;
      move.linear.x = 0.0;
      move.angular.z = 0.523567 * turn_dir_;

      // target / 30 (degree/sec) * 10(1s / 100ms)
      t_cnt = (abs(turn_degree_) / 30) * 10 + 10;

      RCLCPP_INFO(this->get_logger(),
                  "Got the target position and to get turn degree %d(r:%f)",
                  turn_degree_, turn_radian_);
    } else if (robot_stat_ == 4) { // Turning
      if (--t_cnt <= 0)
        robot_stat_++;
      move.linear.x = 0.0;
      move.angular.z = 0.523567 * turn_dir_; // 30 degree / sec
      if (t_cnt % 5 == 0)
        RCLCPP_INFO(this->get_logger(), "Turning...");
    } else if (robot_stat_ == 5) {
      RCLCPP_INFO(this->get_logger(),
                  "Turn to the target degrees, completed the "
                  "mission for Task1 Pre-Approach");
      move.linear.x = 0.0;
      move.angular.z = 0.0;
      robot_stat_++;
    } else { // Default stop the robot
      move.linear.x = 0.0;
      move.angular.z = 0.0;
    }

    cmd_publisher_->publish(move);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  int robot_stat_;
  double obstacle_dist_;
  int turn_degree_;
  double turn_radian_, turn_dir_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (robot_stat_ == 0) {
      robot_stat_ = 1;
      RCLCPP_INFO(this->get_logger(),
                  "angle_min: %f, angle_max: %f, angle_inc: %f, ranges size: "
                  "%ld, intesities size: %ld",
                  msg->angle_min, msg->angle_max, msg->angle_increment,
                  msg->ranges.size(), msg->intensities.size());
    } else if (robot_stat_ == 2) {
      // Check the front obstacle : +- 10 degree(20)
      int F_L = 0;
      int F_R = 0;
      int front = 540; // size: 1081, -2.3562 ~ 2.3562
      int f_range = 5;
      for (int i = front - f_range; i <= front; i++) {
        if (msg->ranges[i] < obstacle_dist_) {
          F_L++;
          RCLCPP_INFO(this->get_logger(), "F_L Dist : %d, %f ", i,
                      msg->ranges[i]);
        }
      }
      for (int i = front + 1; i < front + f_range; i++) {
        if (msg->ranges[i] < obstacle_dist_) {
          F_R++;
          RCLCPP_INFO(this->get_logger(), "F_R Dist : %d, %f ", i,
                      msg->ranges[i]);
        }
      }
      std::cout << std::endl;
      if (F_L > 0 || F_R > 0) {
        robot_stat_ = 3;
        RCLCPP_INFO(this->get_logger(), "F_L : %d, F_R : %d", F_L, F_R);
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}