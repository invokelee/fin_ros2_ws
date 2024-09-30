// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

std::string Send_msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    cb_grp_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_grp_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subs_options;
    subs_options.callback_group = cb_grp_1;
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MinimalSubscriber::odom_topic_callback, this, _1),
        subs_options);
    RCLCPP_INFO(this->get_logger(), "Subscriber for '/odom' created");

    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&MinimalSubscriber::scan_callback, this, _1),
            subs_options);
    RCLCPP_INFO(this->get_logger(), "Subscriber for '/scan' created");

    timer_ = this->create_wall_timer(
        100ms, std::bind(&MinimalSubscriber::timer_callback, this), cb_grp_2);
    RCLCPP_INFO(this->get_logger(), "timer for checking, created");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  rclcpp::CallbackGroup::SharedPtr cb_grp_1;
  rclcpp::CallbackGroup::SharedPtr cb_grp_2;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  sensor_msgs::msg::LaserScan ls_msg;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  nav_msgs::msg::Odometry odom_msg;

  bool sensor_rd_fg_ = false;
  bool tf_publish_fg_ = false;

  void timer_callback() {
    std::string fromF = "Start";
    std::string toF = "Target";
    static int tcnt = 0;
    char buf[100];

    if (tf_publish_fg_ == true) {
      auto stamp = this->get_clock()->now();
      sprintf(buf, "%lf", stamp.seconds());
      if (tcnt++ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "%s : In timer_callback : Published tf %s -> %s", buf,
                    toF.c_str(), fromF.c_str());
      }
      Send_msg = std::string(buf) + std::string(" : ") + fromF + " -> " + toF +
                 std::to_string(tcnt) + " : ";
    } else {
      if (tcnt++ % 100 == 0)
        Send_msg = "From_master : " + std::to_string(tcnt) + " : ";
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (sensor_rd_fg_ == false) {
      sensor_rd_fg_ = 1;
      RCLCPP_INFO(this->get_logger(),
                  "angle_min: %f, angle_max: %f, angle_inc: %f, ranges size: "
                  "%ld, intesities size: %ld",
                  msg->angle_min, msg->angle_max, msg->angle_increment,
                  msg->ranges.size(), msg->intensities.size());
      tf_publish_fg_ = true;
    }
    ls_msg = *msg;
  }

  void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_msg = *msg;

    // current_pos_.x = msg->pose.pose.position.x;
    // current_pos_.y = msg->pose.pose.position.y;

    // tf2::Quaternion q(
    //     msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    //     msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    // current_pos_.theta = yaw;
    // if (yaw < 0.0) {
    //   curr_inv_theta = 1.57079632679 + yaw;
    // } else {
    //   curr_inv_theta = yaw - 1.57079632679;
    // }
  }

  void topic_callback(const std_msgs::msg::String &msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
};

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher_inside"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = Send_msg + "Hello, world";
    // message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  //   rclcpp::spin(std::make_shared<MinimalSubscriber>());

  auto app_svc_svr_node = std::make_shared<MinimalSubscriber>();
  RCLCPP_INFO(app_svc_svr_node->get_logger(),
              "approach_service_server_node INFO...");

  auto pub_svc_node = std::make_shared<MinimalPublisher>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pub_svc_node);
  executor.add_node(app_svc_svr_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
