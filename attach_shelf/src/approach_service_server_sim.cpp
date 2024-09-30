#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <math.h>
#include <memory>
#include <string>

using ApproachServiceMessage = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

sensor_msgs::msg::LaserScan ls_msg;
nav_msgs::msg::Odometry odom_msg;
geometry_msgs::msg::Pose2D current_pos_;
bool sensor_rd_fg_ = false;

class SubscribeSensorNode : public rclcpp::Node {
public:
  SubscribeSensorNode()
      : Node("approach_service_sub_node",
             rclcpp::NodeOptions().use_intra_process_comms(true)) {
    this->cb_grp_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    this->cb_grp_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions odom_subs_options;
    odom_subs_options.callback_group = this->cb_grp_1;
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&SubscribeSensorNode::odom_topic_callback, this, _1),
        odom_subs_options);
    RCLCPP_INFO(this->get_logger(), "Subscriber for '/odom' created");

    rclcpp::SubscriptionOptions scan_subs_options;
    scan_subs_options.callback_group = this->cb_grp_2;
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&SubscribeSensorNode::scan_callback, this, _1),
            scan_subs_options);
    RCLCPP_INFO(this->get_logger(), "Subscriber for '/scan' created");
  }

private:
  rclcpp::CallbackGroup::SharedPtr cb_grp_1;
  rclcpp::CallbackGroup::SharedPtr cb_grp_2;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (sensor_rd_fg_ == false) {
      sensor_rd_fg_ = 1;
      RCLCPP_INFO(this->get_logger(),
                  "angle_min: %f, angle_max: %f, angle_inc: %f, ranges size: "
                  "%ld, intesities size: %ld",
                  msg->angle_min, msg->angle_max, msg->angle_increment,
                  msg->ranges.size(), msg->intensities.size());
    }
    ls_msg = *msg;
  }

  void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_msg = *msg;
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pos_.theta = yaw;
  }
};

class ApproachServiceServerNode : public rclcpp::Node {
public:
  ApproachServiceServerNode()
      : Node("approach_service_server_node",
             rclcpp::NodeOptions().use_intra_process_comms(true)) {
    cb_grp_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_grp_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    sensor_rd_fg_ = false;
    this->svc_state_ = 0;
    this->srv_ = this->create_service<ApproachServiceMessage>(
        "approach_shelf",
        std::bind(&ApproachServiceServerNode::approach_callback, this, _1, _2),
        rmw_qos_profile_services_default, cb_grp_2);
    RCLCPP_INFO(this->get_logger(), "Service '/approach_shelf' created");

    this->cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);
    RCLCPP_INFO(this->get_logger(),
                "Publisher for topic "
                "'/diffbot_base_controller/cmd_vel_unstamped' created");

    this->elev_up_ =
        this->create_publisher<std_msgs::msg::String>("elevator_up", 1);
    this->elev_down_ =
        this->create_publisher<std_msgs::msg::String>("elevator_down", 1);
    RCLCPP_INFO(
        this->get_logger(),
        "Publishers for topic '/elevator_up' and '/elevator_down' created");

    this->tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(this->get_logger(),
                "TF Broadcaster for "
                "'robot_front_laser_base_link'->'cart_frame' created");

    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // Listen to the buffer of transforms
    this->tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
    RCLCPP_INFO(this->get_logger(), "TF Listener created");

    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&ApproachServiceServerNode::timer_callback, this),
        cb_grp_1);

    RCLCPP_INFO(this->get_logger(), "Arrorach Service Server is ready...");
  }

  void timer_callback() {
    geometry_msgs::msg::TransformStamped t;
    std::string fromF = transf_.child_frame_id;
    std::string toF = transf_.header.frame_id;
    geometry_msgs::msg::Twist twist_msg;
    // static int cnt = 0,
    static int tcnt = 0;

    if (tf_publish_fg_ == true) {
      transf_.header.stamp = this->get_clock()->now();
      if (attached_shelf_fg_ == false) {
        // update current TF against desired pos from current pos of robot
        transf_.transform.translation.y = current_pos_.x - desired_pos_.x;
        transf_.transform.translation.x = current_pos_.y - desired_pos_.y;
        double yaw = desired_pos_.theta - current_pos_.theta;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        transf_.transform.rotation.x = q.getX();
        transf_.transform.rotation.y = q.getY();
        transf_.transform.rotation.z = q.getZ();
        transf_.transform.rotation.w = q.getW();
      }
      if (tcnt++ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "In timer_callback : Published tf %s -> %s", toF.c_str(),
                    fromF.c_str());
      }
      tf_broadcaster_->sendTransform(transf_);
    }
  }

private:
  rclcpp::CallbackGroup::SharedPtr cb_grp_1;
  rclcpp::CallbackGroup::SharedPtr cb_grp_2;

  rclcpp::Service<ApproachServiceMessage>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elev_up_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elev_down_;
  //   rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elev_up_;
  //   rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elev_down_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D dist_pos_;

  double curr_inv_theta;
  const double kp_yaw = 1.0;
  const double kp_distance = 0.5;
  double error_distance, error_yaw;
  const double pi_ = 3.14159265359;
  const double h_pi_ = 1.57079632679;
  bool rb_dir;
  int svc_state_;
  bool tf_publish_fg_ = false;
  bool attached_shelf_fg_ = false;

  // Find shelf legs
  int find_shelf_legs() {
    rclcpp::Rate r(10);
    while (rclcpp::ok()) {
      if (sensor_rd_fg_ == true)
        break;
      r.sleep();
    }
    // svc_state_ = 2;
    RCLCPP_INFO(this->get_logger(),
                "In find_shelf_legs : sensor_rd_fg_ is true");

    int front = 540;
    int left = front - 240;  // 90- 360, 60- 240;
    int right = front + 240; // 90- 360, 60- 240;
    int pos;
    struct obj_ary {
      int st;
      int ed;
    };
    obj_ary i_obj[5];

    int cnt = 0;
    int ret = 0;
    RCLCPP_INFO(this->get_logger(),
                "In find_shelf_legs : intensities array search index: left %d, "
                "front %d, right %d",
                left, front, right);
    pos = left;
    for (cnt = 0; cnt < 2; cnt++) {
      for (; pos < right; pos++) {
        if (ls_msg.intensities[pos] >= 8000.0) {
          i_obj[cnt].st = pos;
          i_obj[cnt].ed = 0;
          break;
        }
      }
      for (; pos < right; pos++) {
        if (ls_msg.intensities[pos] < 8000.0) {
          i_obj[cnt].ed = pos - 1;
          break;
        }
      }
      if (cnt == 0 && (i_obj[cnt].st == 0 || i_obj[cnt].ed == 0)) {
        if (i_obj[cnt].st == 0)
          ret = 0;
        else
          ret = 1;
        return ret;
      }
      if (cnt == 1 && i_obj[cnt].st == 0) {
        ret = 1;
        return ret;
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "In find_shelf_legs : found the legs of the shelf");
    RCLCPP_INFO(this->get_logger(), "First leg - i_obj[0]: st %d, ed %d",
                i_obj[0].st, i_obj[0].ed);
    RCLCPP_INFO(this->get_logger(), "Second leg - i_obj[1]: st %d, ed %d",
                i_obj[1].st, i_obj[1].ed);

    if (i_obj[0].ed > 0 && i_obj[1].st > 0) {
      double deg_o = (i_obj[1].st - i_obj[0].ed) * 0.004363;
      // theta O between 2 legs from the robot
      // gate length = sqrt((a*sin O)^2 + (b-a*cos O)^2)

      // double a = a side length of the triangle
      double a = ls_msg.ranges[i_obj[0].ed];
      // double b = the another side length of the triangle
      double b = ls_msg.ranges[i_obj[1].st];
      // double d = Calculates the width between the two legs.
      double d = sqrt(pow(a * sin(deg_o), 2.0) + pow((b - a * cos(deg_o)), 2));

      double hd = d / 2.0;
      if (d < 0.5) { // distance of shelf's legs is too small for RB1(0.461)
        ret = 2;
        return ret;
      }
      RCLCPP_INFO(
          this->get_logger(),
          "In find_shelf_legs : distance between two legs: %f and deg_o: %f", d,
          deg_o);
      // calculate the distance from center of the legs to RB1
      double dist = sqrt((pow(a, 2) + pow(b, 2) - 2 * pow(hd, 2)) / 2.0);

      // calc theta from front to hd(x, y)
      int idx = (i_obj[1].st - i_obj[0].ed) / 2;
      int idx2 = idx + i_obj[0].ed;

      double theta = (idx2 - front) * 0.004363;
      //   double theta = (front - idx2) * 0.004363;

      RCLCPP_INFO(
          this->get_logger(),
          "In find_shelf_legs : distance from robot to shelf %f, theta: %f",
          dist, theta);
      transf_.header.frame_id = "robot_front_laser_base_link";
      transf_.child_frame_id = "cart_frame";

      double d_x = dist * cos(theta);
      double d_y = dist * sin(theta);
      transf_.transform.translation.x = d_x;
      transf_.transform.translation.y = d_y;

      // rotate 90 degree: delta x -> y, delta y -> x
      desired_pos_.x = current_pos_.x - d_y;
      desired_pos_.y = current_pos_.y - d_x;
      desired_pos_.theta = theta + current_pos_.theta;

      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      //   q.setRPY(0, 0, 0);
      transf_.transform.rotation.x = q.getX();
      transf_.transform.rotation.y = q.getY();
      transf_.transform.rotation.z = q.getZ();
      transf_.transform.rotation.w = q.getW();
      ret = 3;

      RCLCPP_INFO(this->get_logger(),
                  "current pos x(%f), y(%f), theta(%f) - delta x(%f), y(%f)",
                  current_pos_.x, current_pos_.y, current_pos_.theta, d_y, d_x);
      RCLCPP_INFO(this->get_logger(), "desired pos x(%f), y(%f), theta(%f)",
                  desired_pos_.x, desired_pos_.y, desired_pos_.theta);
    }
    return ret;
  }

  void approach_callback(
      const std::shared_ptr<ApproachServiceMessage::Request> request,
      const std::shared_ptr<ApproachServiceMessage::Response> response) {
    int cnt = 0;
    svc_state_ = 1;

    RCLCPP_INFO(this->get_logger(),
                "Calleld '/approach_shelf' Service from Client with "
                "'attach_to_shelf' is '%s'",
                request->attach_to_shelf ? "true" : "false");

    RCLCPP_INFO(this->get_logger(),
                "Find the legs of the shelf using laser intensity values");

    // Find the legs of the shelf using laser intensity values
    int ret = find_shelf_legs();
    svc_state_ = 2;

    if (ret == 3) {
      // then start to publish cart_frame TF
      tf_publish_fg_ = true;
      response->complete = true;
      RCLCPP_INFO(this->get_logger(),
                  "In SVC Callback: found the legs of the shelf : ret: %d and "
                  "Start to publish the TF for 'cart_frame'",
                  ret);

    } else {
      response->complete = false;
      RCLCPP_INFO(this->get_logger(),
                  "In SVC Callback: not found the legs of the shelf : ret: %d",
                  ret);
    }
    // requested attach to shelf
    if (request->attach_to_shelf == true) {
      RCLCPP_INFO(
          this->get_logger(),
          "In SVC Callback: requested with request->attach_to_shelf = true");

      rclcpp::Rate loop_rate(10);
      auto move = geometry_msgs::msg::Twist();

      calc_distance_to_goal();
      RCLCPP_INFO(this->get_logger(), "desired_pos x(%f), y(%f), theta(%f)",
                  desired_pos_.x, desired_pos_.y, desired_pos_.theta);
      RCLCPP_INFO(this->get_logger(), "current_pos x(%f), y(%f), theta(%f)",
                  current_pos_.x, current_pos_.y, current_pos_.theta);
      RCLCPP_INFO(this->get_logger(), "distance_pos x(%f), y(%f), theta(%f)",
                  dist_pos_.x, dist_pos_.y, dist_pos_.theta);
      double c_t = current_pos_.theta;
      rb_dir = false;
      if (c_t > -pi_ && c_t <= 0) // current pose
        rb_dir = true;
      if (dist_pos_.theta > 0 && dist_pos_.theta <= pi_) {
        if (rb_dir == true) {
          RCLCPP_INFO(this->get_logger(),
                      "truned robot to heading back for PI");
          turning_robot(h_pi_);
          rb_dir = false;
        }
      } else if (dist_pos_.theta > -pi_ && dist_pos_.theta <= 0) {
        if (rb_dir == false) {
          RCLCPP_INFO(this->get_logger(),
                      "truned robot to heading front for PI");
          turning_robot(-h_pi_);
          rb_dir = false;
        }
      }

      calc_distance_to_goal();

      if (fabs(dist_pos_.theta - current_pos_.theta) > 0.2) {
        RCLCPP_INFO(
            this->get_logger(),
            "current pose theta > 10 degrees, so robot turn to there first");
        turning_robot(dist_pos_.theta);
      }

      cnt = 0;
      while (rclcpp::ok()) {
        calc_distance_to_goal();
        if (cnt++ % 10 == 0) {
          RCLCPP_INFO(this->get_logger(), "desired_pos x(%f), y(%f), theta(%f)",
                      desired_pos_.x, desired_pos_.y, desired_pos_.theta);
          RCLCPP_INFO(this->get_logger(),
                      "current_pos x(%f), y(%f), theta(%f), inv_theta(%f)",
                      current_pos_.x, current_pos_.y, current_pos_.theta,
                      curr_inv_theta);
          RCLCPP_INFO(this->get_logger(), "dist_pos x(%f), y(%f), theta(%f)",
                      dist_pos_.x, dist_pos_.y, dist_pos_.theta);
        }
        int dir_f = 1;
        double dir_v = dist_pos_.theta - current_pos_.theta;

        if (rb_dir == false) {
          if (dir_v <= 0 && dir_v > -pi_)
            dir_v += pi_;
          else
            dir_v -= pi_;
          dir_v = fmod(dir_v, pi_);
        }
        if (fabs(dir_v) > 0.1) {
          if (dir_v < 0.0)
            dir_f = -1;
          move.angular.z = 0.5 * dir_f;
        } else {
          move.angular.z = 0.0;
        }
        move.linear.x = 0.2;
        if (fabs(dist_pos_.x) < 0.1 && fabs(dist_pos_.y) < 0.1) {
          RCLCPP_INFO(this->get_logger(), "arrived near by 0.1m");
          move.linear.x = 0.0;
          move.angular.z = 0.0;
        }

        cmd_publisher_->publish(move);

        if (cnt % 5 == 0) {
          RCLCPP_INFO(this->get_logger(), "move linear.x(%f), angular.z(%f)",
                      move.linear.x, move.angular.z);
        }
        loop_rate.sleep();
        if (move.linear.x == 0.0 && move.angular.z == 0.0)
          break;
      }

      turning_robot(desired_pos_.theta);

      // move forward about 30 cm
      move.angular.z = 0.0;
      move.linear.x = 0.3;
      cnt = 0;
      while (rclcpp::ok()) {
        if (++cnt > 60) {
          move.linear.x = 0.0;
          RCLCPP_INFO(this->get_logger(),
                      "Arrived at the right underneath !! (count: %d)", cnt);
        }

        cmd_publisher_->publish(move);

        if (cnt++ % 5 == 0)
          RCLCPP_INFO(this->get_logger(),
                      "Keep moving to the right underneath the shelf "
                      "position... (count: %d)",
                      cnt);
        loop_rate.sleep();
        if (cnt > 60)
          break;
      }
      cnt = 0;
      move.angular.z = 0.0;
      move.linear.x = 0.0;
      while (rclcpp::ok() && cnt++ < 5) {
        cmd_publisher_->publish(move);
        loop_rate.sleep();
      }

      RCLCPP_INFO(this->get_logger(), "Lift up the shelf...");
      do_lift_up_shelf();

      // stop moving to stabilize
      while (rclcpp::ok() && cnt++ < 20) {
        cmd_publisher_->publish(move);
        loop_rate.sleep();
      }

      // Set the response complete variable to true
      response->complete = true;
      RCLCPP_INFO(
          this->get_logger(),
          "Done: Approach Service completed ('attach_to_shelf' = true)");
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "Done: Approach Service completed ('attach_to_shelf' = false)");
    }
  }

  void do_lift_up_shelf() {
    std_msgs::msg::String msg;
    msg.data = "up";
    elev_up_->publish(msg);
    attached_shelf_fg_ = true;
  }

  void do_lift_down_shelf() {
    std_msgs::msg::String msg;
    msg.data = "down";
    elev_down_->publish(msg);
    attached_shelf_fg_ = false;
  }

  void calc_distance_to_goal() {
    dist_pos_.x = desired_pos_.x - current_pos_.x;
    dist_pos_.y = desired_pos_.y - current_pos_.y;
    dist_pos_.theta = atan2(dist_pos_.y, dist_pos_.x);
  }

  void turning_robot(float rdn) {
    rclcpp::Rate loop_rate(10);
    auto mv = geometry_msgs::msg::Twist();

    int c_f, r_f; // current_flag, rdn_flag
    int dr = 1;   // rotation direction
    double c_t = current_pos_.theta;

    if (fabs(c_t - rdn) < 0.1)
      return; // no rotation

    if (c_t <= 0 && c_t >= -h_pi_) // area 1
      c_f = 1;
    else if (c_t > 0 && c_t <= h_pi_) // area 2
      c_f = 2;
    else if (c_t > h_pi_ && c_t <= pi_) // area 3
      c_f = 3;
    else
      c_f = 4;

    if (rdn <= 0 && rdn >= -h_pi_) // area 1
      r_f = 1;
    else if (rdn > 0 && rdn <= h_pi_) // area 2
      r_f = 2;
    else if (rdn > h_pi_ && rdn <= pi_) // area 3
      r_f = 3;
    else
      r_f = 4;

    if (c_f == 1) {
      if (r_f == 1) {
        if (c_t < rdn)
          dr = -1;
        else
          dr = 1;
      } else if (r_f == 4)
        dr = -1;
      else if (r_f == 2)
        dr = 1;
      else {
        // c_f -= pi_;
        if (rdn > h_pi_ && rdn < c_t + pi_)
          dr = 1;
        else
          dr = -1;
      }
    } else if (c_f == 4) {
      if (r_f == 4) {
        if (c_t < rdn)
          dr = -1;
        else
          dr = 1;
      } else if (r_f == 3)
        dr = -1;
      else if (r_f == 1)
        dr = 1;
      else {
        // c_f -= pi_;
        if (rdn > 0 && rdn < pi_ + c_t)
          dr = 1;
        else
          dr = -1;
      }
    } else if (c_f == 2) {
      if (r_f == 2) {
        if (c_t < rdn)
          dr = -1;
        else
          dr = 1;
      } else if (r_f == 1)
        dr = -1;
      else if (r_f == 3)
        dr = 1;
      else {
        // c_f -= pi_;
        if (rdn > -pi_ && rdn < c_t - pi_)
          dr = 1;
        else
          dr = -1;
      }
    } else if (c_f == 3) {
      if (r_f == 3) {
        if (c_t < rdn)
          dr = 1;
        else
          dr = -1;
      } else if (r_f == 2)
        dr = -1;
      else if (r_f == 4)
        dr = 1;
      else {
        // c_f -= pi_;
        if (rdn <= 0 && rdn > -pi_ + c_t)
          dr = -1;
        else
          dr = 1;
      }
    }
    mv.angular.z = 0.5 * dr;
    mv.linear.x = 0.0;

    bool lp = true;
    while (rclcpp::ok() && lp == true) {
      if (fabs(rdn - current_pos_.theta) < 0.2) {
        mv.angular.z = 0.0;
        lp = false;
      }
      cmd_publisher_->publish(mv);
      loop_rate.sleep();
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto app_svc_svr_node = std::make_shared<ApproachServiceServerNode>();
  auto app_svc_sub_node = std::make_shared<SubscribeSensorNode>();

  RCLCPP_INFO(app_svc_svr_node->get_logger(),
              "approach_service_server_node INFO...");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(app_svc_svr_node);
  executor.add_node(app_svc_sub_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
