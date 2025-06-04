// Copyright 2025 ROBOTIS CO., LTD.
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
//
// Author: Hyungyu Kim

#include "turtlebot3_autorace_mission/undocking.hpp"
#include "geometry_msgs/msg/twist.hpp"

Undocking::Undocking(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("undocking_node", options),
  target_x_(-0.5),
  target_y_(0.0),
  tolerance_(0.1),
  reached_target_(false)
  {}

CallbackReturn Undocking::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Undocking Node");

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose",
    10,
    std::bind(&Undocking::amcl_pose_callback, this, std::placeholders::_1));
  // 타이머
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&Undocking::publish_cmd_vel, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Undocking Node");

  cmd_vel_pub_->on_activate();
  reached_target_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Undocking Node");

  cmd_vel_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Undocking Node");

  timer_.reset();
  cmd_vel_pub_.reset();
  amcl_sub_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Undocking Node");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(this->get_logger(), "Error occurred in Undocking Node");
  return CallbackReturn::SUCCESS;
}

// amcl 콜백
void Undocking::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  double dx = msg->pose.pose.position.x - target_x_;

  if (dx < tolerance_) {
    if (!reached_target_) {
      RCLCPP_INFO(this->get_logger(), "Target reached!");
    }
    reached_target_ = true;
  } else {
    reached_target_ = false;
  }
}
// cmd 퍼블리셔 함수
void Undocking::publish_cmd_vel()
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }

  auto msg = geometry_msgs::msg::TwistStamped();
msg.header.stamp = this->now();
msg.header.frame_id = "base_link";

if (reached_target_) {
  msg.twist.linear.x = 0.0;
  msg.twist.angular.z = 0.0;
} else {
  msg.twist.linear.x = -0.2;
  msg.twist.angular.z = 0.0;
}

cmd_vel_pub_->publish(msg);
  cmd_vel_pub_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Undocking>(rclcpp::NodeOptions{});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
