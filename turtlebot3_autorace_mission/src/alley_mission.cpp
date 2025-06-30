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

#include <algorithm>
#include <cmath>

#include <std_srvs/srv/trigger.hpp>

#include "turtlebot3_autorace_mission/alley_mission.hpp"

AlleyMission::AlleyMission(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("alley_mission_node", options)
{}

CallbackReturn AlleyMission::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;34mAlley Mission INIT\033[0m");
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  reached_target_ = false;
  status_ = 0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;34mAlley Mission ACTIVATE\033[0m");
  cmd_vel_pub_->on_activate();
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&AlleyMission::scan_callback, this, std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mAlley Mission DEACTIVATE\033[0m");
  cmd_vel_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mAlley Mission CLEANUP\033[0m");
  cmd_vel_pub_.reset();
  scan_sub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mAlley Mission SHUTDOWN\033[0m");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(this->get_logger(), "Error occurred in Alley Mission Node");
  return CallbackReturn::SUCCESS;
}

void AlleyMission::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (reached_target_) {
    return;
  }

  if (status_ == 0) {
    align_to_wall(msg);
  } else if (status_ == 1) {
    wall_following(msg);
  }
}

void AlleyMission::align_to_wall(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const float angle_min = msg->angle_min;
  const float angle_max = msg->angle_max;
  const float angle_increment = msg->angle_increment;
  const float target_deg = 5.0 * M_PI / 180.0;

  int left_idx = std::round((target_deg - angle_min) / angle_increment);
  int right_idx = std::round((angle_max - target_deg) / angle_increment);
  float left_dist = msg->ranges[left_idx];
  float right_dist = msg->ranges[right_idx];
  float forward_distance = msg->ranges[0];

  auto cmd = geometry_msgs::msg::TwistStamped();
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  if (std::isfinite(left_dist) && std::isfinite(right_dist)) {
    double diff = left_dist - right_dist;
    if (std::fabs(diff) > 0.03) {
      float kp = -0.8f;
      cmd.twist.angular.z = kp * diff;
    } else if (forward_distance > 0.7) {
      cmd.twist.linear.x = 0.08;
    } else {
      status_ = 1;
      RCLCPP_INFO(this->get_logger(), "Aligned to wall");
    }
  }
  cmd_vel_pub_->publish(cmd);
}

void AlleyMission::wall_following(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const float angle_min = msg->angle_min;
  const float angle_max = msg->angle_max;
  const float angle_increment = msg->angle_increment;
  float front_distance = msg->ranges[0];
  float left_distance = msg->ranges[std::round((M_PI_2 - angle_min) / angle_increment)];
  float back_distance = msg->ranges[std::round((M_PI - angle_min) / angle_increment)];
  const std::vector<float> target_deg =
  {
    90.0 * M_PI / 180.0,
    75.0 * M_PI / 180.0,
    60.0 * M_PI / 180.0,
    45.0 * M_PI / 180.0,
    30.0 * M_PI / 180.0
  };

  float min = 0.3f;
  for (const auto & deg : target_deg) {
    int idx = std::round((angle_max - deg) / angle_increment);
    float target_distance = msg->ranges[idx] * std::sin(deg);
    min = std::min(min, target_distance);
  }

  auto cmd = geometry_msgs::msg::TwistStamped();
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  float criterion = (0.15f - min);
  int kp = 20;
  cmd.twist.angular.z = std::clamp(kp * criterion, -2.0f, 2.0f);
  cmd.twist.linear.x = cmd.twist.angular.z > 0.2 ? 0.0 : 0.05;
  if (front_distance > 0.9 && left_distance > 0.8 && back_distance > 0.5) {
    RCLCPP_INFO(this->get_logger(), "Alley Mission is completed");
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    auto client = this->create_client<std_srvs::srv::Trigger>("state_change_trigger");
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "state_change_client for service not available.");
      return;
    }
    reached_target_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request);
  }
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlleyMission>(rclcpp::NodeOptions{});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
