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

#include "std_srvs/srv/trigger.hpp"
#include "turtlebot3_autorace_mission/door_docking.hpp"


DoorDocking::DoorDocking(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("door_docking_node", options)
{}

CallbackReturn DoorDocking::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;34mDoorDocking Node INIT\033[0m");
  reached_target_ = false;
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DoorDocking::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;34mDoorDocking Node ACTIVATE\033[0m");
  cmd_vel_pub_->on_activate();
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&DoorDocking::scan_callback, this, std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

CallbackReturn DoorDocking::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mDoorDocking Node DEACTIVATE\033[0m");
  cmd_vel_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DoorDocking::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mDoorDocking Node CLEANUP\033[0m");
  cmd_vel_pub_.reset();
  scan_sub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DoorDocking::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mDoorDocking Node SHUTDOWN\033[0m");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DoorDocking::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(this->get_logger(), "Error occurred in DoorDocking Node");
  return CallbackReturn::SUCCESS;
}

void DoorDocking::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (reached_target_) {
    return;
  }

  const double angle_min = msg->angle_min;
  const double angle_max = msg->angle_max;
  const double angle_increment = msg->angle_increment;

  const double target_deg = 10.0 * M_PI / 180.0;
  int left_idx = std::round((target_deg - angle_min) / angle_increment);
  int right_idx = std::round((angle_max - target_deg) / angle_increment);

  float left_dist = msg->ranges[left_idx];
  float right_dist = msg->ranges[right_idx];
  float forward_dist = msg->ranges[0];

  auto cmd = geometry_msgs::msg::TwistStamped();
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  if (std::isfinite(left_dist) && std::isfinite(right_dist)) {
    double diff = left_dist - right_dist;
    if (std::fabs(diff) > 0.02) {
      cmd.twist.angular.z = -0.8 * diff;
      RCLCPP_INFO(this->get_logger(), "DIFF: %f", diff);
    } else if (forward_dist > 0.3) {
      cmd.twist.linear.x = 0.8;
    } else {
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
  }
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DoorDocking>(rclcpp::NodeOptions{});
  rclcpp::executors::SingleThreadedExecutor exec;

  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
