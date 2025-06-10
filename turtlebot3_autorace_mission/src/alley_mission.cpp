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
// Author: Hyungyu Kim (Modified by ChatGPT for position following)

#include "turtlebot3_autorace_mission/alley_mission.hpp"
#include <cmath>

AlleyMission::AlleyMission(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("alley_mission_node", options),
current_waypoint_index_(0)
{
  waypoints_ = {
    {-0.03, -0.07},
    {-0.03, -0.09},
    {-0.03, -0.11},
    {-0.03, -0.13}
  };
}

CallbackReturn AlleyMission::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Alley Mission Node");

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&AlleyMission::amcl_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&AlleyMission::publish_cmd_vel, this));

  cmd_vel_.header.stamp = this->get_clock()->now();
  cmd_vel_.header.frame_id = "base_link";
  cmd_vel_.twist.linear.x = 0.1;
  cmd_vel_.twist.angular.z = 0.0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Alley Mission Node");
  cmd_vel_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating Alley Mission Node");
  cmd_vel_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up Alley Mission Node");
  cmd_vel_pub_.reset();
  amcl_sub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Alley Mission Node");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(this->get_logger(), "Error occurred in Alley Mission Node");
  return CallbackReturn::SUCCESS;
}

void AlleyMission::amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (current_waypoint_index_ >= waypoints_.size()) {
    RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "amcl callback");
  double target_x = waypoints_[current_waypoint_index_].first;
  double target_y = waypoints_[current_waypoint_index_].second;

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double dx = target_x - x;
  double dy = target_y - y;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance > 0.1) {
    double angle_to_goal = std::atan2(dy, dx);

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double angle_diff = std::atan2(std::sin(angle_to_goal - yaw), std::cos(angle_to_goal - yaw));

    if (std::abs(angle_diff) > 0.1) {
      cmd_vel_.twist.angular.z = 0.5 * angle_diff;
    } else {
      cmd_vel_.twist.linear.x = 0.2 * distance;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_index_);
    current_waypoint_index_++;
  }
}

void AlleyMission::publish_cmd_vel()
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }

  if (current_waypoint_index_ < waypoints_.size()) {
    cmd_vel_pub_->publish(cmd_vel_);
  } else {
    RCLCPP_INFO(this->get_logger(), "All waypoints reached, stopping.");
  }
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
