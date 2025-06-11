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

#include "turtlebot3_autorace_mission/alley_mission.hpp"
#include <cmath>

AlleyMission::AlleyMission(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("alley_mission_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
  {}

CallbackReturn AlleyMission::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Alley Mission INIT #####");

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&AlleyMission::publish_cmd_vel, this));

  current_waypoint_index_ = 0;
  waypoints_ = {
    {-0.03, -0.7},
    {-0.03, -0.9},
    {-0.03, -1.1},
    {-0.03, -1.32},
    {-0.23, -1.32},
    {-0.43, -1.32},
    {-0.59, -1.32},
    {-0.59, -1.52},
    {-0.59, -1.72},
    {-0.59, -1.92},
    {-0.59, -2.12}
  };

  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Alley Mission ACTIVATE #####");
  cmd_vel_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Alley Mission DEACTIVATE #####");
  cmd_vel_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Alley Mission CLEANUP #####");
  cmd_vel_pub_.reset();
  timer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Alley Mission SHUTDOWN #####");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlleyMission::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(this->get_logger(), "Error occurred in Alley Mission Node");
  return CallbackReturn::SUCCESS;
}


void AlleyMission::publish_cmd_vel()
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }
  geometry_msgs::msg::TransformStamped transform =
    tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

  double roll, pitch, yaw;
  tf2::Quaternion q(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  double dx = waypoints_[current_waypoint_index_].first - transform.transform.translation.x;
  double dy = waypoints_[current_waypoint_index_].second - transform.transform.translation.y;
  double look_ahead_distance = std::hypot(dx, dy);
  double target_yaw = std::atan2(dy, dx) - yaw;
  target_yaw = std::atan2(std::sin(target_yaw), std::cos(target_yaw));

  cmd_vel_.header.stamp = this->get_clock()->now();
  cmd_vel_.header.frame_id = "map";
  if (current_waypoint_index_ < waypoints_.size()) {
    if (look_ahead_distance > 0.1) {
      if(target_yaw <M_PI/6 && target_yaw > -M_PI/6) {
        cmd_vel_.twist.linear.x = 0.08;
        cmd_vel_.twist.angular.z = target_yaw;
      } else if (target_yaw > M_PI/6 || target_yaw < -M_PI/6) {
        cmd_vel_.twist.linear.x = 0.03;
        cmd_vel_.twist.angular.z = target_yaw;
      }
    } else {
      current_waypoint_index_++;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Reached Destination.");
    cmd_vel_.twist.linear.x = 0;
    cmd_vel_.twist.angular.z = 0;
  }
  cmd_vel_pub_->publish(cmd_vel_);
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
