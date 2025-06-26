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

#ifndef TURTLEBOT3_AUTORACE_MISSION__TASK_MANAGER_HPP_
#define TURTLEBOT3_AUTORACE_MISSION__TASK_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "turtlebot3_autorace_msgs/srv/detection_result.hpp"
#include "turtlebot3_autorace_msgs/srv/undocking_target.hpp"

class TaskManager : public rclcpp::Node
{
public:
  TaskManager();

private:
  void exec_step(int step);
  void state_change_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void detection_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Request> req,
    const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Response> res);
  void configure_activate_node(
    const std::string & node_name,
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr & client);
  void deactivate_cleanup_node(
    const std::string & node_name,
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr & client);
  void goal_pose_publish(double x, double y, double theta);
  void detection_callback_order_details(
    const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Request> req,
    const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Response> res);
  void detection_callback_store_sign(
    const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Request> req,
    const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Response> res);
  void detection_callback_door_sign(
    const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Request> req,
    const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Response> res);
  void undocking_target_send(float x, float y);

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr secondary_client_;
  rclcpp::Client<turtlebot3_autorace_msgs::srv::UndockingTarget>::SharedPtr
    undocking_target_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_change_trigger_;
  rclcpp::Service<turtlebot3_autorace_msgs::srv::DetectionResult>::SharedPtr detection_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;

  int step_;
  std::vector<std::string> node_names_;
  std::vector<std::vector<std::string>> order_details_;
};

#endif  // TURTLEBOT3_AUTORACE_MISSION__TASK_MANAGER_HPP_
