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

#ifndef TASK_MANAGER_HPP_
#define TASK_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "lifecycle_msgs/srv/change_state.hpp"

class TaskManager : public rclcpp::Node
{
public:
  TaskManager();

private:
  void exec_step(int step);
  void state_change_callback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void configure_activate_node(const std::string & node_name);
  void shutdown_node(const std::string & node_name);
  void goal_pose_publish(double x, double y, double theta);

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_check_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_change_trigger_;

  int step_;
  std::vector<std::string> node_names_;
};


#endif  // TASK_MANAGER_HPP_
