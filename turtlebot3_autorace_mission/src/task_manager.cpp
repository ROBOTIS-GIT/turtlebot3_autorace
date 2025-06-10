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

#include "turtlebot3_autorace_mission/task_manager.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

TaskManager::TaskManager()
: Node("task_manager"),
  step_(1)
{
  RCLCPP_INFO(get_logger(), "TaskManaging Start");
  state_change_trigger_=this->create_service<std_srvs::srv::Trigger>(
    "state_change_trigger",
    std::bind(&TaskManager::state_change_callback,
      this, std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3)
  );
  exec_step(step_);
}
void TaskManager::exec_step(int step){
  if(step == 1){
    configure_activate_node("undocking_node");
  }
}

void TaskManager::state_change_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res){
  (void)request_header;
  (void)req;
  RCLCPP_INFO(this->get_logger(), "Received undocking done signal.");
  shutdown_node("undocking_node");
  res->success = true;
  goal_pose_publish(-0.22,-0.5, 0.0);
}

void TaskManager::configure_activate_node(const std::string & node_name){
  undocking_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + node_name + "/change_state");
  int retry_count = 0;
  int max_retries = 10;
  while (!undocking_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "%s", ("Waiting for /" + node_name + "/change_state service for configuring").c_str());
    retry_count++;
    if (retry_count > max_retries) {
      RCLCPP_ERROR(get_logger(), "%s", (node_name + "ConfiguringService not available.").c_str());
      return;
    }
  }

  auto request_configure= std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_configure->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

  undocking_client_->async_send_request(request_configure,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "%s", ("Successfully configured " + node_name).c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s", ("Failed to configure" + node_name).c_str());
        RCLCPP_ERROR(this->get_logger(), "Task Node will shut down");
        rclcpp::shutdown();
      }
    });

  auto request_activate = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_activate->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

  undocking_client_->async_send_request(request_activate,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "%s", ("Successfully activated " + node_name).c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s", ("Failed to activate" + node_name).c_str());
        RCLCPP_ERROR(this->get_logger(), "Task Node will shut down");
        shutdown_node("undocking_node");
        rclcpp::shutdown();
      }
    }
);
}

void TaskManager::shutdown_node(const std::string & node_name){
  undocking_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + node_name + "/change_state");
  int retry_count = 0;
  int max_retries = 10;
  while (!undocking_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "%s", ("Waiting for /" + node_name + "/change_state service for shutdown").c_str());
    retry_count++;
    if (retry_count > max_retries) {
      RCLCPP_ERROR(get_logger(), "%s", (node_name + "ConfiguringService not available.").c_str());
      return;
    }
  }

  auto request_deactivate = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_deactivate->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  undocking_client_->async_send_request(request_deactivate,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "%s", ("Successfully deactivated" + node_name).c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s", ("Failed to deactivate" + node_name).c_str());
      }
    });

  auto request_cleanup = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_cleanup->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

  undocking_client_->async_send_request(request_cleanup,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "%s", ("Successfully cleaned up" + node_name).c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "%s", ("Failed to clean up" + node_name).c_str());
      }
    });

  auto request_shutdown = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_shutdown->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;

  undocking_client_->async_send_request(request_shutdown,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "%s", ("Successfully shutdown" + node_name).c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "%s", ("Failed to shutdown" + node_name).c_str());
      }
    });
}

void TaskManager::goal_pose_publish(double x, double y, double theta)
{
  RCLCPP_INFO(this->get_logger(), "Publishing goal pose...");
  auto goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.frame_id = "map";
  goal_msg.header.stamp = this->get_clock()->now();
  goal_msg.pose.position.x = x;
  goal_msg.pose.position.y = y;
  goal_msg.pose.orientation = tf2::toMsg(q);

  goal_pub->publish(goal_msg);

    RCLCPP_INFO(this->get_logger(), "Publishing goal...");
    goal_pub->publish(goal_msg);
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}
