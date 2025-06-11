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
#include <lifecycle_msgs/msg/transition.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
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
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "/navigate_to_pose");
  node_names_ = {
    "undocking_node",
    "yolo_detection_node",
    "alley_mission_node"
  };

  exec_step(step_);
}

void TaskManager::exec_step(int step){
  if(step == 1){
    RCLCPP_INFO(this->get_logger(), "########## Undocking Mission ##########");
    configure_activate_node("undocking_node");
  }
  else if(step==2){
    RCLCPP_INFO(this->get_logger(), "########## Move to Next step ##########");
    goal_pose_publish(-0.1,-0.5, 0.0);
  }
  else if(step==3){
    RCLCPP_INFO(this->get_logger(), "########## Yolo Detection Mission ##########");
    goal_pose_publish(-0.1,-0.7, -1.57);
  }
  else if(step==4){
    RCLCPP_INFO(this->get_logger(), "########## Alley Mission ##########");
    configure_activate_node("alley_mission_node");
  }
}

void TaskManager::state_change_callback(const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res){
  (void)request_header;
  (void)req;
  RCLCPP_INFO(this->get_logger(), "Mission");
  shutdown_node(node_names_[step_]);
  step_++;
  res->success = true;
}

void TaskManager::configure_activate_node(const std::string & node_name){
  client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + node_name + "/change_state");
  int retry_count = 0;
  int max_retries = 10;
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "%s", ("Waiting for /" + node_name + "/change_state service for configuring").c_str());
    retry_count++;
    if (retry_count > max_retries) {
      RCLCPP_ERROR(get_logger(), "%s", (node_name + "Configuring Service not available.").c_str());
      return;
    }
  }

  auto request_configure= std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_configure->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

  client_->async_send_request(request_configure,
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

  client_->async_send_request(request_activate,
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
  client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + node_name + "/change_state");
  int retry_count = 0;
  int max_retries = 10;
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "%s", ("Waiting for /" + node_name + "/change_state service for shutdown").c_str());
    retry_count++;
    if (retry_count > max_retries) {
      RCLCPP_ERROR(get_logger(), "%s", (node_name + "ConfiguringService not available.").c_str());
      return;
    }
  }

  auto request_deactivate = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_deactivate->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  client_->async_send_request(request_deactivate,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "%s", ("Successfully deactivated" + node_name).c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s", ("Failed to deactivate" + node_name).c_str());
      }
    });

  auto request_cleanup = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_cleanup->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

  client_->async_send_request(request_cleanup,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "%s", ("Successfully cleaned up" + node_name).c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "%s", ("Failed to clean up" + node_name).c_str());
      }
    });

  auto request_shutdown = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_shutdown->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;

  client_->async_send_request(request_shutdown,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "%s", ("Successfully shutdown" + node_name).c_str());
        exec_step(step_);
      } else {
        RCLCPP_INFO(this->get_logger(), "%s", ("Failed to shutdown" + node_name).c_str());
      }
    });
}

void TaskManager::goal_pose_publish(double x, double y, double theta)
{
  RCLCPP_INFO(this->get_logger(), "Publishing goal pose: (%.2f, %.2f, %.2f)", x, y, theta);
  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available.");
      return;
    }
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->get_clock()->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  // ## FOR DEBUG ##
  // send_goal_options.feedback_callback =
  //   [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
  //          const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  //   {
  //     RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f",
  //                 feedback->distance_remaining);
  //   };

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
          step_++;
          exec_step(step_);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
          break;
      }
    };
  nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}

