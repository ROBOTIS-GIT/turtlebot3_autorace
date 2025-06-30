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

#include <lifecycle_msgs/msg/transition.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "turtlebot3_autorace_mission/task_manager.hpp"


using NavigateToPose = nav2_msgs::action::NavigateToPose;

TaskManager::TaskManager()
: Node("task_manager"),
  step_(1)
{
  state_change_trigger_ = this->create_service<std_srvs::srv::Trigger>(
    "state_change_trigger",
    std::bind(&TaskManager::state_change_callback,
      this, std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3)
  );
  undocking_target_client_ = this->create_client<turtlebot3_autorace_msgs::srv::UndockingTarget>(
    "undocking_target"
  );
  detection_ = this->create_service<turtlebot3_autorace_msgs::srv::DetectionResult>(
    "detection_result",
    std::bind(&TaskManager::detection_callback,
      this, std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3)
  );
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "/navigate_to_pose");
  node_names_ = {
    "error",
    "undocking_node",
    "nav2_node",
    "object_detection_node",
    "nav2_node",
    "alley_mission_node",
    "nav2_node",
    "object_detection_node",
    "nav2_node",
    "object_detection_node",
    "nav2_node",
    "object_detection_node",
    "aruco_parking",
    "aruco_tracker",
    "undocking_node",
    "nav2_node",
    "nav2_node",
    "nav2_node",
    "object_detection_node",
    "nav2_node",
    "object_detection_node",
    "nav2_node",
    "object_detection_node",
    "door_docking_node",
    "undocking_node"
  };
  exec_step(step_);
}

void TaskManager::exec_step(int step)
{
  if(step == 1) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Undocking Starts #####\033[0m");
    configure_activate_node("undocking_node", client_);
    undocking_target_send(-0.7, 0.0);
  } else if(step == 2) {
    RCLCPP_INFO(
      this->get_logger(),
      "\033[1;32m##### Move forward to the ordering panel #####\033[0m");
    goal_pose_publish(-0.1, -0.6, 0.0);
  } else if(step == 3) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Order Details Perception #####\033[0m");
    configure_activate_node("object_detection_node", client_);
  } else if(step == 4) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to the alley #####\033[0m");
    goal_pose_publish(-0.1, -0.7, -1.57);
  } else if(step == 5) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Alley Driving #####\033[0m");
    configure_activate_node("alley_mission_node", client_);
  } else if(step == 6) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to first shop #####\033[0m");
    goal_pose_publish(-0.14, -2.19, -1.57);
  } else if(step == 7) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Identify the type of store #####\033[0m");
    configure_activate_node("object_detection_node", client_);
  } else if(step == 8) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to second shop #####\033[0m");
    goal_pose_publish(-1.18, -2.19, -1.57);
  } else if(step == 9) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Identify the type of store #####\033[0m");
    configure_activate_node("object_detection_node", client_);
  } else if(step == 10) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to third shop #####\033[0m");
    goal_pose_publish(-2.26, -2.19, -1.57);
  } else if(step == 11) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Identify the type of store #####\033[0m");
    configure_activate_node("object_detection_node", client_);
  } else if(step == 12) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### ArUco docking #####\033[0m");
    configure_activate_node("aruco_tracker", secondary_client_);
    configure_activate_node("aruco_parking", client_);
  } else if (step == 13) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Pickup #####\033[0m");
    step_++;
    deactivate_cleanup_node("aruco_tracker", secondary_client_);
  } else if (step == 14) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Undocking #####\033[0m");
    configure_activate_node("undocking_node", client_);
    undocking_target_send(0.0, -2.2);
  } else if (step == 15) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to bollard #####\033[0m");
    goal_pose_publish(-2.1, -2.12, 1.57);
  } else if (step == 16) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Bollard Mission by Nav2 #####\033[0m");
    goal_pose_publish(-2.1, -0.5, 1.57);
  } else if (step == 17) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to first door #####\033[0m");
    goal_pose_publish(-2.4, -0.36, 1.57);
  } else if (step == 18) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Identify the type of door #####\033[0m");
    configure_activate_node("object_detection_node", client_);
  } else if (step == 19) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to second door #####\033[0m");
    goal_pose_publish(-1.9, -0.36, 1.57);
  } else if (step == 20) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Identify the type of door #####\033[0m");
    configure_activate_node("object_detection_node", client_);
  } else if (step == 21) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to third door #####\033[0m");
    goal_pose_publish(-1.4, -0.36, 1.57);
  } else if (step == 22) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Identify the type of door #####\033[0m");
    configure_activate_node("object_detection_node", client_);
  } else if (step == 23) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Docking at door #####\033[0m");
    configure_activate_node("door_docking_node", client_);
  } else if (step == 24) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Undocking #####\033[0m");
    configure_activate_node("undocking_node", client_);
    undocking_target_send(0.0, -0.36);
  } else if (step == 25) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Move forward to charge station #####\033[0m");
    goal_pose_publish(-0.4, -0.1, 0.0);
  } else if (step == 26) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Charging Station Docking #####\033[0m");
    configure_activate_node("lidar_docking", client_);
  } else {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m##### Mission Completed #####\033[0m");
  }
  return;
}

void TaskManager::state_change_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)request_header;
  (void)req;
  RCLCPP_INFO(this->get_logger(), "Quit Service called");
  deactivate_cleanup_node(node_names_[step_++], client_);
  res->success = true;
}

void TaskManager::detection_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Request> req,
  const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Response> res)
{
  (void)request_header;
  if (step_ == 3) {
    detection_callback_order_details(req, res);
  } else if(step_ == 7 || step_ == 9 || step_ == 11) {
    detection_callback_store_sign(req, res);
  } else if(step_ == 18 || step_ == 20 || step_ == 22) {
    detection_callback_door_sign(req, res);
  }
}

void TaskManager::configure_activate_node(
  const std::string & node_name,
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr & client_)
{
  client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/" + node_name + "/change_state");
  int retry_count = 0;
  int max_retries = 10;

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(),
      "%s",
      ("Waiting for /" + node_name + "/change_state service for configuring").c_str());
    retry_count++;
    if (retry_count > max_retries) {
      RCLCPP_ERROR(get_logger(), "%s", (node_name + "Configuring Service not available.").c_str());
      return;
    }
  }
  auto request_configure = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_configure->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  client_->async_send_request(request_configure,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(
          this->get_logger(),
          "%s",
          ("\033[1;34mSuccessfully configured " + node_name + "\033[0m").c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s", ("Failed to configure" + node_name).c_str());
        RCLCPP_ERROR(this->get_logger(), "Task Node will shut down");
        rclcpp::shutdown();
      }
    }
  );

  auto request_activate = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_activate->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  client_->async_send_request(request_activate,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(
          this->get_logger(),
          "%s",
          ("\033[1;34mSuccessfully activated " + node_name + "\033[0m").c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s", ("Failed to activate" + node_name).c_str());
        RCLCPP_ERROR(this->get_logger(), "Task Node will shut down");
        rclcpp::shutdown();
      }
    }
  );
}

void TaskManager::deactivate_cleanup_node(
  const std::string & node_name,
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr & client_)
{
  int retry_count = 0;
  int max_retries = 10;
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(),
      "%s",
      ("Waiting for /" + node_name + "/change_state service for shutdown").c_str());
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
        RCLCPP_INFO(
          this->get_logger(),
          "%s",
          ("\033[1;31mSuccessfully deactivated" + node_name + "\033[0m").c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s", ("Failed to deactivate" + node_name).c_str());
      }
    });

  auto request_cleanup = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request_cleanup->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

  client_->async_send_request(request_cleanup,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(
          this->get_logger(),
          "%s",
          ("\033[1;31mSuccessfully cleaned up " + node_name + "\033[0m").c_str());
        exec_step(step_);
      } else {
        RCLCPP_INFO(this->get_logger(), "%s", ("Failed to clean up " + node_name).c_str());
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

void TaskManager::undocking_target_send(float x, float y)
{
  RCLCPP_INFO(this->get_logger(), "Sending undocking target: (%.2f, %.2f)", x, y);
  if (!undocking_target_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "UndockingTarget service not available.");
    return;
  }
  auto request = std::make_shared<turtlebot3_autorace_msgs::srv::UndockingTarget::Request>();
  request->target_x = x;
  request->target_y = y;

  undocking_target_client_->async_send_request(request,
    [this](rclcpp::Client<turtlebot3_autorace_msgs::srv::UndockingTarget>::SharedFuture result) {
      if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Undocking target sent successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send undocking target.");
      }
    }
  );
}

void TaskManager::detection_callback_order_details(
  const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Request> req,
  const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Response> res)
{
  if (req->stores.size() == 3 && req->rooms.size() == 3) {
    for (size_t i = 0; i < req->stores.size(); ++i) {
      order_details_.push_back({req->stores[i], req->rooms[i]});
    }
    RCLCPP_INFO(
      this->get_logger(),
      "First order: %s -> %s",
      order_details_[0][0].c_str(),
      order_details_[0][1].c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Second order: %s -> %s",
      order_details_[1][0].c_str(),
      order_details_[1][1].c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Third order: %s -> %s",
      order_details_[2][0].c_str(),
      order_details_[2][1].c_str());
    res->success = true;
    deactivate_cleanup_node(node_names_[step_++], client_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Detection failed. Detection again.");
    res->success = false;
  }
}

void TaskManager::detection_callback_store_sign(
  const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Request> req,
  const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Response> res)
{
  if (req->stores.size() == 1) {
    res->success = true;
    if (req->stores[0] == order_details_[0][0]) {
      deactivate_cleanup_node("object_detection_node", client_);
      step_ = 12;
    } else {
      deactivate_cleanup_node("object_detection_node", client_);
      step_++;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "It is not unique detection result. Detection again.");
    res->success = false;
  }
}

void TaskManager::detection_callback_door_sign(
  const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Request> req,
  const std::shared_ptr<turtlebot3_autorace_msgs::srv::DetectionResult::Response> res)
{
  if (req->rooms.size() == 1) {
    res->success = true;
    if (req->rooms[0] == order_details_[0][1]) {
      deactivate_cleanup_node("object_detection_node", client_);
      step_ = 23;
    } else {
      deactivate_cleanup_node("object_detection_node", client_);
      step_++;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "It is not unique detection result. Detection again.");
    res->success = false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}
