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


AlleyMission::AlleyMission(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("alley_mission_node", options)
  {}

CallbackReturn AlleyMission::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring...");

    return CallbackReturn::SUCCESS;
  }

CallbackReturn AlleyMission::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AlleyMission::on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AlleyMission::on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AlleyMission::on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AlleyMission::on_error(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_WARN(this->get_logger(), "Error occurred.");
    return CallbackReturn::SUCCESS;
  }


int main(int argc, char * argv[]){
  return 0;
}