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

#include "turtlebot3_autorace_mission/undocking.hpp"
#include "std_srvs/srv/trigger.hpp"

Undocking::Undocking(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("undocking_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
  {}

CallbackReturn Undocking::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Undocking Node CONFIGURED #####");

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&Undocking::publish_cmd_vel, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Undocking Node ACTIVATED #####");
  cmd_vel_pub_->on_activate();
  reached_target_ = false;

  target_x_ = -0.7;
  target_y_ = 0.0;
  tolerance_ = 0.1;
  reached_target_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Undocking Node DEACTIVATE #####");

  cmd_vel_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Undocking Node CLEANUP #####");

  timer_.reset();
  cmd_vel_pub_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "##### Undocking Node SHUTDOWN #####");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(this->get_logger(), "Error occurred in Undocking Node");
  return CallbackReturn::SUCCESS;
}

void Undocking::publish_cmd_vel()
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  if (tf_buffer_.canTransform("map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0))) {
    transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } else {
    RCLCPP_WARN(this->get_logger(), "Transform not available yet");
    return;
  }

  double dx = transform.transform.translation.x - target_x_;
  if (dx < tolerance_) {
    if (!reached_target_) {
      RCLCPP_INFO(this->get_logger(), "Undocking completed.");
    }
    reached_target_ = true;
  } else {
    reached_target_ = false;
  }

  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  if (reached_target_) {
    msg.twist.linear.x = 0.0;
    msg.twist.angular.z = 0.0;
    auto client = this->create_client<std_srvs::srv::Trigger>("state_change_trigger");
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "state_change_client for service not available.");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request);
  } else {
    msg.twist.linear.x = -0.2;
    msg.twist.angular.z = 0.0;
  }

  cmd_vel_pub_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Undocking>(rclcpp::NodeOptions{});
  rclcpp::executors::SingleThreadedExecutor exec;

  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
