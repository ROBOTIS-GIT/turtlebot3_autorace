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
#include "turtlebot3_autorace_mission/undocking.hpp"


Undocking::Undocking(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("undocking_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{}

CallbackReturn Undocking::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;34mUndocking Node INIT\033[0m");
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  target_received_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;34mUndocking Node ACTIVATE\033[0m");
  cmd_vel_pub_->on_activate();
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&Undocking::publish_cmd_vel, this));
  undocking_target_ = this->create_service<turtlebot3_autorace_msgs::srv::UndockingTarget>(
    "undocking_target",
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<turtlebot3_autorace_msgs::srv::UndockingTarget::Request> request,
      const std::shared_ptr<turtlebot3_autorace_msgs::srv::UndockingTarget::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Received undocking target request");
      (void)request_header;
      target_x_ = request->target_x;
      target_y_ = request->target_y;
      target_received_ = true;
      response->success = true;
    });
  reached_target_ = false;
  tolerance_ = 0.1;

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mUndocking Node DEACTIVATE\033[0m");
  cmd_vel_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mUndocking Node CLEANUP\033[0m");
  timer_.reset();
  cmd_vel_pub_.reset();
  undocking_target_.reset();
  target_x_ = 0.0;
  target_y_ = 0.0;
  target_received_ = false;
  reached_target_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;31mUndocking Node SHUTDOWN\033[0m");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Undocking::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(this->get_logger(), "Error occurred in Undocking Node");
  return CallbackReturn::SUCCESS;
}

void Undocking::publish_cmd_vel()
{
  if (reached_target_ || !target_received_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  bool can_transform = tf_buffer_.canTransform(
    "map",
    "base_link",
    rclcpp::Time(0),
    rclcpp::Duration::from_seconds(1.0));
  if (can_transform) {
    transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } else {
    RCLCPP_WARN(this->get_logger(), "transform base_link to map not available yet");
    return;
  }

  double criterion = 0.0;
  if (target_x_ == 0.0) {
    criterion = transform.transform.translation.y - target_y_;
  } else if (target_y_ == 0.0) {
    criterion = transform.transform.translation.x - target_x_;
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid undocking target");
    return;
  }

  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  if (abs(criterion) < tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Undocking completed.");
    msg.twist.linear.x = 0.0;
    msg.twist.angular.z = 0.0;
    auto client = this->create_client<std_srvs::srv::Trigger>("state_change_trigger");
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "state_change_client for service not available.");
      return;
    }
    reached_target_ = true;
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
