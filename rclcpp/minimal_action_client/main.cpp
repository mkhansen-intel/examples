// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <cinttypes>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using StringMsg = std_msgs::msg::String;


void feedback_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Feedback"), "'%s'", msg->data.c_str())
}

void send_and_wait(rclcpp::Node::SharedPtr node)
{
  auto client =
    node->create_action_client<AddTwoInts, StringMsg>("add_two_ints", feedback_callback);

  while (!client->wait_for_action(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for action to appear.")
      return;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for action to appear...")
  }
  RCLCPP_INFO(node->get_logger(), "Sending request...");
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 41;
  request->b = 1;
  auto result_future = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Waiting for response...");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "action call failed :(")
    return;
  }
  RCLCPP_INFO(node->get_logger(), "Received response...");

  auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
    request->a, request->b, result->sum)
}

void send_and_cancel(rclcpp::Node::SharedPtr node)
{
  auto client =
    node->create_action_client<AddTwoInts, StringMsg>("add_two_ints", feedback_callback);

  while (!client->wait_for_action(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.")
      return;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...")
  }
  RCLCPP_INFO(node->get_logger(), "Sending request...")
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 41;
  request->b = 1;
  auto result_future = client->async_send_request(request);

  RCLCPP_INFO(node->get_logger(), "Cancelling request...")
  auto cancel_future = client->cancel_request(request);

  if (rclcpp::spin_until_future_complete(node, cancel_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Cancel failed!")
    return;
  }

  auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "Cancel succeeded")
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_action_client");

  send_and_wait(node);
  send_and_cancel(node);

  rclcpp::shutdown();
  return 0;
}
