// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <inttypes.h>
#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using StringMsg = std_msgs::msg::String;

rclcpp::Node::SharedPtr g_node = nullptr;
bool g_cancel = false;
rclcpp::ActionServer<AddTwoInts, StringMsg>::SharedPtr g_action_server;

using namespace std::chrono_literals;

using SharedPtrWithRequestHeaderCallback = std::function<
  void(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<AddTwoInts::Request>,
    std::shared_ptr<AddTwoInts::Response>
  )>;

void handle_action(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  g_cancel = false; // this really should be enclosed in a mutex lock, but this is only a simple example

  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b)

  auto message = std_msgs::msg::String();
  message.data = "Waiting 10 seconds for cancellation";
  g_action_server->publish_feedback(message);
  RCLCPP_INFO(g_node->get_logger(), message.data.c_str())

  std::this_thread::sleep_for(10s);
  if (g_cancel == false)
  {
    response->sum = request->a + request->b;
    auto message = std_msgs::msg::String();
    message.data = "Response sent";
    g_action_server->publish_feedback(message);
    RCLCPP_INFO(g_node->get_logger(), message.data.c_str())
  }
  else {
	auto message = std_msgs::msg::String();
	message.data = "Request cancelled!";
	g_action_server->publish_feedback(message);
	RCLCPP_INFO(g_node->get_logger(), message.data.c_str())
  }
}

void handle_cancel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  std::shared_ptr<AddTwoInts::Response> response)
{
  // TODO: replace code here with correct message type for cancelling
  (void)request_header;
  g_cancel = true; // this really should be enclosed in a mutex lock, but this is only a simple example
  response->sum = 0; // TODO: make this return a status response
  RCLCPP_INFO(g_node->get_logger(), "Cancelling request: %" PRId64 " + %" PRId64, request->a, request->b)
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("minimal_action_server");
  // TODO: Add node interface to action server

  const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default;
  g_action_server = g_node->create_action_server<AddTwoInts, StringMsg>("add_two_ints",
		  handle_action,
		  handle_cancel,
		  qos_profile);
  
  RCLCPP_INFO(g_node->get_logger(), "Started minimal action server")

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
