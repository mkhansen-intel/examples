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
#include "rclcpp/rclcpp.hpp"
 #include "rclcpp/utilities.hpp"

#include "rclcpp/action_server.hpp" // TODO: remove when node can create action server

using AddTwoInts = example_interfaces::srv::AddTwoInts;
rclcpp::Node::SharedPtr g_node = nullptr;
bool g_cancel = false;

void handle_action(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b)
  RCLCPP_INFO(g_node->get_logger(), "Waiting 10 seconds for cancellation")
  rclcpp::utilities::sleep_for(10000000000);
  if (g_cancel = false)
  {
    response->sum = request->a + request->b;
  }
}

void handle_cancel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
// TODO: replace code here with correct message type for cancelling
  (void)request_header;
  RCLCPP_INFO(
	g_node->get_logger(),
	"Cancelled request: %" PRId64 " + %" PRId64, request->a, request->b)
  response->sum = request->a + request->b;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("minimal_action_server");
  // TODO: Add node interface to action server
  //auto action_server = g_node->create_service<AddTwoInts>("add_two_ints", handle_action);
  const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default;
  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos_profile;
  auto action_server = ActionServer(g_node, "add_two_ints_action", handle_action, handle_cancel, service_options,  )
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
