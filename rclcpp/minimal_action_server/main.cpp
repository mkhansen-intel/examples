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

#include "rclcpp/action_server.hpp" // TODO: remove when node can create action server

using AddTwoInts = example_interfaces::srv::AddTwoInts;
rclcpp::Node::SharedPtr g_node = nullptr;
bool g_cancel = false;
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
  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b)
  RCLCPP_INFO(g_node->get_logger(), "Waiting 10 seconds for cancellation")
  std::this_thread::sleep_for(10s);
  if (g_cancel == false)
  {
    response->sum = request->a + request->b;
  }
  RCLCPP_INFO(g_node->get_logger(), "Response sent")
}

void handle_cancel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  std::shared_ptr<AddTwoInts::Response> response)
{
// TODO: replace code here with correct message type for cancelling
  (void)request_header;
  RCLCPP_INFO(
	g_node->get_logger(),
	"Cancelled request: %" PRId64 " + %" PRId64, request->a, request->b)
  g_cancel = true;
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
  auto node_handle = g_node->get_node_base_interface()->get_shared_rcl_node_handle();
  const std::string & action_name = "add_two_ints_action";
  rclcpp::AnyServiceCallback<AddTwoInts> action_callback;
  action_callback.set(std::forward<SharedPtrWithRequestHeaderCallback>(handle_action));
  rclcpp::AnyServiceCallback<AddTwoInts> cancel_callback;
  cancel_callback.set(std::forward<SharedPtrWithRequestHeaderCallback>(handle_cancel));
  auto action_server = rclcpp::ActionServer<AddTwoInts>::make_shared(node_handle,
		  action_name, action_callback, cancel_callback, service_options,
		  g_node->get_node_services_interface(), nullptr);
  
  RCLCPP_INFO(g_node->get_logger(), "Started minimal action server")

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
