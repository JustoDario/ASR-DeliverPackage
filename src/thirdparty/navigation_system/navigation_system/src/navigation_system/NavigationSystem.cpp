// Copyright 2024 Juan Carlos Manzanares Serrano
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

#include "navigation_system/NavigationSystem.hpp"

namespace navigation_system
{

NavigationSystem::NavigationSystem(const rclcpp::NodeOptions & options)
: CascadeLifecycleNode("navigation_system", options)
{
  message("NavigationSystem constructor", 1);

  declare_parameter("nodes", nodes_);
  declare_parameter("mode", std::string());
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
NavigationSystem::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  get_parameter("nodes", nodes_);
  std::string mode;
  get_parameter("mode", mode);

  set_initial_mode(mode);
  start_services();
  startup();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
NavigationSystem::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
NavigationSystem::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

void NavigationSystem::set_initial_mode(const std::string & mode)
{
  if (mode == "amcl") {
    mode_.id = navigation_system_interfaces::msg::Mode::AMCL;
  } else if (mode == "slam") {
    mode_.id = navigation_system_interfaces::msg::Mode::SLAM;
  } else {
    mode_.id = navigation_system_interfaces::msg::Mode::NO_MODE;
  }
}

void NavigationSystem::start_services()
{
  std::string map_service = std::string(this->get_name()) + "/set_map";
  std::string mode_service = std::string(this->get_name()) + "/set_mode";
  std::string truncate_distance_service = std::string(this->get_name()) + "/set_truncate_distance";

  set_map_service_ =
    this->create_service<navigation_system_interfaces::srv::SetMap>(
    map_service,
    std::bind(
      &NavigationSystem::handleSetMap, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_mode_service_ =
    this->create_service<navigation_system_interfaces::srv::SetMode>(
    mode_service,
    std::bind(
      &NavigationSystem::handleSetMode, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_truncate_distance_service_ =
    this->create_service<navigation_system_interfaces::srv::SetTruncateDistance>(
    truncate_distance_service,
    std::bind(
      &NavigationSystem::handleSetTruncateDistance, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void NavigationSystem::startup()
{
  change_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  change_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  message("NavigationSystem started", 1);
}

void NavigationSystem::reset()
{
  change_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  change_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  change_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  change_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

void NavigationSystem::startup_node(const std::string & node_name)
{
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

void NavigationSystem::clean_node(const std::string & node_name)
{
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

void NavigationSystem::reset_node(const std::string & node_name)
{
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

void NavigationSystem::change_all_nodes(uint8_t transition)
{
  for (auto node : nodes_) {
    if ((mode_.id == navigation_system_interfaces::msg::Mode::SLAM && node == "amcl") ||
      (mode_.id == navigation_system_interfaces::msg::Mode::AMCL && node == "slam_toolbox") ||
      (mode_.id == navigation_system_interfaces::msg::Mode::NO_MODE &&
      (node == "slam_toolbox" || node == "amcl")))
    {
      continue;
    }

    change_state(node, transition);
  }
}

void NavigationSystem::reset_localization()
{
  reset_node("map_server");

  if (mode_.id == navigation_system_interfaces::msg::Mode::AMCL) {
    reset_node("amcl");
  } else if (mode_.id == navigation_system_interfaces::msg::Mode::SLAM) {
    reset_node("slam_toolbox");
  }
}

void NavigationSystem::change_state(const std::string & node_name, uint8_t transition)
{
  auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    message("service not available, waiting again...", 4);
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto result = client->async_send_request(request);
  message_transition(node_name, transition);
}

void NavigationSystem::handleSetMap(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<navigation_system_interfaces::srv::SetMap::Request> request,
  const std::shared_ptr<navigation_system_interfaces::srv::SetMap::Response> response)
{
  (void)request_header;

  std::string map_path = request->map_path;

  auto change_parameter_client = this->create_client<rcl_interfaces::srv::SetParameters>(
    "/map_server/set_parameters");

  while (!change_parameter_client->wait_for_service(std::chrono::seconds(1))) {
    message("service not available, waiting again...", 4);
  }

  auto request_service = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = "yaml_filename";
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  parameter.value.string_value = map_path;

  request_service->parameters.push_back(parameter);
  auto result = change_parameter_client->async_send_request(request_service);

  reset_localization();

  response->success = true;
}

void NavigationSystem::handleSetMode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<navigation_system_interfaces::srv::SetMode::Request> request,
  const std::shared_ptr<navigation_system_interfaces::srv::SetMode::Response> response)
{
  (void)request_header;

  navigation_system_interfaces::msg::Mode mode = request->mode;

  if (mode_.id == mode.id) {
    response->success = false;
    response->message = "Mode already set";
    return;
  }

  if (mode.id == navigation_system_interfaces::msg::Mode::AMCL &&
    mode_.id == navigation_system_interfaces::msg::Mode::SLAM)
  {
    clean_node("slam_toolbox");
    reset_node("map_server");
    startup_node("amcl");
    response->message = "Mode changed to AMCL";
  } else if (mode.id == navigation_system_interfaces::msg::Mode::SLAM &&
    mode_.id == navigation_system_interfaces::msg::Mode::AMCL)
  {
    clean_node("amcl");
    reset_node("map_server");
    startup_node("slam_toolbox");
    response->message = "Mode changed to SLAM";
  } else if (mode_.id == navigation_system_interfaces::msg::Mode::NO_MODE &&
    mode.id == navigation_system_interfaces::msg::Mode::SLAM)
  {
    startup_node("slam_toolbox");
    response->message = "Mode changed to SLAM";
  } else if (mode_.id == navigation_system_interfaces::msg::Mode::NO_MODE &&
    mode.id == navigation_system_interfaces::msg::Mode::AMCL)
  {
    startup_node("amcl");
    response->message = "Mode changed to AMCL";
  } else if (mode.id == navigation_system_interfaces::msg::Mode::NO_MODE &&
    mode_.id == navigation_system_interfaces::msg::Mode::SLAM)
  {
    clean_node("slam_toolbox");
    reset_node("map_server");
    response->message = "Mode changed to NO_MODE";
  } else if (mode.id == navigation_system_interfaces::msg::Mode::NO_MODE &&
    mode_.id == navigation_system_interfaces::msg::Mode::AMCL)
  {
    clean_node("amcl");
    reset_node("map_server");
    response->message = "Mode changed to NO_MODE";
  } else {
    response->success = false;
    response->message = "Mode not supported";
    return;
  }

  mode_ = mode;

  response->success = true;
}

void NavigationSystem::handleSetTruncateDistance(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<navigation_system_interfaces::srv::SetTruncateDistance::Request> request,
  const std::shared_ptr<navigation_system_interfaces::srv::SetTruncateDistance::Response> response)
{
  (void)request_header;

  response->xml_path = generate_xml_file(request->xml_content, request->distance);
  response->success = true;
}

void NavigationSystem::message_transition(const std::string & node_name, uint8_t transition)
{
  std::string transition_str;
  switch (transition) {
    case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
      transition_str = "Configuring";
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
      transition_str = "Cleaning";
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
      transition_str = "Activating";
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
      transition_str = "Deactivating";
      break;
    default:
      transition_str = "UNKNOWN";
      break;
  }

  message(transition_str + " " + node_name, 0);
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"
#define ANSI_COLOR_ORANGE   "\x1b[38;5;208m"
#define ANSI_COLOR_RED      "\x1b[31m"
#define ANSI_COLOR_GREEN    "\x1b[32m"

void
NavigationSystem::message(const std::string & msg, int level)
{
  switch (level) {
    case 0:
      RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
      break;
    case 1:
      RCLCPP_INFO(get_logger(), ANSI_COLOR_GREEN "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
      break;
    case 2:
      RCLCPP_DEBUG(get_logger(), ANSI_COLOR_ORANGE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
      break;
    case 3:
      RCLCPP_WARN(get_logger(), ANSI_COLOR_ORANGE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
      break;
    case 4:
      RCLCPP_ERROR(get_logger(), ANSI_COLOR_RED "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
      break;
    case 5:
      RCLCPP_FATAL(get_logger(), ANSI_COLOR_RED "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
      break;
    default:
      RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
      break;
  }
}

}  // namespace navigation_system
