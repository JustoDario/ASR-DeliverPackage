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

#ifndef NAVIGATION_SYSTEM__NAVIGATION_SYSTEM_HPP_
#define NAVIGATION_SYSTEM__NAVIGATION_SYSTEM_HPP_


#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "navigation_system/utils.hpp"
#include "navigation_system_interfaces/srv/set_map.hpp"
#include "navigation_system_interfaces/srv/set_mode.hpp"
#include "navigation_system_interfaces/srv/set_truncate_distance.hpp"
#include "navigation_system_interfaces/msg/mode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace navigation_system
{

class NavigationSystem : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  NavigationSystem(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  void startup();
  void reset();

private:
  void set_initial_mode(const std::string & mode);
  void start_services();
  void startup_node(const std::string & node_name);
  void clean_node(const std::string & node_name);
  void reset_node(const std::string & node_name);
  void change_all_nodes(uint8_t transition);
  void reset_localization();
  void change_state(const std::string & node_name, uint8_t transition);
  void message_transition(const std::string & node_name, uint8_t transition);
  void message(const std::string & message, int level);

  void handleSetMap(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navigation_system_interfaces::srv::SetMap::Request> request,
    const std::shared_ptr<navigation_system_interfaces::srv::SetMap::Response> response);

  void handleSetMode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navigation_system_interfaces::srv::SetMode::Request> request,
    const std::shared_ptr<navigation_system_interfaces::srv::SetMode::Response> response);

  void handleSetTruncateDistance(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navigation_system_interfaces::srv::SetTruncateDistance::Request> request,
    const std::shared_ptr<navigation_system_interfaces::srv::SetTruncateDistance::Response> response);

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr change_parameter_client_;
  rclcpp::Service<navigation_system_interfaces::srv::SetMap>::SharedPtr set_map_service_;
  rclcpp::Service<navigation_system_interfaces::srv::SetMode>::SharedPtr set_mode_service_;
  rclcpp::Service<navigation_system_interfaces::srv::SetTruncateDistance>::SharedPtr
    set_truncate_distance_service_;

  std::vector<std::string> nodes_;
  navigation_system_interfaces::msg::Mode mode_;
};

}  // namespace navigation_system

#endif  // NAVIGATION_SYSTEM__NAVIGATION_SYSTEM_HPP_
