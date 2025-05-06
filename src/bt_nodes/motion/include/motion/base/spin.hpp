// Copyright 2024 Intelligent Robotics Lab
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

#ifndef BASE_ROTATE_HPP_
#define BASE_ROTATE_HPP_

#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace base
{

class Spin : public BT::ActionNodeBase
{
public:
  explicit Spin(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<double>("angle"), // if negative, rotate right, if positive, rotate left
        BT::InputPort<double>("speed"), // rad/s
        BT::InputPort<bool>("forever") // if true, keep rotating
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  double angle_, rotated_angle_, speed_;
  bool forever_;
  // int direction_;
  
  std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
};

}  // namespace base

#endif  // BASE_ROTATE_HPP_
