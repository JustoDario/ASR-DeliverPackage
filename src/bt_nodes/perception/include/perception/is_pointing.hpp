// Copyright 2024 Intelligent Robotics Lab - Gentlebots
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

#ifndef PERCEPTION__IS_POINTING_HPP_
#define PERCEPTION__IS_POINTING_HPP_

#include <algorithm>
#include <string>
#include <string>
#include <utility>


#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_system/PerceptionListener.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"
#include "perception_system_interfaces/msg/detection.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"


namespace perception
{

class IsPointing : public BT::ConditionNode
{
public:
  explicit IsPointing(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<int>("low_pointing_limit"),
        BT::InputPort<int>("high_pointing_limit"),
        BT::InputPort<float>("threshold"),
        BT::OutputPort<std::shared_ptr<perception_system_interfaces::msg::Detection>>("detection"),
        BT::OutputPort<int>("pointing_direction"),
        BT::OutputPort<std::string>("output_frame")
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::string output_frame_ = "someone_pointing";
  int low_pointing_limit_, high_pointing_limit_;
  int pointing_direction_;
  float threshold_ = 0.6;

 };

}  // namespace perception

#endif  // PERCEPTION__IS_POINTING_HPP_
