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

#ifndef PERCEPTION__FILTER_ENTITY_HPP_
#define PERCEPTION__FILTER_ENTITY_HPP_

#include <limits>
#include <string>
#include <utility>
#include <algorithm>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "perception_system/PerceptionUtils.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace perception
{

using namespace std::chrono_literals;

class FilterEntity : public BT::ActionNodeBase
{
public:
  explicit FilterEntity(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("frame"),
        BT::InputPort<float>("lambda", "filtering parameter")
      });
  }

private:
  geometry_msgs::msg::TransformStamped initialize_state_observer(
    const geometry_msgs::msg::TransformStamped & entity);
  geometry_msgs::msg::TransformStamped update_state_observer(
    const geometry_msgs::msg::TransformStamped & entity);
  
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::string frame_;
  double lambda_;
  bool state_obs_initialized_ = false;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::TransformStamped filtered_entity_;
  
};

}  // namespace perception

#endif  // PERCEPTION__FILTER_ENTITY_HPP_
