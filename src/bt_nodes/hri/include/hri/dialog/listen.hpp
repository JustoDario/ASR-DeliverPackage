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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef HRI__LISTEN_HPP_
#define HRI__LISTEN_HPP_

#include <algorithm>
#include <cstdint>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ctrl_support/BTActionNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/int8.hpp"
#include "whisper_msgs/action/stt.hpp"


namespace dialog
{

class Listen : public hri::BtActionNode<
    whisper_msgs::action::STT, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit Listen(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({BT::OutputPort<std::string>("listened_text")});
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr listen_start_publisher_; // To indicate the robot is listening
  void replaceAllOccurrences(std::string& str, const std::string& from, const std::string& to);
  std::string normalizeString(const std::string& inputStr);
  double jaroSimilarity(const std::string& s1, const std::string& s2);
  double jaroWinklerSimilarity(const std::string& s1, const std::string& s2, double p_scaling_factor = 0.1, int max_prefix_length = 4);
  bool areSimilar(const std::string& str1, const std::string& str2, double threshold = 0.80);
  
  const int START_LISTENING_{0};
  rclcpp::Time start_time_;
  std::vector<std::string> split_string(const std::string& s);

};

}  // namespace dialog

#endif  // HRI__LISTEN_HPP_
