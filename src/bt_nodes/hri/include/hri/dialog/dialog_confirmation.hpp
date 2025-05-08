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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef HRI__DIALOGCONFIRMATION_HPP_
#define HRI__DIALOGCONFIRMATION_HPP_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <cctype>
#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ctrl_support/BTActionNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/int8.hpp"
#include "whisper_msgs/action/stt.hpp"

#include "std_msgs/msg/int8.hpp"

namespace hri
{

class DialogConfirmation
  : public hri::BtActionNode<
    whisper_msgs::action::STT, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit DialogConfirmation(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("language","Language to use (es)"), // es/en
        BT::InputPort<std::string>("password","contraseña recibida para guardarla"),
        BT::InputPort<std::string>("mode", "Modo de dialogo (set_dest),(set_password),(check_password),(receive/give_pkg)"),
        BT::OutputPort<std::string>("heard_text","Text heard from the user"),
        BT::OutputPort<double>("cordx","Cord x to deliver the package"),
        BT::OutputPort<double>("cordy", "Cord y to deliver the package"),
        BT::OutputPort<std::string>("password","'publicador' de la contraseña"),
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr speech_start_publisher_;
  std::vector<std::string> split_string(const std::string& s);
  
  // New string comparison methods
  void replaceAllOccurrences(std::string& str, const std::string& from, const std::string& to);
  std::string normalizeString(const std::string& inputStr);
  double jaroSimilarity(const std::string& s1, const std::string& s2);
  double jaroWinklerSimilarity(const std::string& s1, const std::string& s2, double p_scaling_factor = 0.1, int max_prefix_length = 4);
  bool areSimilar(const std::string& str1, const std::string& str2);
  
  int count_words(const std::string& s);
  std::string mode_;
  std::string pswrd_;
  std::string lang_;
  const int START_LISTENING_{0};
};

}  // namespace dialog

#endif  // HRI__DIALOGCONFIRMATION_HPP_
