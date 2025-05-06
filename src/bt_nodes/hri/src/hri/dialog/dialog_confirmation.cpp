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

#include "hri/dialog/dialog_confirmation.hpp"



namespace hri
{

using namespace std::chrono_literals;
using namespace std::placeholders;

DialogConfirmation::DialogConfirmation(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: hri::BtActionNode<whisper_msgs::action::STT, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  speech_start_publisher_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);

  getInput("language", lang_);
  getInput("mode", mode_);
  getInput("password",pswrd_);
}

void DialogConfirmation::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "DialogConfirmation ticked");
  std::string text;

  goal_ = whisper_msgs::action::STT::Goal();
  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = START_LISTENING_;

  speech_start_publisher_->publish(msg_dialog_action);
}

int
DialogConfirmation::count_words(const std::string& s) {
  int counter = 0;
  bool inside_wrd = false;

  for (char c : s) {
    if (c != ' ' && !inside_wrd) {
      inside_wrd = true;
      counter++;
    } else if (c == ' ') {
      inside_wrd = false;
    }
  }
  return counter;
}

bool
DialogConfirmation::is_yes(std::string& s)
{
  bool previous_was_s = false;
  for(char c : s) {
    if(previous_was_s) {
      if(c == static_cast<unsigned char>('í') || c=='i'){
        return true;
      }
    }
    else {
      if(c=='s'){
        previous_was_s = true;
      }
    }
  }
  return false;
}

std::vector<std::string>
DialogConfirmation::split_string(const std::string& s) {
  std::vector<std::string> words;
  std::string current_word;
  bool inside_word = false;
  
  for (char c : s) {
    if (c != ' ' && !inside_word) {
      inside_word = true;
      current_word = c;
    } else if (c != ' ' && inside_word) {
      current_word += c;
    } else if (c == ' ') {
      if (inside_word) {
        words.push_back(current_word);
        current_word = "";
      }
      inside_word = false;
    }
  }
  
  if (inside_word) {
    words.push_back(current_word);
  }
  
  return words;
}

std::string normalize(const std::string &s) {
    std::string t;
    t.reserve(s.size());
    for (char c : s) {
        if (!std::isspace(static_cast<unsigned char>(c)))
            t += std::tolower(static_cast<unsigned char>(c));
    }
    return t;
}

int levenshtein(const std::string &s, const std::string &t) {
    int n = s.size(), m = t.size();
    if (n == 0) return m;
    if (m == 0) return n;
    std::vector<std::vector<int>> dp(n+1, std::vector<int>(m+1));
    for (int i = 0; i <= n; ++i) dp[i][0] = i;
    for (int j = 0; j <= m; ++j) dp[0][j] = j;
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= m; ++j) {
            int cost = (s[i-1] == t[j-1] ? 0 : 1);
            dp[i][j] = std::min({
                dp[i-1][j] + 1,       // borrado
                dp[i][j-1] + 1,       // inserción
                dp[i-1][j-1] + cost   // sustitución
            });
        }
    }
    return dp[n][m];
}

// threshold: porcentaje máximo de errores
bool fuzzyEqual(const std::string &a, const std::string &b, double threshold = 0.2) {
    std::string A = normalize(a);
    std::string B = normalize(b);
    int dist = levenshtein(A, B);
    int maxLen = std::max<int>(A.size(), B.size());
    if (maxLen == 0) return true;  
    double ratio = double(dist) / maxLen;
    return ratio <= threshold;
}

BT::NodeStatus DialogConfirmation::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "I heard: %s", result_.result->transcription.text.c_str());

  if (result_.result->transcription.text.size() == 0) {
    return BT::NodeStatus::FAILURE;
  }

  std::transform(
    result_.result->transcription.text.begin(), result_.result->transcription.text.end(), result_.result->transcription.text.begin(),
    [](unsigned char c) {return std::tolower(c);});
  
  std::string yes_word = "yes";
  if (lang_ == "es") {
    yes_word = "sí";
  }
  if (mode_ == "set_dest") {
    float x;
    float y;
    std::vector<std::string> points = { "laboratorio", "pasillo", "aula"};
    for(int i = 0; i < 3; i++) {
      if (result_.result->transcription.text.find(points[i]) != std::string::npos) {
        switch(i) {
          case 0:
            x = 1.0;
            y = 1.0;//Cambiar
            break;
          case 1:
            x = 2.0;
            y = 2.0;
            break;
          case 2:
            x = 3.0;
            y = 3.0;
            break;
        }
        setOutput("cordx", x);
        setOutput("cordy", y);
        setOutput("heard_text", result_.result->transcription.text);
        return BT::NodeStatus::SUCCESS;
      }
    }
  } else if (mode_ == "set_password") {
    if(count_words(result_.result->transcription.text) == 2) {
      std::vector<std::string> words = split_string(result_.result->transcription.text);
      setOutput("name", words[1]);
      setOutput("password", words[2]);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  } else if (mode_ == "check_password") {
    if (fuzzyEqual(result_.result->transcription.text, pswrd_, 0.4)) {
      return BT::NodeStatus::SUCCESS; 
    } else {
      return BT::NodeStatus::FAILURE; // (igual) poner traza pa saber que ha escuchado
    }
  } else if (mode_ == "receive/give_pkg") {
    if (result_.result->transcription.text.find(yes_word) != std::string::npos) {
      return BT::NodeStatus::SUCCESS;
    } else  if(is_yes(result_.result->transcription.text)){
      return BT::NodeStatus::SUCCESS;
    } else if(result_.result->transcription.text.find("sí") != std::string::npos) {
      return BT::NodeStatus::SUCCESS;
    }
    else if(result_.result->transcription.text.find("ya") != std::string::npos) {
      return BT::NodeStatus::SUCCESS;
    }
    fprintf(stderr,"Texto convertido: %s", result_.result->transcription.text.c_str());

    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace hri
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<hri::DialogConfirmation>(name, "/whisper/listen", config);
    };

  factory.registerBuilder<hri::DialogConfirmation>("DialogConfirmation", builder);
}
