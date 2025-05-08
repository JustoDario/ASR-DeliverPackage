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

#include "hri/dialog/listen.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;

Listen::Listen(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: hri::BtActionNode<whisper_msgs::action::STT, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  start_time_ = node_->get_clock()->now();

  listen_start_publisher_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_phase", 10);
}

void Listen::replaceAllOccurrences(std::string& str, const std::string& from, const std::string& to) {
    if (from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();
    }
}

std::string Listen::normalizeString(const std::string& inputStr) {
    std::string currentStr = inputStr;

    replaceAllOccurrences(currentStr, "\xc3\x81", "a"); replaceAllOccurrences(currentStr, "\xc3\xa1", "a");
    replaceAllOccurrences(currentStr, "\xc3\x89", "e"); replaceAllOccurrences(currentStr, "\xc3\xa9", "e");
    replaceAllOccurrences(currentStr, "\xc3\x8d", "i"); replaceAllOccurrences(currentStr, "\xc3\xad", "i");
    replaceAllOccurrences(currentStr, "\xc3\x93", "o"); replaceAllOccurrences(currentStr, "\xc3\xb3", "o");
    replaceAllOccurrences(currentStr, "\xc3\x9a", "u"); replaceAllOccurrences(currentStr, "\xc3\xba", "u");
    replaceAllOccurrences(currentStr, "\xc3\x9c", "u"); replaceAllOccurrences(currentStr, "\xc3\xbc", "u");
    replaceAllOccurrences(currentStr, "\xc3\x91", "n"); replaceAllOccurrences(currentStr, "\xc3\xb1", "n");

    std::string fullyLoweredStr;
    fullyLoweredStr.reserve(currentStr.length());
    for (char ch_orig : currentStr) {
        fullyLoweredStr += static_cast<char>(std::tolower(static_cast<unsigned char>(ch_orig)));
    }

    std::string phoneticallyProcessedStr;
    phoneticallyProcessedStr.reserve(fullyLoweredStr.length());
    for (size_t i = 0; i < fullyLoweredStr.length(); ++i) {
        char currentChar = fullyLoweredStr[i];
        char charToAdd = 0;
        if (currentChar == 'h') {}
        else if (currentChar == 'v') { charToAdd = 'b'; }
        else if (currentChar == 'z') { charToAdd = 's'; }
        else if (currentChar == 'c') {
            char nextChar = (i + 1 < fullyLoweredStr.length()) ? fullyLoweredStr[i+1] : '\0';
            if (nextChar == 'e' || nextChar == 'i') { charToAdd = 's'; }
            else { charToAdd = 'k'; }
        } else if (currentChar == 'q') {
            if (i + 1 < fullyLoweredStr.length() && fullyLoweredStr[i+1] == 'u') {
                char charAfterU = (i + 2 < fullyLoweredStr.length()) ? fullyLoweredStr[i+2] : '\0';
                if (charAfterU == 'e' || charAfterU == 'i') { charToAdd = 'k'; i++; }
                else { charToAdd = 'k'; i++;}
            } else { charToAdd = 'k'; }
        } else { charToAdd = currentChar; }
        if (charToAdd != 0) phoneticallyProcessedStr += charToAdd;
    }

    std::string finalResultStr;
    finalResultStr.reserve(phoneticallyProcessedStr.length());
    for (char ch : phoneticallyProcessedStr) {
        if (std::isalnum(static_cast<unsigned char>(ch))) {
            finalResultStr += ch;
        }
    }
    return finalResultStr;
}

double Listen::jaroSimilarity(const std::string& s1, const std::string& s2) {
    const int len1 = s1.length();
    const int len2 = s2.length();

    if (len1 == 0 || len2 == 0) {
        return (len1 == 0 && len2 == 0) ? 1.0 : 0.0;
    }

    int match_distance = std::max(0, static_cast<int>(std::floor(std::max(len1, len2) / 2.0)) - 1);

    std::vector<bool> s1_matches(len1, false);
    std::vector<bool> s2_matches(len2, false);

    int matches = 0;
    for (int i = 0; i < len1; ++i) {
        int start = std::max(0, i - match_distance);
        int end = std::min(i + match_distance + 1, len2);

        for (int j = start; j < end; ++j) {
            if (s2_matches[j]) continue;
            if (s1[i] != s2[j]) continue;

            s1_matches[i] = true;
            s2_matches[j] = true;
            matches++;
            break;
        }
    }

    if (matches == 0) {
        return 0.0;
    }

    std::string s1_matched_chars; s1_matched_chars.reserve(matches);
    for(int i=0; i < len1; ++i) if(s1_matches[i]) s1_matched_chars += s1[i];

    std::string s2_matched_chars; s2_matched_chars.reserve(matches);
    for(int i=0; i < len2; ++i) if(s2_matches[i]) s2_matched_chars += s2[i];

    int half_transpositions = 0;
    for (int i = 0; i < matches; ++i) {
        if (s1_matched_chars[i] != s2_matched_chars[i]) {
            half_transpositions++;
        }
    }
    int transpositions = half_transpositions / 2;

    double dj = ((double)matches / len1 +
                (double)matches / len2 +
                (double)(matches - transpositions) / matches) / 3.0;
    return dj;
}

double Listen::jaroWinklerSimilarity(const std::string& s1, const std::string& s2, double p_scaling_factor, int max_prefix_length) {
    if (s1.empty() && s2.empty()) return 1.0;
    if (s1.empty() || s2.empty()) return 0.0;

    double jaro_dist = jaroSimilarity(s1, s2);

    if (jaro_dist == 0.0) return 0.0;

    int common_prefix_len = 0;
    for (int i = 0; i < std::min({(int)s1.length(), (int)s2.length(), max_prefix_length}); ++i) {
        if (s1[i] == s2[i]) {
            common_prefix_len++;
        } else {
            break;
        }
    }
    return jaro_dist + common_prefix_len * p_scaling_factor * (1.0 - jaro_dist);
}

bool Listen::areSimilar(const std::string& str1, const std::string& str2, double threshold) {
    std::string normalizedStr1 = normalizeString(str1);
    std::string normalizedStr2 = normalizeString(str2);

    if (normalizedStr1.empty() && normalizedStr2.empty()) {
        return true;
    }

    double similarity = jaroWinklerSimilarity(normalizedStr1, normalizedStr2);

    return similarity >= threshold;
}
std::vector<std::string>
Listen::split_string(const std::string& s) {
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

void
Listen::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Listen ticked");
  rclcpp::Duration elapsed_time = node_->get_clock()->now() - start_time_;
  if (elapsed_time.seconds() > 5.0) {
    RCLCPP_WARN(node_->get_logger(), "Listen timed out after 5 seconds");
    halt();  // detiene el nodo si está corriendo
    setStatus(BT::NodeStatus::FAILURE);
    return;
  }
  goal_ = whisper_msgs::action::STT::Goal();
  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = START_LISTENING_;

  listen_start_publisher_->publish(msg_dialog_action);
}

BT::NodeStatus Listen::on_success()
{

  RCLCPP_INFO(node_->get_logger(), "I heard: %s", result_.result->transcription.text.c_str());

  if (result_.result->transcription.text.size() == 0) {
    return BT::NodeStatus::FAILURE;
  }
  std::vector<std::string> words = split_string(result_.result->transcription.text);
  for(std::string w : words) {
    if(areSimilar(w,"kobuki",0.7)) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  setOutput("listened_text", result_.result->transcription.text);

  return BT::NodeStatus::FAILURE;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::Listen>(name, "whisper/listen", config);
    };

  factory.registerBuilder<dialog::Listen>("Listen", builder);
}
