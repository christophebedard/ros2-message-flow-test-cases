// Copyright 2021 Christophe Bedard
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

#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tracetools/tracetools.h"

#include "ros2_message_flow_testcases/utils.hpp"

struct cached_sub_t
{
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  std_msgs::msg::String::SharedPtr cache;
  bool empty;
};

/**
 * 2-to-N
 * Node that subscribes to 2 topics and publishes N topics.
 * New messages are cached.
 * Publishes on N topics as soon as 2 new messages have been received.
 *
 * Based on:
 * https://github.com/ros-realtime/reference-system/blob/main/reference_system/include/reference_system/nodes/rclcpp/fusion.hpp
 */
class TwoToNNode : public rclcpp::Node
{
public:
  TwoToNNode(const std::pair<std::pair<char, char>, std::vector<char>> & topics)
  : Node("two_to_n")
  {
    std::string info = "2-to-N topics: ";
    uint32_t sub_i = 0;
    for (const auto & sub_topic : {topics.first.first, topics.first.second}) {
      info += sub_topic;
      subs_[sub_i] =
        cached_sub_t{
          this->create_subscription<std_msgs::msg::String>(
            std::string("topic_") + sub_topic,
            10,
            [this, sub_i](const std_msgs::msg::String::SharedPtr msg) {
              this->sub_callback(msg, sub_i);
            }),
          nullptr,
          true};
      sub_i++;
    }
    info += "-";
    for (const auto & pub_topic : topics.second) {
      info += pub_topic;
      pubs_.push_back(
        this->create_publisher<std_msgs::msg::String>(std::string("topic_") + pub_topic, 10));
    }
    RCLCPP_INFO(this->get_logger(), "%s", info.c_str());

    // Annotate message links
    std::vector<const void *> link_subs;
    std::vector<const void *> link_pubs;
    std::transform(
      subs_.cbegin(),
      subs_.cend(),
      std::back_inserter(link_subs),
      [](const auto & s){ return static_cast<const void *>(s.sub->get_subscription_handle().get()); });
    std::transform(
      pubs_.cbegin(),
      pubs_.cend(),
      std::back_inserter(link_pubs),
      [](const auto & p){ return static_cast<const void *>(p->get_publisher_handle().get()); });
    TRACEPOINT(message_link_2_to_n, link_subs.data(), link_pubs.data(), link_pubs.size());
  }

private:
  void sub_callback(const std_msgs::msg::String::SharedPtr msg, const uint32_t sub_i)
  {
    // Put message in corresponding cache
    subs_[sub_i].cache = msg;
    subs_[sub_i].empty = false;

    // Unless one of the caches is empty (hasn't received a new message yet)...
    if (subs_[0].empty || subs_[1].empty) {
      return;
    }
    // Compute something based on messages in cache
    // Note that actually using the messages to compute a result isn't actually necessary here
    std::string result = "";
    for (const auto & sub : subs_) {
      result += sub.cache->data;
    }
    auto next_msg = std_msgs::msg::String();
    next_msg.data = result;
    // Publish
    for (const auto & pub : pubs_) {
      pub->publish(next_msg);
    }
    // Reset caches
    subs_[0].empty = true;
    subs_[1].empty = true;
  }

  std::array<cached_sub_t, 2> subs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
};

int main(int argc, char * argv[])
{
  auto topics = utils::parse_topic_pair_and_list(argc, argv);
  if (!topics) {
    std::cerr << "USAGE: ab c" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwoToNNode>(topics.value()));
  rclcpp::shutdown();
  return 0;
}
