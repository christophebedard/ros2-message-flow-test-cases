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
 * Periodic asynchronous N-to-M
 * Node that subscribes to a N topics and publishes M topics.
 * New messages are cached.
 * Publishes on M topics periodically.
 *
 * Based on the cyclic node:
 * https://github.com/ros-realtime/reference-system/blob/main/reference_system/include/reference_system/nodes/rclcpp/cyclic.hpp
 */
class PeriodicAsyncNToMNode : public rclcpp::Node
{
public:
  PeriodicAsyncNToMNode(const std::pair<std::vector<char>, std::vector<char>> & topics, const uint32_t period)
  : Node("periodic_async_n_to_m")
  {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period),
      std::bind(&PeriodicAsyncNToMNode::timer_callback, this));

    std::string info = "Periodic async N-to-M topics: ";
    uint32_t sub_i = 0;
    for (const auto & sub_topic : topics.first) {
      info += sub_topic;
      subs_.push_back(
        cached_sub_t{
          this->create_subscription<std_msgs::msg::String>(
            std::string("topic_") + sub_topic,
            10,
            [this, sub_i](const std_msgs::msg::String::SharedPtr msg) {
              subs_[sub_i].cache = msg;
              subs_[sub_i].empty = false;
            }),
          nullptr,
          true});
      sub_i++;
    }
    info += "-";
    for (const auto & pub_topic : topics.second) {
      info += pub_topic;
      pubs_.push_back(
        this->create_publisher<std_msgs::msg::String>(std::string("topic_") + pub_topic, 10));
    }
    RCLCPP_INFO(this->get_logger(), "%s @ %d ms", info.c_str(), period);

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
    TRACEPOINT(message_link_periodic_async, link_subs.data(), link_subs.size(), link_pubs.data(), link_pubs.size());
  }

private:
  void timer_callback()
  {
    // Unless one of the caches is empty (hasn't received a message yet)...
    if (std::any_of(subs_.cbegin(), subs_.cend(), [](const auto & s){ return s.empty; })) {
      return;
    }
    // Compute something based on messages in cache
    // Note that actually using the messages to compute a result isn't actually necessary here
    std::string result = "";
    for (const auto & sub : subs_) {
      result += sub.cache->data;
    }
    auto msg = std_msgs::msg::String();
    msg.data = result;
    // Publish
    for (const auto & pub : pubs_) {
      pub->publish(msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<cached_sub_t> subs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
};

int main(int argc, char * argv[])
{
  auto topics = utils::parse_topic_list_pair(argc, argv);
  auto period = utils::parse_period(argc, argv, 2);
  if (!topics || !period) {
    std::cerr << "USAGE: ab cd 100" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeriodicAsyncNToMNode>(topics.value(), period.value()));
  rclcpp::shutdown();
  return 0;
}
