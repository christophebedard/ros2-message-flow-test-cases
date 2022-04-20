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
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tracetools/tracetools.h"

template<typename MessageT>
struct cached_sub_t
{
  typename rclcpp::Subscription<MessageT>::SharedPtr sub;
  typename MessageT::SharedPtr cache;
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
template<typename MessageT>
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
        cached_sub_t<MessageT>{
          this->create_subscription<MessageT>(
            std::string("topic_") + sub_topic,
            10,
            [this, sub_i](const typename MessageT::SharedPtr msg) {
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
        this->create_publisher<MessageT>(std::string("topic_") + pub_topic, 10));
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
    auto next_msg = MessageT();
    // next_msg.data = msg->data + ", and another thing";
    // next_msg = *msg;
    for (const auto & sub : subs_) {
      next_msg.data += sub.cache->data;
    }
    // Publish
    for (const auto & pub : pubs_) {
      pub->publish(next_msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<cached_sub_t<MessageT>> subs_;
  std::vector<typename rclcpp::Publisher<MessageT>::SharedPtr> pubs_;
};
