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

#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

/**
 * synchronous 1-to-N
 * Node that subscribes to a single topic and publishes N topics.
 * New messages are published directly on N topics on every new message that is received.
 * This link type does not require any instrumentation.
 */
template<typename MessageT>
class SyncOneToNNode : public rclcpp::Node
{
public:
  SyncOneToNNode(const std::pair<char, std::vector<char>> & topics)
  : Node("sync_one_to_n")
  {
    std::string info = std::string("Sync 1-to-N topics: ") + topics.first + "-";

    sub_ = this->create_subscription<MessageT>(
      std::string("topic_") + topics.first,
      10,
      std::bind(&SyncOneToNNode::sub_callback, this, std::placeholders::_1));

    for (const auto & topic : topics.second) {
      info += topic;
      pubs_.push_back(
        this->create_publisher<MessageT>(std::string("topic_") + topic, 10));
    }
    RCLCPP_INFO(this->get_logger(), "%s", info.c_str());
  }

private:
  void sub_callback(const typename MessageT::ConstSharedPtr msg) const
  {
    auto next_msg = MessageT();
    // next_msg.data = msg->data + ", and another thing";
    next_msg = *msg;
    for (const auto & pub : pubs_) {
      pub->publish(next_msg);
    }
  }

  typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
  std::vector<typename rclcpp::Publisher<MessageT>::SharedPtr> pubs_;
};
