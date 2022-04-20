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
#include <optional>
#include <utility>

#include "rclcpp/rclcpp.hpp"

/**
 * synchronous 1-to-1
 * Node that subscribes to a single topic and publishes a single topic.
 * A new message is published directly on every new message that is received.
 * This link type does not require any instrumentation.
 */
template<typename MessageT>
class SyncOneToOneNode : public rclcpp::Node
{
public:
  SyncOneToOneNode(const std::pair<char, char> & topics, std::optional<std::function<void()>> work_function = std::nullopt)
  : Node("sync_one_to_one"),
    work_function_(work_function)
  {
    RCLCPP_INFO(this->get_logger(), "Sync 1-to-1 topics: %c-%c", topics.first, topics.second);
    sub_ = this->create_subscription<MessageT>(
      std::string("topic_") + topics.first,
      10,
      std::bind(&SyncOneToOneNode::sub_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<MessageT>(
      std::string("topic_") + topics.second, 10);
  }

private:
  void sub_callback(const typename MessageT::ConstSharedPtr msg) const
  {
    if (work_function_.has_value()) {
      work_function_.value()();
    }
    auto next_msg = MessageT();
    // next_msg.data = msg->data + ", and another thing";
    next_msg = *msg;
    pub_->publish(next_msg);
  }

  typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
  typename rclcpp::Publisher<MessageT>::SharedPtr pub_;
  std::optional<std::function<void()>> work_function_;
};
