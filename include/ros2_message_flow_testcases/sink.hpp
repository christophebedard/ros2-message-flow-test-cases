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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * N-sub
 * Sink node that only subscribes.
 * Subscribes to N topics.
 */
template<typename MessageT>
class SinkNode : public rclcpp::Node
{
public:
  SinkNode(const std::vector<char> & topics, std::optional<std::function<void(const typename MessageT::ConstSharedPtr)>> callback = std::nullopt)
  : Node("sink"),
    subs_()
  {
    auto cb = callback.value_or(std::bind(&SinkNode::sub_callback, this, std::placeholders::_1));

    std::string info = "Sink topics: ";
    for (const auto & topic : topics) {
      info += topic;
      subs_.push_back(
        this->create_subscription<MessageT>(
          std::string("topic_") + topic,
          10,
          // std::bind(&SinkNode::sub_callback, this, std::placeholders::_1)));
          cb));
    }
    RCLCPP_INFO(this->get_logger(), "%s", info.c_str());
  }

protected:
  void sub_callback(const typename MessageT::ConstSharedPtr msg) const
  {
    if constexpr (std::is_same_v<std_msgs::msg::String, MessageT>) {
      RCLCPP_INFO(this->get_logger(), "received: '%s'", msg->data.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "received: '%s'", std::to_string(msg->data).c_str());
    }
  }

private:
  std::vector<typename rclcpp::Subscription<MessageT>::SharedPtr> subs_;
};
