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
#include <vector>

#include "rclcpp/rclcpp.hpp"

/**
 * timer-to-N
 * Source node that only publishes.
 * Publishes on N topics periodically.
 */
template<typename MessageT>
class SourceNode : public rclcpp::Node
{
public:
  SourceNode(const std::vector<char> & topics, const uint32_t period, std::function<MessageT()> msg_gen)
  : Node("source"),
    pubs_(),
    msg_gen_(msg_gen)
  {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period),
      std::bind(&SourceNode::timer_callback, this));

    std::string info = "Source topics: ";
    for (const auto & topic : topics) {
      info += topic;
      pubs_.push_back(
        this->create_publisher<MessageT>(std::string("topic_") + topic, 10));
    }
    RCLCPP_INFO(this->get_logger(), "%s @ %d ms", info.c_str(), period);
  }

private:
  void timer_callback()
  {
    for (const auto & pub : pubs_) {
      auto msg = MessageT();
      msg = msg_gen_();
      pub->publish(msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<typename rclcpp::Publisher<MessageT>::SharedPtr> pubs_;
  std::function<MessageT()> msg_gen_;
};
