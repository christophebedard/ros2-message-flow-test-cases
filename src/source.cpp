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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_message_flow_testcases/utils.hpp"

using namespace std::chrono_literals;

/**
 * timer-to-N
 * Source node that only publishes.
 * Publishes on N topics periodically.
 */
class SourceNode : public rclcpp::Node
{
public:
  SourceNode(const std::vector<char> & topics, const uint32_t period)
  : Node("source"),
    pubs_()
  {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period),
      std::bind(&SourceNode::timer_callback, this));

    std::string info = "Source topics: ";
    for (const auto & topic : topics) {
      info += topic;
      pubs_.push_back(
        this->create_publisher<std_msgs::msg::String>(std::string("topic_") + topic, 10));
    }
    RCLCPP_INFO(this->get_logger(), "%s @ %d ms", info.c_str(), period);
  }

private:
  void timer_callback()
  {
    for (const auto & pub : pubs_) {
      auto msg = std_msgs::msg::String();
      msg.data = "Some sensor data";
      pub->publish(msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
};

int main(int argc, char * argv[])
{
  auto topics = utils::parse_single_topic_list(argc, argv);
  auto period = utils::parse_period(argc, argv, 1);
  if (!topics || !period) {
    std::cerr << "USAGE: ab 100" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SourceNode>(topics.value(), period.value()));
  rclcpp::shutdown();
  return 0;
}
