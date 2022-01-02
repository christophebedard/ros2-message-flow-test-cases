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

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_message_flow_testcases/utils.hpp"

/**
 * 1-to-N
 * Node that subscribes to a single topic and publishes N topics.
 * New messages are published directly on N topics on every new message that is received.
 * This link type does not require any instrumentation.
 */
class OneToNNode : public rclcpp::Node
{
public:
  OneToNNode(const std::pair<char, std::vector<char>> & topics)
  : Node("one_to_n")
  {
    std::string info = std::string("1-to-N topics: ") + topics.first + "-";

    sub_ = this->create_subscription<std_msgs::msg::String>(
      std::string("topic_") + topics.first,
      10,
      std::bind(&OneToNNode::sub_callback, this, std::placeholders::_1));

    for (const auto & topic : topics.second) {
      info += topic;
      pubs_.push_back(
        this->create_publisher<std_msgs::msg::String>(std::string("topic_") + topic, 10));
    }
    RCLCPP_INFO(this->get_logger(), "%s", info.c_str());
  }

private:
  void sub_callback(const std_msgs::msg::String::ConstSharedPtr msg) const
  {
    auto next_msg = std_msgs::msg::String();
    next_msg.data = msg->data + ", and another thing";
    for (const auto & pub : pubs_) {
      pub->publish(next_msg);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
};

int main(int argc, char * argv[])
{
  auto topics = utils::parse_single_topic_and_list(argc, argv);
  if (!topics) {
    std::cerr << "USAGE: a bc" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OneToNNode>(topics.value()));
  rclcpp::shutdown();
  return 0;
}
