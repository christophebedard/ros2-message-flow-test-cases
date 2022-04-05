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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_message_flow_testcases/utils.hpp"

/**
 * synchronous 1-to-1
 * Node that subscribes to a single topic and publishes a single topic.
 * A new message is published directly on every new message that is received.
 * This link type does not require any instrumentation.
 */
class SyncOneToOneNode : public rclcpp::Node
{
public:
  SyncOneToOneNode(const std::pair<char, char> & topics)
  : Node("sync_one_to_one")
  {
    RCLCPP_INFO(this->get_logger(), "Sync 1-to-1 topics: %c-%c", topics.first, topics.second);
    sub_ = this->create_subscription<std_msgs::msg::String>(
      std::string("topic_") + topics.first,
      10,
      std::bind(&SyncOneToOneNode::sub_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::String>(
      std::string("topic_") + topics.second, 10);
  }

private:
  void sub_callback(const std_msgs::msg::String::ConstSharedPtr msg) const
  {
    auto next_msg = std_msgs::msg::String();
    next_msg.data = msg->data + ", and another thing";
    pub_->publish(next_msg);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  auto topics = utils::parse_single_topic_pair(argc, argv);
  if (!topics) {
    std::cerr << "USAGE: a b" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncOneToOneNode>(topics.value()));
  rclcpp::shutdown();
  return 0;
}
