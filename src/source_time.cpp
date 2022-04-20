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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

#include "ros2_message_flow_testcases/utils.hpp"
#include "ros2_message_flow_testcases/source.hpp"

int main(int argc, char * argv[])
{
  auto topics = utils::parse_single_topic_list(argc, argv);
  auto period = utils::parse_period(argc, argv, 1);
  if (!topics || !period) {
    std::cerr << "USAGE: ab 100" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SourceNode<std_msgs::msg::Int64>>(
    topics.value(),
    period.value(),
    [](){
      auto msg = std_msgs::msg::Int64();
      msg.data = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      return msg;
    }));
  rclcpp::shutdown();
  return 0;
}
