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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_message_flow_testcases/utils.hpp"
#include "ros2_message_flow_testcases/partial_sync_n_to_m.hpp"

int main(int argc, char * argv[])
{
  auto topics = utils::parse_topic_list_pair(argc, argv);
  if (!topics) {
    std::cerr << "USAGE: ab c" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PartialSyncNtoMNode<std_msgs::msg::String>>(topics.value()));
  rclcpp::shutdown();
  return 0;
}
