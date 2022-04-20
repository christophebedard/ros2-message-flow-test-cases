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
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

#include "ros2_message_flow_testcases/utils.hpp"
#include "ros2_message_flow_testcases/sink.hpp"

class SinkNodeTimestamp : public SinkNode<std_msgs::msg::Int64>
{
public:
  SinkNodeTimestamp(const std::vector<char> & topics, const std::string & fileprefix)
  : SinkNode(topics, [this](const std_msgs::msg::Int64::ConstSharedPtr msg) { this->collect(msg); }),
    filename_(get_filename(fileprefix))
  {
    // Manual value: @ 1 kHz -> ~33 minutes
    timestamps_.reserve(2000000);

    RCLCPP_INFO(this->get_logger(), "Filename: %s", filename_.c_str());
  }

  ~SinkNodeTimestamp()
  {
    dump();
  }

  void collect(const std_msgs::msg::Int64::ConstSharedPtr msg)
  {
    // Compute time difference and add to vector
    timestamps_.push_back(now() - msg->data);
  }

private:
  static int64_t now()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }

  static std::string get_filename(const std::string & fileprefix)
  {
    return fileprefix + "_" + std::to_string(now()) + ".txt";
  }

  void dump()
  {
    // Print to file
    std::ofstream file;
    file.open(filename_);
    for (int64_t t : timestamps_) {
      file << t << std::endl;
    }
    file.close();
  }

  std::vector<int64_t> timestamps_;
  const std::string filename_;
};

int main(int argc, char * argv[])
{
  auto topics = utils::parse_single_topic_list(argc, argv);
  auto fileprefix = utils::parse_string(argc, argv, 2);
  if (!topics || !fileprefix) {
    std::cerr << "USAGE: ab fileprefix" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SinkNodeTimestamp>(topics.value(), fileprefix.value()));
  rclcpp::shutdown();
  return 0;
}
