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

#ifndef ROS2_MESSAGE_FLOW_TESTCASES__UTILS_HPP_
#define ROS2_MESSAGE_FLOW_TESTCASES__UTILS_HPP_

#include <optional>  // NOLINT cpplint mistakes <optional> for a C system header
#include <string>
#include <utility>
#include <vector>

namespace utils
{

std::optional<std::pair<std::vector<char>, std::vector<char>>>
parse_topic_list_pair(int argc, char * argv[]);

std::optional<std::vector<char>>
parse_single_topic_list(int argc, char * argv[]);

/**
 * Parse a single topic pair.
 */
std::optional<std::pair<char, char>>
parse_single_topic_pair(int argc, char * argv[]);

std::optional<std::pair<char, std::vector<char>>>
parse_single_topic_and_list(int argc, char * argv[]);

std::optional<uint32_t>
parse_period(int argc, char * argv[], const uint16_t num_args_before);

std::optional<std::string>
parse_string(int argc, char * argv[], int pos);

}  // namespace utils

#endif  // ROS2_MESSAGE_FLOW_TESTCASES__UTILS_HPP_
