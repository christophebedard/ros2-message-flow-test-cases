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

// #include <iostream>
#include <optional>  // NOLINT cpplint mistakes <optional> for a C system header
#include <string>
#include <vector>

// #include "rcpputils/split.hpp"

#include "ros2_message_flow_testcases/utils.hpp"

namespace utils
{

std::optional<std::pair<std::vector<char>, std::vector<char>>>
parse_topic_list_pair(int argc, char * argv[])
{
  if (argc < 3) {
    return std::nullopt;
  }

  std::string list1(argv[1]);
  std::string list2(argv[2]);
  if (list1.empty() || list2.empty()) {
    return std::nullopt;
  }
  return std::make_pair(
    std::vector<char>(list1.cbegin(), list1.cend()),
    std::vector<char>(list2.cbegin(), list2.cend()));
}

std::optional<std::pair<char, std::vector<char>>>
parse_single_topic_and_list(int argc, char * argv[])
{
  auto topic_list_pair_opt = parse_topic_list_pair(argc, argv);
  if (!topic_list_pair_opt) {
    return std::nullopt;
  }
  auto topic_list_pair = topic_list_pair_opt.value();

  if (topic_list_pair.first.size() != 1) {
    return std::nullopt;
  }
  return std::make_pair(topic_list_pair.first[0], topic_list_pair.second);
}

std::optional<std::pair<char, char>>
parse_single_topic_pair(int argc, char * argv[])
{
  auto single_topic_and_list_opt = parse_single_topic_and_list(argc, argv);
  if (!single_topic_and_list_opt) {
    return std::nullopt;
  }
  auto single_topic_and_list = single_topic_and_list_opt.value();

  if (single_topic_and_list.second.size() != 1) {
    return std::nullopt;
  }
  return std::make_pair(single_topic_and_list.first, single_topic_and_list.second[0]);
}

std::optional<std::vector<char>>
parse_single_topic_list(int argc, char * argv[])
{
  if (argc < 2) {
    return std::nullopt;
  }

  std::string list(argv[1]);
  return std::vector<char>(list.cbegin(), list.cend());
}

std::optional<uint32_t>
optional_stoul(const std::string & str)
{
  try {
    return static_cast<uint32_t>(std::stoul(str));
  } catch (const std::invalid_argument &) {
  } catch (const std::out_of_range &) {  // LCOV_EXCL_LINE
  } catch (const std::exception &) {  // LCOV_EXCL_LINE
  } catch (...) {  // LCOV_EXCL_LINE
  }
  return std::nullopt;
}

std::optional<uint32_t>
parse_period(int argc, char * argv[], const uint16_t num_args_before)
{
  if (argc < 2 + num_args_before) {
    return std::nullopt;
  }

  std::string str(argv[num_args_before + 1]);
  if (str.empty()) {
    return std::nullopt;
  }
  return optional_stoul(str);
}

}  // namespace utils
