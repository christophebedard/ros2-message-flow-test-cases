# Copyright 2021 Christophe Bedard
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file for the 1-to-N message link type."""

from launch import LaunchDescription
from launch_ros.actions import Node
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    return LaunchDescription([
        Trace(
            session_name='single_1-to-n',
            events_kernel=[],
            events_ust=[
                'dds:*',
            ] + DEFAULT_EVENTS_ROS,
        ),
        Node(
            package='ros2_message_flow_testcases',
            executable='source',
            arguments=['a', '100'],
            output='screen',
        ),
        Node(
            package='ros2_message_flow_testcases',
            executable='one_to_n',
            arguments=['a', 'bc'],
            output='screen',
        ),
        Node(
            package='ros2_message_flow_testcases',
            executable='sink',
            arguments=['bc'],
            output='screen',
        ),
    ])
