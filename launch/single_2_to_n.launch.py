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

"""Launch file for the 2-to-N message link type."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from tracetools_launch.action import Trace


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        Trace(
            session_name='single_2-to-n',
            events_kernel=[],
            events_ust=[
                'dds:*',
                'ros2:*',
            ],
        ),
        Node(
            package='ros2_message_flow_testcases',
            executable='source',
            arguments=['a', '100'],
            output='screen',
        ),
        Node(
            package='ros2_message_flow_testcases',
            executable='source',
            arguments=['b', '40'],
            output='screen',
        ),
        Node(
            package='ros2_message_flow_testcases',
            executable='two_to_n',
            arguments=['ab', 'c'],
            output='screen',
        ),
        Node(
            package='ros2_message_flow_testcases',
            executable='sink',
            arguments=['c'],
            output='screen',
        ),
    ])
