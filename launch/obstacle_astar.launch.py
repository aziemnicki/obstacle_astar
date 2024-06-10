# Copyright 2024 Andrzej_Norbert_Jeremiasz
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('obstacle_astar')
    param_path = PathJoinSubstitution([pkg_prefix, 'config/obstacle_astar.param.yaml'])

    obstacle_astar_node = Node(
        package='obstacle_astar',
        executable='obstacle_astar_node_exe',
        name='obstacle_astar_node',
        parameters=[
            param_path
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
        remappings=[
            ("~/input/trajectory", LaunchConfiguration('input_trajectory')),
            ("~/input/occupancy_grid", LaunchConfiguration('input_occupancy_grid')),
            ("~/input/odometry", LaunchConfiguration('input_odometry')),
            ("~/output/trajectory", LaunchConfiguration('output_trajectory')),
            ("~/output/markers", LaunchConfiguration('output_markers'))
        ],
    )

    return [
        obstacle_astar_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('input_trajectory', '/planning/racing_planner/trajectory')
    add_launch_arg('input_occupancy_grid', '/map_modified')
    add_launch_arg('input_odometry', '/localization/kinematic_state')
    add_launch_arg('output_trajectory', '/planning/racing_planner/avoidance/trajectory')
    add_launch_arg('output_markers', '/markers')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])