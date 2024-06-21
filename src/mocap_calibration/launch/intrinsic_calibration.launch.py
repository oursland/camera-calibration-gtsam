# Copyright (c) 2020, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    hostname = platform.node().split('.')[0]
    namespace = '/feynman'

    composable_nodes = [
        ComposableNode(
            package='mocap_calibration',
            plugin='mocap::IntrinsicCalibration',
            name='mocap_intrinsic_calibration_node',
            namespace=namespace,
        ),
    ]

    arg_container = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace=namespace,
        # namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    # play calibration ros bag
    play_calibration_bag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "rosbags/rosbag2_2024_06_20-01_19_11",
        ],
        output="screen",
    )

    perform_calibration = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/feynman/calibrate",
            "mocap_msgs/srv/Calibrate",
        ],
        output="screen",
    )

    perform_calibration_event = RegisterEventHandler(
        OnProcessExit(
            target_action=play_calibration_bag,
            on_exit=[
                perform_calibration,
            ]
        )
    )

    shutdown_event = RegisterEventHandler(
        OnProcessExit(
            target_action=perform_calibration,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Completed'))
            ]
        )
    )

    return LaunchDescription([
        arg_container,
        image_processing_container,
        load_composable_nodes,
        play_calibration_bag,
        perform_calibration_event,
        shutdown_event,
    ])
