"""
@file bridge_nodes.launch.py
@author Naval Group
@brief Launch file for Gazebo-ROS 2 bridge nodes.

@details
This launch file starts the bridge node for:
- Simulation statistics (e.g., simulation time, RTF) (`stats_gz_to_ros_bridge`)

It ensures previous instances are killed before launch and introduces a small
startup delay to avoid race conditions.

@note
Designed to integrate Gazebo simulation data with ROS 2 topics for monitoring,
logging, or control purposes.

@version 0.2
@date 2026-07-28

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches the `stats_gz_to_ros_bridge` node, bridging simulation statistics
    (e.g., simulation time, RTF) from Gazebo to ROS 2, with proper cleanup.
    Delays the startup of the node slightly (1 second) to avoid race conditions
    with the pkill command below.
    """

    # ------------------------------------------------------------------
    # Kill any existing bridge node
    # ------------------------------------------------------------------
    # This ensures that if a previous instance of this node is running,
    # it is terminated before launching a new one.
    kill_stats_node = ExecuteProcess(cmd=["pkill", "-f", "stats_gz_to_ros_bridge"], shell=False, output="screen")

    # ------------------------------------------------------------------
    # Define bridge node
    # ------------------------------------------------------------------
    # Define the ROS 2 node that will bridge Gazebo stats to ROS 2
    stats_node = Node(
        package="gz_ros2_bridge", executable="stats_gz_to_ros_bridge", name="stats_gz_to_ros_bridge", output="screen"
    )

    # ------------------------------------------------------------------
    # Delay node startup
    # ------------------------------------------------------------------
    # TimerAction is used to delay the launch of the node slightly (1 second)
    # to ensure it is not killed by the pkill command above.
    delayed_stats_node = TimerAction(period=1.0, actions=[stats_node])

    return LaunchDescription([kill_stats_node, delayed_stats_node])
