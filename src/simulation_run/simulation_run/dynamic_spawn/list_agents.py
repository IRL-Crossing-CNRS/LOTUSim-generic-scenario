"""
@file list_agents.py
@brief CLI script — list all active agents in a running simulation.

Usage::

    ros2 run simulation_run list_agents

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0
"""

import sys

import rclpy
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.create_node("list_agents_cli")

    client = node.create_client(Trigger, "/list_agents")
    if not client.wait_for_service(timeout_sec=5.0):
        print("Service /list_agents not available (is the simulation running?)", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    node.destroy_node()
    rclpy.shutdown()

    if future.done():
        print(future.result().message)
    else:
        print("Service call timed out", file=sys.stderr)
        sys.exit(1)
