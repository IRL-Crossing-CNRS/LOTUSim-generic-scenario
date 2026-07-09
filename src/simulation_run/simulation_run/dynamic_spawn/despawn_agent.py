"""
@file despawn_agent.py
@brief CLI script — despawn a LOTUSim agent from a running simulation.

Usage::

    ros2 run simulation_run despawn_agent --name bluerov2heavybase0

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0
"""

import argparse
import json
import sys
import time

import rclpy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description="Despawn a LOTUSim agent from a running simulation")
    parser.add_argument("--name", required=True, help="Agent name (e.g. bluerov2heavybase0)")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("despawn_agent_cli")

    result = []

    def on_status(msg):
        result.append(json.loads(msg.data))

    node.create_subscription(String, "/spawn_status", on_status, 10)
    pub = node.create_publisher(String, "/despawn_cmd", 10)

    # Wait until DynamicSpawnService is subscribed before publishing
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        print("Simulation not reachable on /despawn_cmd (is it running?)", file=sys.stderr)
        sys.exit(1)

    time.sleep(0.1)

    msg = String()
    msg.data = args.name
    pub.publish(msg)

    deadline = time.time() + 5.0
    while not result and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not result:
        print("No response from simulation (is it running?)", file=sys.stderr)
        sys.exit(1)

    r = result[0]
    if r.get("success"):
        print(f"Despawned: {r['agent_name']}")
    else:
        print(f"Failed: {r.get('error', 'unknown error')}", file=sys.stderr)
        sys.exit(1)
