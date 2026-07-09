"""
@file spawn_agent.py
@brief CLI script — spawn a LOTUSim agent into a running simulation.

Usage::

    ros2 run simulation_run spawn_agent --config path/to/agent.json
    ros2 run simulation_run spawn_agent --json '{"Bluerov2HeavyBase": {"pose": [0,0,-30,0,0,0]}}'

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
    parser = argparse.ArgumentParser(description="Spawn a LOTUSim agent into a running simulation")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--config", help="Path to agent JSON config file")
    group.add_argument("--json", help="Inline JSON agent config")
    args = parser.parse_args()

    if args.config:
        try:
            with open(args.config) as f:
                json_content = f.read()
        except FileNotFoundError:
            print(f"Config file not found: {args.config}", file=sys.stderr)
            sys.exit(1)
    else:
        json_content = args.json

    try:
        json.loads(json_content)
    except json.JSONDecodeError as e:
        print(f"Invalid JSON: {e}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("spawn_agent_cli")

    result = []

    def on_status(msg):
        result.append(json.loads(msg.data))

    node.create_subscription(String, "/spawn_status", on_status, 10)
    pub = node.create_publisher(String, "/spawn_cmd", 10)

    # Wait until DynamicSpawnService is subscribed before publishing
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        print("Simulation not reachable on /spawn_cmd (is it running?)", file=sys.stderr)
        sys.exit(1)

    # Small buffer for our /spawn_status subscription to be discovered
    time.sleep(0.1)

    msg = String()
    msg.data = json_content
    pub.publish(msg)

    deadline = time.time() + 10.0
    while not result and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not result:
        print("No response from simulation (is it running?)", file=sys.stderr)
        sys.exit(1)

    r = result[0]
    if r.get("success"):
        print(f"Spawned: {r['agent_name']}")
    else:
        print(f"Failed: {r.get('error', 'unknown error')}", file=sys.stderr)
        sys.exit(1)
