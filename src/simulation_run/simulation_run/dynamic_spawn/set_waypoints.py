"""
@file set_waypoints.py
@brief CLI script -- push a waypoint path to a running kinematic vehicle.

Calls the ``<model_name>/waypoints`` service (``lotusim_msgs/SetWaypoints``)
served by the C++ ``waypoint_follower`` plugin
(``systems/waypoint_follower/src/waypoint_follower.cpp``). That plugin is a
self-contained kinematic guidance+control system, entirely separate from the
Python bluerov_gnc-style Guidance tasks -- it only acts on an entity whose
spawned SDF carries a ``lotus_param/waypoint_follower`` element (see
``assets/worlds/circling_ship_example.world`` for the only shipped example).
This CLI exists so the service -- otherwise dead code, with no caller
anywhere in the codebase -- is demonstrably usable.

Usage::

    ros2 run simulation_run set_waypoints --world lotusim --model fremm \
        --lat 43.1 --lon 5.9 --lat 43.2 --lon 5.9 --loop

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0
"""

import argparse
import sys

import rclpy
from geographic_msgs.msg import GeoPoint
from lotusim_msgs.srv import SetWaypoints


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True,
                         help="World name (the plugin's ROS node namespace, e.g. 'lotusim')")
    parser.add_argument("--model", required=True,
                         help="Model name the waypoint_follower service is namespaced under")
    parser.add_argument("--lat", type=float, action="append", required=True,
                         help="Waypoint latitude (repeat --lat/--lon in pairs, in order)")
    parser.add_argument("--lon", type=float, action="append", required=True,
                         help="Waypoint longitude (repeat --lat/--lon in pairs, in order)")
    parser.add_argument("--alt", type=float, default=0.0,
                         help="Altitude for every waypoint (default 0)")
    parser.add_argument("--loop", action="store_true",
                         help="Loop back to the first waypoint after the last")
    args = parser.parse_args()

    if len(args.lat) != len(args.lon):
        print("--lat and --lon must be given the same number of times", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("set_waypoints_cli")

    service_name = f"/{args.world}/{args.model}/waypoints"
    client = node.create_client(SetWaypoints, service_name)
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"Service {service_name} not available (is the simulation "
              f"running, with a {args.model} carrying lotus_param/"
              f"waypoint_follower?)", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    request = SetWaypoints.Request()
    request.path = [
        GeoPoint(latitude=lat, longitude=lon, altitude=args.alt)
        for lat, lon in zip(args.lat, args.lon)
    ]
    request.loop = args.loop

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    node.destroy_node()
    rclpy.shutdown()

    if future.done() and future.result() is not None:
        result = future.result()
        print(f"success={result.success}")
        sys.exit(0 if result.success else 1)
    else:
        print("Service call timed out", file=sys.stderr)
        sys.exit(1)
