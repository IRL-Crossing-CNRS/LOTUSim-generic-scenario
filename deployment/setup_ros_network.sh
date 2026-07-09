#!/usr/bin/env bash
# setup_ros_network.sh
# Source this file on the remote machine to connect to the LOTUSim ROS 2 network.
#
# Usage:
#   source deployment/setup_ros_network.sh
#
# Then verify connectivity:
#   ros2 topic list   # should show /energy/mas_cmd etc.

# ---------------------------------------------------------------------------
# ROS 2 domain — must match the simulation machine (default: 0)
# ---------------------------------------------------------------------------
export ROS_DOMAIN_ID=67

# ---------------------------------------------------------------------------
# Optional: force a specific RMW implementation (uncomment if needed)
# ---------------------------------------------------------------------------
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "ROS network configured:"
echo "  ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo ""
echo "To verify connectivity with the simulation machine:"
echo "  ros2 topic list"
echo "  ros2 service list"
