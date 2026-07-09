"""
@file dynamic_spawn_service.py
@brief ROS 2 node exposing runtime spawn/despawn/list services for LOTUSim agents.

Topics (std_msgs/String):
  /spawn_cmd   — JSON payload matching the scenario config format:
                 {"AgentType": {"pose": [x,y,z,r,p,y], ...}}
  /despawn_cmd — plain agent name string (e.g. "bluerov2heavybase0")
  /spawn_status — JSON feedback published after every operation

Services:
  /list_agents (std_srvs/Trigger) — returns comma-separated list of active agents

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0
"""

import json
import logging

from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

logger = logging.getLogger(__name__)


class DynamicSpawnService(Node):
    """
    Listens on ROS topics to spawn or despawn agents while the simulation runs.

    Spawn payload example::

        {"Bluerov2HeavyBase": {"pose": [0, 0, -30, 0, 0, 0]}}

    Despawn payload: just the agent name string, e.g. ``bluerov2heavybase0``.
    """

    def __init__(self, agents_manager, executor, world_name: str, world_file: str = "") -> None:
        super().__init__("dynamic_spawn_service")
        self._manager = agents_manager
        self._executor = executor
        self._world_name = world_name
        self._world_file = world_file

        self._status_pub = self.create_publisher(String, "/spawn_status", 10)
        self.create_subscription(String, "/spawn_cmd", self._on_spawn, 10)
        self.create_subscription(String, "/despawn_cmd", self._on_despawn, 10)
        self.create_service(Trigger, "/list_agents", self._on_list_agents)

        self.get_logger().info("DynamicSpawnService ready on /spawn_cmd and /despawn_cmd")

    # ------------------------------------------------------------------
    # Handlers
    # ------------------------------------------------------------------

    def _on_spawn(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish_status({"success": False, "error": f"JSON parse error: {e}"})
            return

        if not data or not isinstance(data, dict):
            self._publish_status({"success": False, "error": "Payload must be a non-empty JSON object"})
            return

        agent_type = next(iter(data))
        agent_info = data[agent_type] if isinstance(data[agent_type], dict) else {}

        agent_name = self._manager.spawn_one_agent(
            agent_type, agent_info, self._world_name, self._executor
        )

        if agent_name:
            self._publish_status({"success": True, "agent_name": agent_name})
        else:
            self._publish_status({"success": False, "error": f"Could not spawn agent of type '{agent_type}'"})

    def _on_despawn(self, msg: String) -> None:
        agent_name = msg.data.strip()
        if not agent_name:
            self._publish_status({"success": False, "error": "Empty agent name"})
            return

        success = self._manager.despawn_agent(agent_name, self._executor)
        if success:
            self._publish_status({"success": True, "agent_name": agent_name})
        else:
            self._publish_status({"success": False, "error": f"Agent '{agent_name}' not found"})

    def _on_list_agents(self, _request, response) -> Trigger.Response:
        names = [k for k in self._manager.agents.keys()]
        response.success = True
        response.message = ", ".join(names) if names else "(no agents)"
        return response

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _publish_status(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)
        logger.info("spawn_status: %s", msg.data)
