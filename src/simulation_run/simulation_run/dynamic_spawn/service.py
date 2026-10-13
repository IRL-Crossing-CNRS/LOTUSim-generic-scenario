"""
@file dynamic_spawn_service.py
@brief ROS 2 node exposing runtime spawn/despawn/list services for LOTUSim agents.

Topics (std_msgs/String):
  /spawn_cmd   — JSON payload matching the scenario config format:
                 {"AgentType": {"pose": [x,y,z,r,p,y], ...}}
  /despawn_cmd — plain agent name string (e.g. "bluerov2heavybase0")
  /spawn_status — JSON feedback published after every operation
  /shard_status — each shard's agent list, so spawns go to the least loaded

Services:
  /list_agents (std_srvs/Trigger) — returns comma-separated list of active agents

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0
"""

import json
import logging
import time

from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

logger = logging.getLogger(__name__)

# How often a shard advertises the agents it hosts, and how long that
# advertisement is trusted. A shard that stops publishing drops out of the
# placement choice rather than silently receiving spawns it cannot serve.
SHARD_STATUS_PERIOD_S = 1.0
SHARD_STATUS_TTL_S = 5.0


class DynamicSpawnService(Node):
    """
    Listens on ROS topics to spawn or despawn agents while the simulation runs.

    Spawn payload example::

        {"Bluerov2HeavyBase": {"pose": [0, 0, -30, 0, 0, 0]}}

    Despawn payload: just the agent name string, e.g. ``bluerov2heavybase0``.

    Every shard runs one of these. A spawn arriving at runtime goes to the
    shard hosting the fewest agents, so a scene an operator keeps adding to
    stays spread across processes instead of piling onto the one that happens
    to own the public topic. Shard 0 alone reads ``/spawn_cmd`` and routes each
    request to a single shard's private topic: one decider means one spawn per
    request, whatever the shards' views of each other.
    """

    def __init__(
        self, agents_manager, executor, world_name: str, world_file: str = "", shard: int = 0, shards: int = 1
    ) -> None:
        super().__init__(f"dynamic_spawn_service_{shard}")
        self._manager = agents_manager
        self._executor = executor
        self._world_name = world_name
        self._world_file = world_file
        self._shard = shard
        self._shards = shards
        self._peers: dict[int, tuple[float, list]] = {}

        self._status_pub = self.create_publisher(String, "/spawn_status", 10)
        self._shard_pub = self.create_publisher(String, "/shard_status", 10)
        self.create_subscription(String, "/shard_status", self._on_shard_status, 10)
        self.create_timer(SHARD_STATUS_PERIOD_S, self._publish_shard_status)

        # Own inbox: only the shard the router picked spawns the agent.
        self.create_subscription(String, self._inbox(shard), self._on_spawn, 10)
        # A name lives in exactly one shard, so every shard may listen and the
        # one that owns it answers.
        self.create_subscription(String, "/despawn_cmd", self._on_despawn, 10)

        if shard == 0:
            self._routes = {i: self.create_publisher(String, self._inbox(i), 10) for i in range(shards)}
            self.create_subscription(String, "/spawn_cmd", self._on_spawn_request, 10)
            self.create_service(Trigger, "/list_agents", self._on_list_agents)
            self.get_logger().info("DynamicSpawnService ready on /spawn_cmd and /despawn_cmd")

    # ------------------------------------------------------------------
    # Shard placement
    # ------------------------------------------------------------------

    @staticmethod
    def _inbox(shard: int) -> str:
        return f"/spawn_cmd/shard{shard}"

    def _publish_shard_status(self) -> None:
        msg = String()
        msg.data = json.dumps({"shard": self._shard, "agents": list(self._manager.agents.keys())})
        self._shard_pub.publish(msg)

    def _on_shard_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            shard = int(data["shard"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        if shard != self._shard:
            self._peers[shard] = (time.monotonic(), data.get("agents", []))

    def _least_loaded_shard(self) -> int:
        """The shard hosting the fewest agents; ties go to the lowest index.

        Shard 0 also carries the CSV recorder, the mission watcher and this
        router, so it is only chosen once no other shard is known or every
        other shard is busier.
        """
        now = time.monotonic()
        counts = {self._shard: len(self._manager.agents)}
        for shard, (seen, agents) in self._peers.items():
            if now - seen <= SHARD_STATUS_TTL_S:
                counts[shard] = len(agents)
        others = {s: n for s, n in counts.items() if s != 0}
        pool = others or counts
        return min(pool, key=lambda s: (pool[s], s))

    def _on_spawn_request(self, msg: String) -> None:
        """Route a public spawn request to one shard. Shard 0 only."""
        try:
            json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish_status({"success": False, "error": f"JSON parse error: {e}"})
            return
        target = self._least_loaded_shard()
        self.get_logger().info(f"Spawn request routed to shard {target}.")
        self._routes[target].publish(msg)

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

        agent_name = self._manager.spawn_one_agent(agent_type, agent_info, self._world_name, self._executor)

        if agent_name:
            self._publish_shard_status()
            self._publish_status({"success": True, "agent_name": agent_name, "shard": self._shard})
        else:
            self._publish_status({"success": False, "error": f"Could not spawn agent of type '{agent_type}'"})

    def _on_despawn(self, msg: String) -> None:
        agent_name = msg.data.strip()
        if not agent_name:
            if self._shard == 0:
                self._publish_status({"success": False, "error": "Empty agent name"})
            return

        if agent_name not in self._manager.agents:
            # Another shard hosts it, or nobody does. Only shard 0 reports the
            # miss, so one request never yields N "not found" answers.
            if self._shard == 0 and not self._peer_hosts(agent_name):
                self._publish_status({"success": False, "error": f"Agent '{agent_name}' not found"})
            return

        success = self._manager.despawn_agent(agent_name, self._executor)
        if success:
            self._publish_shard_status()
            self._publish_status({"success": True, "agent_name": agent_name})
        else:
            self._publish_status({"success": False, "error": f"Agent '{agent_name}' not found"})

    def _peer_hosts(self, agent_name: str) -> bool:
        now = time.monotonic()
        return any(agent_name in agents for seen, agents in self._peers.values() if now - seen <= SHARD_STATUS_TTL_S)

    def _on_list_agents(self, _request, response) -> Trigger.Response:
        now = time.monotonic()
        names = list(self._manager.agents.keys())
        for _, (seen, agents) in sorted(self._peers.items()):
            if now - seen <= SHARD_STATUS_TTL_S:
                names.extend(agents)
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
