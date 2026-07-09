import math
import threading
import time

from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Point
from lotusim_msgs.msg import LCOEState, WindTurbineArray, WindTurbineState
from lotusim_msgs.msg import Wind as WindMsg
from sensor_msgs.msg import BatteryState

from lotusim_sdk.agents.environment import Environment
from lotusim_sdk.agents.environment.wind.jensen import JensenWakeModel
from lotusim_sdk.agents.environment.wind.gaussian import GaussianWakeModel

_WAKE_MODELS = {
    "jensen": JensenWakeModel,
    "gaussian": GaussianWakeModel,
}

_HOURS_PER_YEAR = 8760.0


class Wind(Environment):
    """
    Environment agent that simulates wind physics over a turbine farm.

    Subscribes to ambient wind, applies the chosen wake model, and publishes
    per-turbine effective wind speed and power. When an 'lcoe' config block is
    present, also computes and publishes farm-level LCOE by discovering robot
    battery topics dynamically — no explicit agent list needed.

    All configuration comes directly from the scenario JSON — no external config file.
    """

    def __init__(
        self,
        world_name: str,
        wake_model: str = "jensen",
        model_params: dict = None,
        diameter: float = 61.0,
        ct: float = 0.8,
        cp: float = 0.35,
        air_density: float = 1.225,
        cut_in: float = 5.0,
        cut_out: float = 25.0,
        maintenance_cost: float = 100000.0,
        lcoe: dict = None,
        turbines: list = None,
        **kwargs,
    ):
        super().__init__(world_name)

        self._turbines_cfg = turbines or []
        self._turbine_positions = [(t["x"], t["y"], t["z"]) for t in self._turbines_cfg]
        self._pos_to_name = {(t["x"], t["y"], t["z"]): t["name"] for t in self._turbines_cfg}

        model_cls = _WAKE_MODELS.get(wake_model)
        if model_cls is None:
            raise ValueError(f"Unknown wake model '{wake_model}'. Choose from: {list(_WAKE_MODELS)}")

        base_params = dict(
            diameter=diameter, ct=ct, air_density=air_density,
            cp=cp, cut_in=cut_in, cut_out=cut_out,
        )
        self._model = model_cls(**base_params, **(model_params or {}))
        self.get_logger().info(f"Using wake model: {wake_model}")

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(
            WindTurbineArray, f"/{world_name}/wind/turbines", latched_qos
        )
        self._sub = self.create_subscription(
            WindMsg, "/aerialWorld/wind", self._on_wind, 10
        )

        if lcoe:
            self._lcoe_enabled = True
            self._alpha_r = lcoe.get("alpha_r_aud_per_hour", 50.0)
            self._alpha_e = lcoe.get("alpha_e_aud_per_kwh", 0.5)
            publish_rate = lcoe.get("publish_rate_hz", 1.0)

            num_turbines = len(self._turbines_cfg)
            self._maintenance_aud_per_h = maintenance_cost * num_turbines / _HOURS_PER_YEAR

            self._sim_start_time: float | None = None
            self._last_turbine_time: float | None = None
            self._last_farm_power_w: float = 0.0
            self._energy_produced_wh: float = 0.0

            self._robot: dict = {}
            self._robot_lock = threading.Lock()
            self._battery_topics: set = set()

            self._lcoe_pub = self.create_publisher(
                LCOEState, f"/{world_name}/lcoe", latched_qos
            )
            self.create_timer(2.0, self._discover_battery_topics)
            self.create_timer(1.0 / publish_rate, self._publish_lcoe)
            self.get_logger().info("LCOE monitoring enabled.")
        else:
            self._lcoe_enabled = False

        self.get_logger().info("Wind ready, waiting for wind vector on /aerialWorld/wind...")

    def _on_wind(self, msg: WindMsg):
        if not msg.enable_wind:
            return

        vx = msg.linear_velocity.x
        vy = msg.linear_velocity.y
        vz = msg.linear_velocity.z
        og_speed = math.sqrt(vx**2 + vz**2)
        if og_speed < 1e-6:
            return

        turbines_sorted, velocities = self._model.wind_speeds_full(
            self._turbine_positions, og_speed, [vx, vz]
        )
        powers = [self._model.power(v) for v in velocities]

        array_msg = WindTurbineArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "world"

        for (x, y, z), velocity, power in zip(turbines_sorted, velocities, powers):
            state = WindTurbineState()
            state.name = self._pos_to_name.get((x, y, z), f"unknown_{x}_{y}")
            state.position = Point(x=float(x), y=float(y), z=float(z))
            state.effective_wind_speed = float(velocity)
            state.power_w = float(power)
            array_msg.turbines.append(state)

            if self._lcoe_enabled:
                self._on_turbines_power(array_msg)

        self.get_logger().info(f"Wind Vector: [X:{vx:.2f}, Y:{vy:.2f}, Z:{vz:.2f}]")
        self._pub.publish(array_msg)

    def _on_turbines_power(self, msg: WindTurbineArray):
        now = time.monotonic()
        farm_power_w = sum(t.power_w for t in msg.turbines)
        if self._sim_start_time is None:
            self._sim_start_time = now
        if self._last_turbine_time is not None:
            dt_h = (now - self._last_turbine_time) / 3600.0
            self._energy_produced_wh += 0.5 * (self._last_farm_power_w + farm_power_w) * dt_h
        self._last_turbine_time = now
        self._last_farm_power_w = farm_power_w

    def _discover_battery_topics(self):
        for topic_name, types in self.get_topic_names_and_types():
            if not topic_name.endswith("/battery/state"):
                continue
            if topic_name in self._battery_topics:
                continue
            agent_name = topic_name.split("/")[-3]
            self.create_subscription(
                BatteryState,
                topic_name,
                lambda msg, n=agent_name: self._on_battery(n, msg),
                10,
            )
            self._battery_topics.add(topic_name)
            self.get_logger().info(f"LCOE: subscribed to battery topic {topic_name}")

    def _on_battery(self, agent_name: str, msg: BatteryState):
        with self._robot_lock:
            if agent_name not in self._robot:
                self._robot[agent_name] = {
                    "start": time.monotonic(),
                    "initial_charge_ah": msg.charge,
                    "charge_ah": msg.charge,
                    "voltage_v": msg.voltage,
                }
            else:
                self._robot[agent_name]["charge_ah"] = msg.charge
                self._robot[agent_name]["voltage_v"] = msg.voltage

    def _publish_lcoe(self):
        now = time.monotonic()
        sim_time_h = (now - self._sim_start_time) / 3600.0 if self._sim_start_time else 0.0

        with self._robot_lock:
            total_robot_time_h = sum((now - s["start"]) / 3600.0 for s in self._robot.values())
            total_robot_energy_wh = sum(
                max(0.0, s["initial_charge_ah"] - s["charge_ah"]) * s["voltage_v"]
                for s in self._robot.values()
            )

        cost_maintenance = self._maintenance_aud_per_h * sim_time_h
        cost_robot_time = self._alpha_r * total_robot_time_h
        cost_robot_energy = self._alpha_e * (total_robot_energy_wh / 1000.0)
        cost_total = cost_maintenance + cost_robot_time + cost_robot_energy

        energy_produced_kwh = self._energy_produced_wh / 1000.0
        energy_produced_mwh = energy_produced_kwh / 1000.0
        lcoe_val = cost_total / energy_produced_mwh if energy_produced_mwh > 0.0 else 0.0

        msg = LCOEState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.lcoe_aud_mwh = float(lcoe_val)
        msg.energy_produced_kwh = float(energy_produced_kwh)
        msg.cost_total_aud = float(cost_total)
        msg.cost_maintenance_aud = float(cost_maintenance)
        msg.cost_robot_time_aud = float(cost_robot_time)
        msg.cost_robot_energy_aud = float(cost_robot_energy)
        msg.robot_operation_time_h = float(total_robot_time_h)
        msg.robot_energy_wh = float(total_robot_energy_wh)
        self._lcoe_pub.publish(msg)
