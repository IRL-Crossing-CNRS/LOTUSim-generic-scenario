import math
import threading

from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Point, Vector3
from lotusim_msgs.msg import LCOEState, WindTurbineArray, WindTurbineState
from lotusim_msgs.msg import Wind as WindMsg
from lotusim_msgs.msg import WindRegion as WindRegionMsg
from lotusim_msgs.msg import WindRegionArray as WindRegionArrayMsg
from sensor_msgs.msg import BatteryState

from lotusim_sdk.agents.environment import Environment
from lotusim_sdk.agents.environment.wind import WIND_REGIONS_TOPIC, WIND_TOPIC
from lotusim_sdk.agents.environment.wake.blended import BlendedWakeModel
from lotusim_sdk.agents.environment.wake.larsen import LarsenWakeModel
from lotusim_sdk.agents.environment.wake.wake_regions import (
    WakeRegionGenerator,
    wind_changed_enough,
)

_WAKE_MODELS = {
    "larsen": LarsenWakeModel,
}

_HOURS_PER_YEAR = 8760.0


class Wake(Environment):
    """Wind farm model: turns an ambient wind vector into per-turbine power, and
    optionally into the farm's LCOE.

    Subscribes to the ambient wind on
    :data:`~lotusim_sdk.agents.environment.wind.WIND_TOPIC` (written by the Unity
    sliders or by a ``Wind`` agent — this agent does not care which), applies the
    chosen wake model over the configured turbine layout, and publishes one
    ``lotusim_msgs/WindTurbineArray`` per wind update on
    ``/<world>/wind/turbines``.

    With an ``lcoe`` config block it also integrates that power into produced
    energy, accounts the cost of the robots operating the farm (time on station +
    battery energy drawn, discovered from their ``*/battery/state`` topics) plus
    fixed turbine maintenance, and publishes the ratio on ``/<world>/lcoe``. The
    energy accounting lives here rather than in an agent of its own because it is
    driven by the very turbine power this agent computes — separating them would
    only put the farm's power output on the wire and read it straight back.

    With a ``wind_regions`` config block it also turns the wake footprint itself
    into ``lotusim_msgs/WindRegionArray`` cone segments on
    :data:`~lotusim_sdk.agents.environment.wind.WIND_REGIONS_TOPIC` — the same
    topic the ``Wind`` agent's static ``regions`` list publishes to, and the one
    the ``wind_regions`` Gazebo plugin already reads. Each turbine's wake is
    represented as a handful of tapered cone segments chained end to end (see
    :mod:`~.wake_regions`), so a wind-enabled vehicle flying through it feels a
    graduated deficit instead of a single blanket rectangle, using a
    closed-form point-in-cone test on the Gazebo side rather than a box
    approximation. Regenerated only when the wind has moved enough to matter
    (``direction_hysteresis_deg`` / ``speed_hysteresis_mps``) — not on every
    wind message — since the Gazebo plugin re-scans the whole region list for
    every wind-enabled link on every physics tick.

    **This makes ``Wake`` a second writer on ``WIND_REGIONS_TOPIC``**, alongside
    ``Wind``. Both publish independently on a latched topic with no merging
    between them, so whichever last published wins — do not declare both
    ``Wind.regions`` and ``Wake.wind_regions`` in the same scenario, or one will
    silently blank out the other.
    """

    def __init__(
        self,
        world_name: str,
        wake_model: str = "larsen",
        model_params: dict = None,
        diameter: float = 61.0,
        ct: float = 0.8,
        cp: float = 0.35,
        air_density: float = 1.225,
        cut_in: float = 5.0,
        cut_out: float = 25.0,
        ambient_ti: float = 0.08,
        shear_exponent: float = 0.12,
        maintenance_cost: float = 100000.0,
        lcoe: dict = None,
        wind_regions: dict = None,
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
            diameter=diameter,
            ct=ct,
            air_density=air_density,
            cp=cp,
            cut_in=cut_in,
            cut_out=cut_out,
            ambient_ti=ambient_ti,
            shear_exponent=shear_exponent,
        )
        self._model = model_cls(**base_params, **(model_params or {}))

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(WindTurbineArray, f"/{world_name}/wind/turbines", latched_qos)
        self._sub = self.create_subscription(WindMsg, WIND_TOPIC, self._on_wind, 10)

        self._lcoe_enabled = bool(lcoe)
        if self._lcoe_enabled:
            self._setup_lcoe(world_name, lcoe, maintenance_cost, latched_qos)

        self._wind_regions_enabled = bool(wind_regions)
        if self._wind_regions_enabled:
            self._setup_wind_regions(wind_regions, latched_qos)

        self.get_logger().info(
            f"Wake ready: model={wake_model}, {len(self._turbines_cfg)} turbines, "
            f"lcoe={'on' if self._lcoe_enabled else 'off'}, "
            f"wind_regions={'on' if self._wind_regions_enabled else 'off'}, "
            f"waiting for wind on {WIND_TOPIC}"
        )

    # ------------------------------------------------------------------
    # Wake model
    # ------------------------------------------------------------------
    def _on_wind(self, msg: WindMsg):
        if not msg.enable_wind:
            return

        vx = msg.linear_velocity.x
        vy = msg.linear_velocity.y
        # ENU: x=East, y=North horizontal plane, z=Up. Matches the world frame
        # the wind_regions Gazebo plugin applies this same ambient vector in
        # (it subscribes to this topic directly) — the wake model must agree
        # on which axis is "up" or the two consumers of this one topic
        # silently disagree on what the wind vector means.
        og_speed = math.sqrt(vx**2 + vy**2)
        if og_speed < 1e-6:
            return

        # RPMs come back from the model but go nowhere: WindTurbineState has no
        # field for them, and that message lives in the core repo.
        turbines_sorted, velocities, _rpms = self._model.wind_speeds_full(self._turbine_positions, [vx, vy])
        # Power is averaged over the rotor disk through the shear profile, so
        # each turbine is evaluated at its own hub height rather than at one
        # farm-wide value.
        powers = [
            self._model.power(velocity, hub_height=z) for (_x, _y, z), velocity in zip(turbines_sorted, velocities)
        ]

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
            # Once per wind update, on the finished array. Integrating inside the
            # loop above would accumulate one dt per turbine over a
            # partially-filled array — energy produced would then scale with the
            # size of the farm rather than with its power.
            self._accumulate_energy(array_msg)

        self._pub.publish(array_msg)

        if self._wind_regions_enabled:
            self._maybe_publish_wind_regions([vx, vy])

    # ------------------------------------------------------------------
    # Wind regions (dynamic wake boxes)
    # ------------------------------------------------------------------
    def _setup_wind_regions(self, cfg: dict, latched_qos: QoSProfile):
        if not isinstance(self._model, LarsenWakeModel):
            raise ValueError(
                "wind_regions requires wake_model='larsen' — BlendedWakeModel "
                "wraps a LarsenWakeModel instance for its rotor geometry."
            )
        blended = BlendedWakeModel(self._model)
        self._region_generator = WakeRegionGenerator(
            blended,
            cell_diameters=cfg.get("cell_diameters", 0.5),
            deficit_threshold=cfg.get("deficit_threshold", 0.05),
            # NOT a self-terminating condition in practice: the calibrated
            # centreline deficit 0.58*(x/D)^-0.35 only falls below a 5%
            # relative deficit around x/D=1100 (see wake_regions.py), so this
            # cap — not deficit_threshold — is what actually determines wake
            # length for any realistic value. Scale it to your farm's own
            # turbine spacing; 8D matches the more loosely-spaced of the two
            # farms this model was validated on (Layout B, 9D).
            max_downstream_diameters=cfg.get("max_downstream_diameters", 8.0),
        )
        self._region_direction_hysteresis_deg = cfg.get("direction_hysteresis_deg", 5.0)
        self._region_speed_hysteresis_mps = cfg.get("speed_hysteresis_mps", 0.5)
        self._last_region_wind: tuple | None = None

        # Same topic and QoS the `Wind` agent uses for its own (static)
        # regions — see the class docstring for why this makes the two
        # mutually exclusive within one scenario.
        self._regions_pub = self.create_publisher(WindRegionArrayMsg, WIND_REGIONS_TOPIC, latched_qos)

    def _maybe_publish_wind_regions(self, wind_vector):
        if not wind_changed_enough(
            self._last_region_wind,
            wind_vector,
            self._region_direction_hysteresis_deg,
            self._region_speed_hysteresis_mps,
        ):
            return
        self._last_region_wind = tuple(wind_vector)

        turbines_named = [(t["name"], t["x"], t["y"], t["z"]) for t in self._turbines_cfg]
        regions = self._region_generator.regions_for_wind(turbines_named, wind_vector)

        msg = WindRegionArrayMsg()
        for r in regions:
            region_msg = WindRegionMsg()
            region_msg.id = r["id"]
            region_msg.shape_type = WindRegionMsg.CONE_SEGMENT
            region_msg.cone.origin = Point(x=r["origin_x"], y=r["origin_y"], z=r["origin_z"])
            region_msg.cone.axis = Vector3(x=r["axis_x"], y=r["axis_y"], z=0.0)
            region_msg.cone.length = r["length"]
            region_msg.cone.r_start = r["r_start"]
            region_msg.cone.r_end = r["r_end"]
            region_msg.linear_velocity = Vector3(x=r["vx"], y=r["vy"], z=r["vz"])
            region_msg.enable_wind = True
            msg.regions.append(region_msg)
        self._regions_pub.publish(msg)

    # ------------------------------------------------------------------
    # LCOE
    # ------------------------------------------------------------------
    def _setup_lcoe(self, world_name, lcoe, maintenance_cost, latched_qos):
        self._alpha_r = lcoe.get("alpha_r_aud_per_hour", 50.0)
        self._alpha_e = lcoe.get("alpha_e_aud_per_kwh", 0.5)
        publish_rate = lcoe.get("publish_rate_hz", 1.0)

        # maintenance_cost is per turbine per year; spread over the run.
        num_turbines = len(self._turbines_cfg)
        self._maintenance_aud_per_h = maintenance_cost * num_turbines / _HOURS_PER_YEAR

        self._sim_start_time: float | None = None
        self._last_turbine_time: float | None = None
        self._last_farm_power_w: float = 0.0
        self._energy_produced_wh: float = 0.0

        self._robot: dict = {}
        self._robot_lock = threading.Lock()
        self._battery_topics: set = set()

        self._lcoe_pub = self.create_publisher(LCOEState, f"/{world_name}/lcoe", latched_qos)
        self.create_timer(2.0, self._discover_battery_topics)
        self.create_timer(1.0 / publish_rate, self._publish_lcoe)

    def _now_s(self) -> float:
        """Seconds from the node clock.

        One clock for both sides of the ratio (energy produced, and robot
        operating cost), so the two stay consistent whatever real-time factor the
        scenario runs at.
        """
        return self.get_clock().now().nanoseconds / 1e9

    def _accumulate_energy(self, msg: WindTurbineArray):
        now = self._now_s()
        farm_power_w = sum(t.power_w for t in msg.turbines)
        if self._sim_start_time is None:
            self._sim_start_time = now
        if self._last_turbine_time is not None:
            # Trapezoidal: power is assumed to vary linearly between two samples.
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
                    "start": self._now_s(),
                    "initial_charge_ah": msg.charge,
                    "charge_ah": msg.charge,
                    "voltage_v": msg.voltage,
                }
            else:
                self._robot[agent_name]["charge_ah"] = msg.charge
                self._robot[agent_name]["voltage_v"] = msg.voltage

    def _publish_lcoe(self):
        now = self._now_s()
        sim_time_h = (now - self._sim_start_time) / 3600.0 if self._sim_start_time else 0.0

        with self._robot_lock:
            total_robot_time_h = sum((now - s["start"]) / 3600.0 for s in self._robot.values())
            total_robot_energy_wh = sum(
                max(0.0, s["initial_charge_ah"] - s["charge_ah"]) * s["voltage_v"] for s in self._robot.values()
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
