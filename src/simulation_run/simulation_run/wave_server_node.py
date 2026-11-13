"""The scenario's sea: the same wave components for every xdyn model and,
on ROS, for the renderer.

xdyn evaluates the components itself, from the rays file simulation_run.prepare_weather
writes into each vehicle model, when the sea state is fixed for the run. When
it can change (``"live": true``, or ``"weather": {"follow_wind": true}``),
this node rewrites that rays file at each change of the sea state, with the
fades of the change (``--rays-file``), and xdyn re-reads it: the physics keeps
the speed of a fixed sea. This node computes no physics: xdyn evaluates the
waves and the current, Gazebo's wind plugin the wind on the drones. With
``"shared": false`` the renderer gets its own draw of the same sea state (same
spectrum, other phases), as each vehicle model does: the baseline where physics
and rendering each sample the sea.

With ``"weather": {"date", "place"}`` (and unless ``"wind": false``) this
node also publishes the 10 m wind of that place and date, once a second, on
``/aerialWorld/wind``, where the drones' wind plugin and the renderer read it.
With ``"weather": {"beaufort": B}`` it publishes U10 of that force, blowing
towards the waves' direction. Not with ``"follow_wind"``, where the wind is the
input and the sea follows it.

A change of weather during the run takes the sea, the wind and the current from
one equilibrium to the other together, over the same ramp: the waves fade with
gains sqrt(u) and sqrt(1 - u), since the energies of two independent seas add;
the wind and the current, which add as velocities, move linearly,
(1 - u) old + u new. The current is xdyn's: with a Beaufort weather the node
writes the change of wind into ``ekman_weather.csv`` next to the rays file
(lotusim_sdk.ekman), every xdyn re-reads it, and its Ekman model moves the
current. The current of a date, replayed by the physics plugin, does not
follow a change.

Started by scenario_launch.sh before the xdyn processes when the scenario has
a ``"waves"`` or ``"weather"`` block (see ``lotusim_sdk.waves.sea_state``):

    python3 -m simulation_run.wave_server_node --config <scenario.json> --world <world> --rays-file <run>/waves/wave_rays.csv

A live sea state can change during the run, on either input:

- ``/<world>/sea_state`` (std_msgs/String): a JSON object with the keys of a
  scenario ``"weather"`` or ``"waves"`` block, e.g. ``{"beaufort": 5}``,
  ``{"hs_m": 1.2, "tp_s": 5}`` or ``{"date": "2024-08-10T12:00", "place": {"lat": 50.25,
  "lon": -4.25}}``, plus an optional ``"ramp_s"``;
- the scenario's own schedule, ``"weather": {"schedule": [{"t_s": 80, "beaufort": 5},
  ...]}``: each entry is applied once the simulation time reaches ``t_s``;
- the wind, when ``"weather": {"follow_wind": true}``: each change of the
  Beaufort force of ``/aerialWorld/wind`` (lotusim_msgs/Wind, the Unity
  sliders) raises or calms the sea to that force, waves travelling with the
  wind.

The old sea fades out while the new one fades in over ``ramp_s`` seconds of
simulation time (default 60), in xdyn and in the renderer alike.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from typing import Any, Dict

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from lotusim_msgs.msg import VesselPositionArray, WaveSpectrum, Wind
from rclpy.qos import qos_profile_sensor_data
from lotusim_sdk.wave_layers import WaveField
from geometry_msgs.msg import Vector3
from lotusim_sdk import ekman
from lotusim_sdk.waves import (
    BEAUFORT, DEFAULTS, build_rays, sea_is_live, sea_is_shared, sea_state, wind_of_weather, write_components_csv,
)

#: Upper wind speed of each Beaufort force 0..11 (m/s), WMO; 12 above.
BEAUFORT_LIMITS = [0.5, 1.5, 3.3, 5.5, 7.9, 10.7, 13.8, 17.1, 20.7, 24.4, 28.4, 32.6]


def beaufort_of(speed: float) -> int:
    for force, limit in enumerate(BEAUFORT_LIMITS):
        if speed < limit:
            return force
    return 12


class WaveServerNode(Node):
    def __init__(self, world: str, scenario: Dict[str, Any], rays_file: str | None = None) -> None:
        super().__init__("wave_server")
        weather = scenario.get("weather") or {}
        self._params = dict(DEFAULTS)
        self._params.update(sea_state(scenario))
        self._ramp_s = float(weather.get("ramp_s", 60.0))
        self._live, self._shared = sea_is_live(scenario), sea_is_shared(scenario)
        self.field = WaveField(build_rays(self._params))
        # shortcut: a separate sea is fixed for the run; a live change only reaches the shared one.
        self._render_field = self.field if self._shared else WaveField(
            build_rays({**self._params, "seed": int(self._params["seed"]) + 1000}))
        self._sent_version = -1
        self._rays_file = rays_file
        # Simulation time, for the fades: from the poses the renderer gets, since
        # xdyn reads the rays file and no longer asks this node.
        self.create_subscription(VesselPositionArray, f"/{world}/renderer_poses", self._on_poses, qos_profile_sensor_data)
        if rays_file:
            # Once a change has faded the old sea out, drop it from the file: xdyn's
            # cost grows with the number of components it sums.
            self.create_timer(1.0, self._drop_faded_layers)
        self._force = self._params.get("beaufort")
        self._weather = weather

        # The Ekman current of a Beaufort weather, computed by xdyn from the wind prepare_weather set:
        # a change of force rewrites the wind of the change in the file xdyn re-reads.
        waves_dir = os.path.dirname(os.path.abspath(rays_file)) if rays_file else None
        self._ekman_file = None
        if waves_dir and os.path.isfile(os.path.join(waves_dir, "ekman_weather.csv")):
            self._ekman_file = os.path.join(waves_dir, "ekman_weather.csv")
            self._ekman_wind = (BEAUFORT[int(self._params["beaufort"])][0], float(self._params["direction_deg"]))

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(WaveSpectrum, f"/{world}/wave_spectrum", qos)
        self.create_subscription(String, f"/{world}/sea_state", self._on_sea_state, 10)
        if weather.get("follow_wind"):
            self.create_subscription(Wind, str(weather.get("wind_topic", "/aerialWorld/wind")), self._on_wind, 10)
            # A force is applied once the wind has held it for wind_settle_s: a slider
            # dragged across several forces, or a Wind agent's zero before its first
            # set_wind, would otherwise start a sea-state change at every step.
            self._wind_settle_s = float(weather.get("wind_settle_s", 2.0))
            self._pending_wind = None
            self.create_timer(0.5, self._apply_settled_wind)
        self.create_timer(1.0, self._publish)
        # Scripted sea-state changes, applied in order of time.
        self._schedule = sorted((dict(e) for e in weather.get("schedule") or []), key=lambda e: float(e["t_s"]))
        if self._schedule:
            self.create_timer(0.2, self._run_schedule)
        self.get_logger().info(
            f"Sea Hs {self._params['hs_m']} m, Tp {self._params['tp_s']:.2f} s, "
            + ("live, re-read by xdyn at each change" if self._live else "fixed, evaluated by xdyn")
            + (", following the wind" if weather.get("follow_wind") else "")
            + ("" if self._shared else ", NOT shared with the renderer")
        )

        # The wind of the mission's place and date, or of its Beaufort force, for the drones
        # and the renderer.
        self._wind = None  # (east, north) published, m/s
        self._wind_ramp = None  # (old, new, t start, duration)
        self._wind_driven = (weather.get("wind", True) and not weather.get("follow_wind")
                             and ("date" in weather or "beaufort" in self._params))
        if self._wind_driven:
            self._wind = self._wind_target(self._params, weather)
            self._wind_msg = Wind()
            self._wind_msg.enable_wind = True
            self._wind_pub = self.create_publisher(Wind, str(weather.get("wind_topic", "/aerialWorld/wind")), qos)
            self.create_timer(1.0, self._publish_wind)
            self.get_logger().info(f"Wind {self._wind[0]:.2f} m/s east, {self._wind[1]:.2f} m/s north at 10 m")

    def _wind_target(self, params: Dict[str, Any], update: Dict[str, Any]):
        """(east, north) 10 m wind of a weather: the measured wind of a date, else U10 of the force."""
        if "date" in update:
            from lotusim_sdk.weather_of_date import wind_of_date

            place = update.get("place") or self._weather.get("place") or {}
            wind = wind_of_date(float(place["lat"]), float(place["lon"]), str(update["date"]))
            return float(wind["east_ms"]), float(wind["north_ms"])
        return wind_of_weather(params)

    @staticmethod
    def _progress(ramp, t: float) -> float:
        _, _, t0, duration = ramp
        return 1.0 if duration <= 0 else min(1.0, max(0.0, (t - t0) / duration))

    def _publish_wind(self) -> None:
        if self._wind_ramp is not None:
            old, new, _, _ = self._wind_ramp
            u = self._progress(self._wind_ramp, self.field.last_t)
            self._wind = ((1 - u) * old[0] + u * new[0], (1 - u) * old[1] + u * new[1])
            if u >= 1.0:
                self._wind_ramp = None
        self._wind_msg.linear_velocity = Vector3(x=float(self._wind[0]), y=float(self._wind[1]), z=0.0)
        self._wind_pub.publish(self._wind_msg)

    def _change(self, update: Dict[str, Any], ramp_s: float) -> None:
        if not self._live:
            self.get_logger().warn(
                "Sea state change ignored: the sea of this mission is fixed and evaluated by xdyn. "
                'Set "live": true in the "waves" or "weather" block to change it during the run.'
            )
            return
        upd = {k: v for k, v in update.items() if k != "ramp_s"}
        if "beaufort" in upd or "date" in upd:
            # A new force, or the sea of a place and date, sets Hs and Tp again (and, for a date,
            # the direction); the rest of the sea is kept.
            kept = ("seed", "spreading_s", "nfreq", "ndir", "energy_fraction", "depth_m")
            if "date" not in upd:
                kept += ("direction_deg",)
            keep = {k: self._params[k] for k in kept}
            params = dict(DEFAULTS)
            params.update(sea_state({"weather": {**keep, **upd}}))
        else:
            params = dict(self._params)
            params.update({k: v for k, v in upd.items() if k in DEFAULTS})
        if all(params.get(k) == self._params.get(k) for k in DEFAULTS):
            self.get_logger().info("Sea state unchanged, request ignored")
            return
        self._params = params
        t = self.field.last_t
        # The wind and the current move to the new weather over the same ramp as the waves.
        if self._wind_driven and ("beaufort" in params or "date" in upd):
            self._publish_wind()  # settles any ramp still running at its value now
            self._wind_ramp = (self._wind, self._wind_target(params, upd), t, ramp_s)
        if self._ekman_file and "beaufort" in params:
            new = (BEAUFORT[int(params["beaufort"])][0], float(params["direction_deg"]))
            ekman.write_weather(self._ekman_file, t, ramp_s, self._ekman_wind, new)
            self.get_logger().info(f"xdyn's Ekman current from U10 {self._ekman_wind[0]} to {new[0]} m/s over {ramp_s:.0f} s")
            self._ekman_wind = new
        self.field.change(build_rays(params), t, ramp_s)
        if self._rays_file:
            self.field.prune(t)
            write_components_csv(self.field.components(), self._rays_file)
        self.get_logger().info(
            f"Sea state at t = {t:.1f} s: Hs {params['hs_m']} m, Tp {params['tp_s']:.2f} s, "
            f"towards {params['direction_deg']:.0f} deg, over {ramp_s:.0f} s"
        )
        self._publish()

    def _drop_faded_layers(self) -> None:
        before = len(self.field.components()["a"])
        self.field.prune(self.field.last_t)
        if len(self.field.components()["a"]) != before:
            write_components_csv(self.field.components(), self._rays_file)
            self.get_logger().info(f"Faded sea dropped at t = {self.field.last_t:.1f} s: "
                                   f"{len(self.field.components()['a'])} components left")

    def _on_poses(self, msg: VesselPositionArray) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t > self.field.last_t:
            self.field.last_t = t

    def _run_schedule(self) -> None:
        while self._schedule and self.field.last_t >= float(self._schedule[0]["t_s"]):
            entry = self._schedule.pop(0)
            update = {k: v for k, v in entry.items() if k != "t_s"}
            self._force = update.get("beaufort", self._force)
            self.get_logger().info(f"Scheduled sea state at t = {float(entry['t_s']):.1f} s: {update}")
            self._change(update, float(update.get("ramp_s", self._ramp_s)))

    def _on_sea_state(self, msg: String) -> None:
        try:
            update = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"sea_state is not JSON: {e}")
            return
        if self.field.last_t <= 0.0:
            # Before xdyn's first step, a request can only be left over from an
            # earlier run: `ros2 topic pub --once` waits for a subscriber and
            # delivers to the next server that starts.
            self.get_logger().warn(f"sea_state received before the simulation started, ignored: {msg.data}")
            return
        self._force = update.get("beaufort", self._force)
        self._change(update, float(update.get("ramp_s", self._ramp_s)))

    def _on_wind(self, msg: Wind) -> None:
        if not msg.enable_wind or self.field.last_t <= 0.0:
            return  # the sea of the mission file holds until the simulation runs
        east, north = msg.linear_velocity.x, msg.linear_velocity.y
        force = beaufort_of(math.hypot(east, north))
        if self._pending_wind is None or self._pending_wind[0] != force:
            self._pending_wind = (force, east, north, self.get_clock().now())
        else:
            self._pending_wind = (force, east, north, self._pending_wind[3])

    def _apply_settled_wind(self) -> None:
        if self._pending_wind is None:
            return
        force, east, north, since = self._pending_wind
        if (self.get_clock().now() - since).nanoseconds * 1e-9 < self._wind_settle_s:
            return
        self._pending_wind = None
        # Compared with the force of the sea now, not with a first wind: Unity publishes nothing
        # until a key or a slider moves, so a first wind taken as reference swallowed F1's storm.
        if force == self._force:
            return
        self._force = force
        # Waves travel with the wind: psi from north towards east (ENU wind vector).
        direction = math.degrees(math.atan2(east, north)) % 360.0
        self._change({"beaufort": force, "direction_deg": direction}, self._ramp_s)

    def _publish(self) -> None:
        c = self._render_field.components()
        msg = WaveSpectrum()
        msg.significant_height_m = float(self._params["hs_m"])
        msg.peak_period_s = float(self._params["tp_s"])
        msg.direction_deg = float(self._params["direction_deg"])
        msg.a, msg.psi, msg.omega, msg.k, msg.phase = c["a"], c["psi"], c["omega"], c["k"], c["phase"]
        msg.fade_in_start, msg.fade_in_duration = c["fade_in_start"], c["fade_in_duration"]
        msg.fade_out_start, msg.fade_out_duration = c["fade_out_start"], c["fade_out_duration"]
        self._pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--rays-file", default=None, help="rays file xdyn re-reads, rewritten at each sea-state change")
    args = parser.parse_args()
    with open(args.config, encoding="utf-8") as f:
        scenario = json.load(f)

    rclpy.init()
    node = WaveServerNode(args.world, scenario, args.rays_file)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # SIGINT from the launcher shuts the context down under the executor,
        # which then fails to wait on it: that is the normal way out.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
