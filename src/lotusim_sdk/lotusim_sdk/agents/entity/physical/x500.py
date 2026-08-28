import os
import re
import signal
import subprocess
import time
from typing import Optional

from lotusim_sdk.agents.physical_entity import PhysicalEntity

# The aerial physics world is always named "aerialWorld" — hardcoded end to end
# (the world file's <world name>, the custom world's AerialEntityManager
# <aerial_namespace>, and the aerial physics_engine_interface namespace all
# agree on it). PX4 attaches to the airframe LOTUSim spawns in this world.
AERIAL_WORLD_NAME = "aerialWorld"


def _px4_cpu_affinity():
    """CPU core list (``taskset -c`` syntax) to pin this PX4 SITL process to.

    Mirrors ``simulation_run.simulation_runner._px4_cpu_affinity()``, which
    pins the aerialWorld gz sim process to the same cores. Both must agree
    so PX4's lockstep clock and the Gazebo IMU it reads share dedicated CPU,
    isolated from the rest of a heavy scenario's agents. See that function's
    docstring for the CPU-contention cause this addresses. Override/disable
    the same way, via ``PX4_CPU_AFFINITY``.
    """
    override = os.environ.get("PX4_CPU_AFFINITY")
    if override is not None:
        return override or None
    cpu_count = os.cpu_count() or 1
    if cpu_count < 4:
        return None
    return f"{cpu_count - 2},{cpu_count - 1}"


class X500(PhysicalEntity):
    MODEL_NAME = "x500"
    # PX4-driven variant: the same airframe with gz MulticopterMotorModel
    # actuators that an external PX4 SITL autopilot writes to. Spawned instead of
    # MODEL_NAME when the scenario JSON sets ``"px4": true`` on the agent.
    PX4_MODEL_NAME = "x500_px4"
    XDYN_PORT = None  # aerial agents never use XDyn
    THRUSTERS = []
    DOMAINS = ["Aerial"]

    # Shared across every X500 in this process so each PX4 SITL launched gets
    # a distinct "-i" instance number. Without it, every instance defaults to
    # 0 -> same MAV_SYS_ID -> QGroundControl merges all heartbeats into one
    # vehicle, and all instances share one working directory (EEPROM/dataman),
    # each overwriting the previous one's.
    _next_px4_instance = 0

    def __init__(
        self,
        sdf_string: str,
        world_name: str,
        xdyn_enabled: bool = False,
        px4_enabled: bool = False,
        px4_control: str = "manual",
        **kwargs,
    ):
        px4_enabled = bool(px4_enabled)
        # Register under the scenario's world (the custom/naval world, e.g.
        # energy), exactly like a non-PX4 X500. The Aerial physics_engine_interface
        # (see PhysicalEntity._lotus_blocks) then makes the custom world's
        # AerialEntityManager forward the real spawn to the aerialWorld physics
        # world AND create a pose-following mirror in the custom world — that
        # mirror is what Unity renders. PX4 later attaches to the airframe the
        # forward created in aerialWorld (see _start_px4_sitl). Registering
        # directly under aerialWorld instead skips the forward, so no mirror is
        # created and the drone never shows up in the custom world / Unity.
        super().__init__(sdf_string, world_name, xdyn_enabled)
        self.px4_enabled = px4_enabled
        self.px4_control = px4_control
        self.px4_process = None
        self.px4_instance: Optional[int] = None  # set once _start_px4_sitl() runs
        if self.px4_enabled:
            # PX4 flies the airframe through gz physics, so spawn the PX4 model.
            # Keep the "x500" renderer type so Unity shows the same drone.
            self.model_name = self.PX4_MODEL_NAME

    # ------------------------------------------------------------------
    # PX4 SITL lifecycle
    # ------------------------------------------------------------------
    def confirm_spawn(self, assigned_name: str) -> None:
        """Launch PX4 SITL once the host confirms the airframe exists.

        ``confirm_spawn`` is the single signal fired by BOTH spawn paths — the
        batch ``MASCmdArray`` path (the default) and the per-agent single-send
        fallback — so PX4 is hooked here rather than in ``send_single_mas_cmd``,
        which the batch path never calls. At this point ``self.agent_name`` is the
        final host-assigned entity name, so PX4 attaches to the right gz model.
        """
        super().confirm_spawn(assigned_name)
        if self.px4_enabled and self.px4_process is None:
            self._start_px4_sitl()

    def _kill_stale_px4_instance(self, px4_binary: str, instance: int) -> None:
        """Kill any leftover PX4 process already holding this instance's lock.

        PX4 takes an flock on a per-``-i`` instance lock file and refuses to
        start a second process for the same instance number: it logs "PX4
        server already running for instance N" and exits, without touching
        the airframe just spawned. PX4 is not a child process of the Gazebo
        processes it flies, so an abrupt end to a previous scenario run
        (closed terminal, plain ``kill`` instead of Ctrl-C, crash) can leave
        a PX4 SITL process alive and orphaned. Each fresh scenario process
        restarts its own ``_next_px4_instance`` counter at 0, so the next
        launch requests the same instance number and is silently refused.
        This sweeps any matching leftover before starting.
        """
        pattern = f"^{re.escape(px4_binary)} -i {instance} -d$"
        try:
            found = subprocess.run(
                ["pgrep", "-f", pattern], capture_output=True, text=True, check=False
            ).stdout.split()
        except FileNotFoundError:
            return  # pgrep unavailable; nothing more we can do here

        if not found:
            return

        self.get_logger().warning(
            f"[{self.agent_name}] Killing {len(found)} stale PX4 instance-{instance} "
            f"process(es) left over from a previous run: {found}"
        )
        for pid in found:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except (ProcessLookupError, ValueError):
                pass

        # SIGKILL releases the instance's flock the moment the kernel reaps
        # the process; poll briefly rather than assuming it's instant.
        for _ in range(20):
            still_there = subprocess.run(
                ["pgrep", "-f", pattern], capture_output=True, text=True, check=False
            ).stdout.split()
            if not still_there:
                return
            time.sleep(0.1)

    def _start_px4_sitl(self) -> None:
        """Start an external PX4 SITL that attaches to the spawned gz airframe.

        Requires a built PX4-Autopilot checkout (``PX4_AUTOPILOT_PATH``, default
        ``~/PX4-Autopilot``). PX4 attaches to the already-spawned entity named
        ``PX4_GZ_MODEL_NAME`` in world ``PX4_GZ_WORLD`` (the aerial world).

        Interactive console (``pxh>``) is not needed here: use QGroundControl's
        Analyze Tools > MAVLink Console instead (connects over MAVLink, no local
        shell required). ``make px4_sitl gz_x500`` doesn't go through PX4's
        classic sitl_run.sh wrapper scripts (that's gazebo-classic/jsbsim only),
        so it ignores ``NO_PXH`` entirely -- setting it does not stop the
        ``pxh>`` prompt redraw loop, which free-spins and writes escape codes
        with nothing on the other end of the pipe to read them. ``make`` only
        adds a "rebuild if stale" check on top of one fixed command (visible
        via ``make``'s own "[0/1] cd .../gz_bridge && cmake -E env ...
        bin/px4" log line once already built), so this invokes that px4
        binary directly with ``-d`` (its own documented "daemon mode, don't
        start pxh shell" flag) instead of going through ``make``. Requires
        ``PX4_AUTOPILOT_PATH`` already built once via ``make px4_sitl
        gz_x500`` -- this does NOT rebuild it.
        """
        instance = X500._next_px4_instance
        X500._next_px4_instance += 1
        # Exposed so a mission task (e.g. px4_offboard_patrol) can compute
        # this vehicle's MAVLink offboard port (14540 + instance, see PX4's
        # px4-rc.mavlink) without re-deriving the spawn-order counter.
        self.px4_instance = instance

        px4_path = os.environ.get("PX4_AUTOPILOT_PATH", os.path.expanduser("~/PX4-Autopilot"))
        # PX4 attaches to the airframe in the aerial world (always "aerialWorld");
        # PX4_GZ_WORLD stays overridable only as an escape hatch for local tests.
        gz_world = os.environ.get("PX4_GZ_WORLD", AERIAL_WORLD_NAME)
        # Prefer LOG_DIR (scenario_launch.sh's per-run scenario_logs/<timestamp>
        # dir, see build_launch_command._with_log for the matching gz sim world
        # logs) so a PX4 flight's whole trace lands in one place instead of a
        # single ever-appended file under PX4_AUTOPILOT_PATH.
        log_dir = os.environ.get("LOG_DIR")
        default_log = (
            os.path.join(log_dir, f"px4_sitl_{self.agent_name}.log")
            if log_dir
            else os.path.join(px4_path, "px4_sitl.log")
        )
        px4_log_path = os.environ.get("PX4_SITL_LOG", default_log)

        # Each PX4 process persists its EEPROM/parameters/dataman/logs relative
        # to its own CWD, so instances sharing a directory overwrite each
        # other's state. Give every instance a private one instead of the
        # single shared gz_bridge source dir used previously.
        instance_dir = os.path.join(px4_path, "build", "px4_sitl_default", f"instance_{instance}")
        os.makedirs(instance_dir, exist_ok=True)

        # A stale dataman file makes PX4 replay a previous mission on boot.
        dataman_path = os.path.join(instance_dir, "dataman")
        if os.path.exists(dataman_path):
            os.remove(dataman_path)
            self.get_logger().info(f"[{self.agent_name}] Removed stale PX4 dataman file.")

        px4_binary = os.path.join(px4_path, "build", "px4_sitl_default", "bin", "px4")
        if not os.path.isfile(px4_binary):
            self.get_logger().error(
                f"[{self.agent_name}] PX4 binary not found at '{px4_binary}'. "
                f"Build it once with: (cd {px4_path} && make px4_sitl gz_x500)"
            )
            return

        self._kill_stale_px4_instance(px4_binary, instance)

        env = os.environ.copy()
        env["GZ_CONFIG_PATH"] = f"/usr/share/gz:{env.get('GZ_CONFIG_PATH', '')}"
        env["PX4_GZ_MODEL_NAME"] = self.agent_name
        env["PX4_GZ_WORLD"] = gz_world
        env["PX4_SIM_MODEL"] = "gz_x500"
        env.setdefault("GZ_IP", "127.0.0.1")

        # -i: instance number (drives MAV_SYS_ID and mavlink port offsets, so
        # QGroundControl sees distinct vehicles instead of merging heartbeats
        # from several instance-0 processes into one).
        # -d: daemon mode, don't start pxh shell (see class docstring above).
        cmd = [px4_binary, "-i", str(instance), "-d"]
        cpu_affinity = _px4_cpu_affinity()
        if cpu_affinity:
            # taskset execs px4_binary directly (no intermediate fork), so
            # the running process's argv/cmdline ends up exactly
            # [px4_binary, "-i", instance, "-d"] either way — the exact-match
            # pgrep pattern in _kill_stale_px4_instance() still matches it.
            cmd = ["taskset", "-c", cpu_affinity] + cmd
        self.get_logger().info(
            f"[{self.agent_name}] Starting PX4 SITL — instance={instance}, "
            f"model='{self.agent_name}', world='{gz_world}'"
        )
        log_file = open(px4_log_path, "a", encoding="utf-8") if px4_log_path else subprocess.DEVNULL
        try:
            self.px4_process = subprocess.Popen(
                cmd,
                cwd=instance_dir,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
            )
        finally:
            if log_file is not subprocess.DEVNULL:
                log_file.close()

    def destroy_node(self):
        if self.px4_process and self.px4_process.poll() is None:
            self.px4_process.terminate()
            try:
                self.px4_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.px4_process.kill()
        return super().destroy_node()
