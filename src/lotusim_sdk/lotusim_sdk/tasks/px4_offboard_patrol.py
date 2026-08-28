from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent

# PX4's own custom-mode encoding for OFFBOARD (see PX4-Autopilot's
# px4_custom_mode.h — PX4_CUSTOM_MAIN_MODE_OFFBOARD). Not exposed by
# pymavlink's mavlink dialect (that only carries the generic MAV_MODE_FLAG_*
# bits), so it's hardcoded here same as every other PX4 companion-computer
# script does.
_PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

# SET_POSITION_TARGET_LOCAL_NED type_mask: use position (x, y, z) only, ignore
# velocity/acceleration/yaw/yaw_rate (bits 3-10 set, per MAVLink's
# POSITION_TARGET_TYPEMASK bit layout).
_POSITION_ONLY_TYPE_MASK = 0b0000_1111_1111_1000

# MAV_LANDED_STATE_ON_GROUND (EXTENDED_SYS_STATE.landed_state) — PX4's own
# land-detector output.
_MAV_LANDED_STATE_ON_GROUND = 1


@dataclass
class Waypoint:
    name: str
    x: float  # world frame, same units/origin as this agent's own "spawn" block
    y: float
    z: float


class Px4OffboardPatrolTask(TaskAgent):
    """Flies a PX4-SITL-controlled X500 through a list of waypoints over
    MAVLink offboard control. Autonomous equivalent of flying the same
    drone by hand in QGroundControl.

    ``waypoint_follower`` (used by non-PX4 ``X500_inspection`` agents) does
    not apply here: it integrates motion host-side via the Gazebo
    ``KinematicInterface`` plugin and never touches PX4. A PX4-armed
    airframe's motion has to be commanded over MAVLink instead.
    ``px4_control`` on :class:`X500` is currently stored but unused by any
    task; this is the first task that uses it.

    Flight sequence: confirm required params -> arm -> ``MAV_CMD_NAV_TAKEOFF``
    -> wait for airborne -> switch to OFFBOARD -> stream position setpoints.
    The param stage gates every later stage (see ``_drive_param_stage``):
    each param is resent until PX4 echoes it back via ``PARAM_VALUE``, since
    ``param_set_send`` is one-way UDP with no acknowledgment — an unconfirmed
    send is not evidence PX4 applied it before this task tries to arm.
    Arming and streaming OFFBOARD position setpoints from the ground is not
    sufficient by itself:
    PX4's land-detector gates thrust output to zero while its internal state
    is "landed", regardless of the offboard position setpoint, until an
    explicit takeoff command clears it (observed via ``VFR_HUD``/
    ``EXTENDED_SYS_STATE``: ``throttle: 0``, ``landed_state: ON_GROUND``
    indefinitely without one). ``MAV_CMD_NAV_TAKEOFF`` is sent with a NaN
    altitude field; the target altitude comes from the ``MIS_TAKEOFF_ALT``
    parameter (set from ``takeoff_alt_m`` in ``on_enter``) instead, to avoid
    the command field's AMSL frame.

    Coordinate frame: PX4's local NED origin is set at the airframe's spawn
    point. Gazebo's world frame is ENU-aligned with PX4's NED axes. For a
    waypoint at world (x, y, z) and this agent's spawn at (spawn_x, spawn_y,
    spawn_z):

        north = y - spawn_y
        east  = x - spawn_x
        down  = -(z - spawn_z)

    ``spawn`` is passed as a task param rather than looked up elsewhere —
    this codebase has no cross-agent config lookup (see
    ``doc/WRITE_SCENARIO.md``), matching ``waypoint_follower``'s own
    ``waypoints`` param. Use the same x/y/z already in this agent's own
    scenario-JSON ``"spawn"`` block.

    Params:
        spawn: {"x", "y", "z"} — this agent's own spawn position (required).
        waypoints: [{"name"?, "x", "y", "z"}, ...] — e.g. a turbine list
            (required, at least one entry).
        loop: bool, default True — cycle back to the first waypoint after the
            last instead of finishing.
        hold_radius_m: float, default 5.0 — distance to a waypoint (3D) at
            which it counts as reached.
        takeoff_alt_m: float, default 15.0 — climb straight up to this
            altitude (above spawn) before turning toward the first waypoint,
            so the very first offboard setpoint isn't a long diagonal climb
            straight out of the ground.
        setpoint_rate_hz: float, default 10.0 — offboard position-setpoint
            stream rate. PX4 requires a continuous stream (its own internal
            timeout is 500 ms) or it falls back out of OFFBOARD — must stay
            comfortably above 2 Hz.
        arm_after_setpoints: int, default 30 — number of setpoints/heartbeats
            sent before requesting arm. PX4 requires an existing setpoint
            stream before accepting OFFBOARD, and a data-link health check
            (cleared by our own outgoing heartbeats) before accepting arm.
            At the default 10 Hz setpoint rate this is a 3 s wait.
        connect_timeout_s: float, default 15.0 — how long to wait for PX4's
            first heartbeat before giving up (FAILURE) in ``on_enter``.
    """

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)

        spawn = self.params.get("spawn") or {}
        self._spawn = (
            float(spawn.get("x", 0.0)),
            float(spawn.get("y", 0.0)),
            float(spawn.get("z", 0.0)),
        )

        self._waypoints: List[Waypoint] = [
            Waypoint(
                name=wp.get("name", f"wp{i}"),
                x=float(wp["x"]),
                y=float(wp["y"]),
                z=float(wp["z"]),
            )
            for i, wp in enumerate(self.params.get("waypoints") or [])
        ]

        self.loop: bool = bool(self.params.get("loop", True))
        self.PERPETUAL = self.loop  # looping patrol never finishes on its own

        self._hold_radius_m: float = float(self.params.get("hold_radius_m", 5.0))
        self._takeoff_alt_m: float = float(self.params.get("takeoff_alt_m", 15.0))
        self._setpoint_period_s: float = 1.0 / float(self.params.get("setpoint_rate_hz", 10.0))
        self._arm_after_setpoints: int = int(self.params.get("arm_after_setpoints", 30))
        self._connect_timeout_s: float = float(self.params.get("connect_timeout_s", 15.0))

        self._master = None
        self._timer = None
        self._setpoints_sent = 0

        # Params this task needs PX4 to hold before arming (see on_enter's
        # docstring reference and _step's param stage) — name -> (value,
        # mavlink type). Sent and re-sent until each is confirmed by its own
        # PARAM_VALUE echo, same as commands below are confirmed by
        # COMMAND_ACK: param_set_send is one-way UDP with no built-in
        # acknowledgment, so an unconfirmed send is not evidence PX4 received
        # or applied it.
        self._pending_params: Dict[bytes, Tuple[float, int]] = {}
        self._params_confirmed = False

        # Flight sequence state (see class docstring). Each *_acked flag is
        # set only from a confirmed COMMAND_ACK, not from having sent the
        # request — PX4 can silently ignore or reject a request.
        self._arm_acked = False
        self._takeoff_acked = False
        self._airborne = False
        self._offboard_acked = False
        self._offboard_engaged = False  # True once actually patrolling under OFFBOARD control
        self._last_command_attempt_step = -1_000_000  # steps ago; forces an immediate first attempt

        self._target_index = 0
        self._done = False
        self._failed = False

    # ------------------------------------------------------------------

    def on_enter(self) -> None:
        logger = self.host.get_logger()

        if not self._waypoints:
            logger.error(f"[{self.host.agent_name}] px4_offboard_patrol: no waypoints given.")
            self._failed = True
            return

        instance = getattr(self.host, "px4_instance", None)
        if instance is None:
            logger.error(
                f"[{self.host.agent_name}] px4_offboard_patrol: host has no "
                "px4_instance — is this an X500 agent with \"px4\": true, and "
                "has PX4 SITL finished starting (confirm_spawn already fired)?"
            )
            self._failed = True
            return

        try:
            from pymavlink import mavutil
        except ImportError:
            logger.error(
                f"[{self.host.agent_name}] px4_offboard_patrol: pymavlink not "
                "installed. It's a PX4 build dependency "
                "(pip3 install --user pymavlink) — see the README's PX4 setup section."
            )
            self._failed = True
            return

        # PX4's offboard/companion-computer link: it sends its stream to
        # "remote port" 14540 + instance (see PX4-Autopilot's
        # ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink), and pymavlink's
        # udpin mode both listens there AND learns the reply address from
        # whatever source port that stream actually arrives from — no need to
        # separately know PX4's own listening port (14580 + instance).
        port = 14540 + instance
        logger.info(f"[{self.host.agent_name}] px4_offboard_patrol: connecting on udp:0.0.0.0:{port}...")
        self._master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")

        if self._master.wait_heartbeat(timeout=self._connect_timeout_s) is None:
            logger.error(
                f"[{self.host.agent_name}] px4_offboard_patrol: no MAVLink "
                f"heartbeat on port {port} within {self._connect_timeout_s}s."
            )
            self._master.close()
            self._master = None
            self._failed = True
            return

        logger.info(
            f"[{self.host.agent_name}] px4_offboard_patrol: heartbeat received "
            f"(sysid={self._master.target_system}, compid={self._master.target_component}) "
            f"— streaming setpoints, {len(self._waypoints)} waypoint(s), loop={self.loop}."
        )

        # Params PX4 must hold before this task proceeds to arm (see _step's
        # param stage, which sends-and-confirms each of these before the
        # arm/takeoff/offboard sequence starts):
        #
        # COM_DISARM_PRFLT=0 — disables the auto-disarm-if-not-airborne
        # -within-N-seconds preflight guard (Commander::handleAutoDisarm(),
        # arm_disarm_reason_t::auto_disarm_preflight). It assumes a human
        # safety pilot is present and disarms mid-climb otherwise.
        #
        # MIS_TAKEOFF_ALT — MAV_CMD_NAV_TAKEOFF's own altitude field is AMSL
        # and needs the vehicle's home altitude to interpret correctly;
        # MIS_TAKEOFF_ALT (relative-to-home meters) sidesteps that entirely —
        # the command is sent below with a NaN altitude so PX4 falls back to
        # this param.
        #
        # COM_RC_IN_MODE=4 / NAV_RCL_ACT=0 — there is no RC transmitter or
        # joystick behind an offboard patrol, but PX4 assumes one by default
        # (COM_RC_IN_MODE=3 "RC or Joystick keep first", NAV_RCL_ACT=2
        # "Return mode"), so the manual-control failsafe fires seconds into
        # the climb and hands the airframe to RTL — observed as repeated
        # "Failsafe activated" with the vehicle never leaving the ground, and
        # every DO_SET_MODE(OFFBOARD) answered TEMPORARILY_REJECTED because a
        # failsafe outranks the mode request. 4 = "Stick input disabled" (no
        # manual source expected at all); NAV_RCL_ACT 0 disables the
        # RC-loss reaction that follows from it.
        #
        # CBRK_SUPPLY_CHK / COM_LOW_BAT_ACT=0 — SITL has no real power rail,
        # so the "system power unavailable" preflight check never clears
        # without the documented circuit-breaker bypass value, and the
        # simulated battery's warnings must not escalate into a Land/Return
        # failsafe mid-patrol — 0 = warn only, leaving the warning visible in
        # QGC without taking the airframe off its mission.
        self._pending_params = {
            b"COM_DISARM_PRFLT": (0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            b"MIS_TAKEOFF_ALT": (self._takeoff_alt_m, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            b"COM_RC_IN_MODE": (4.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            b"NAV_RCL_ACT": (0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            b"CBRK_SUPPLY_CHK": (894281.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            b"COM_LOW_BAT_ACT": (0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
        }

        self._timer = self.host.create_timer(self._setpoint_period_s, self._step)

    def on_exit(self, _status: Status) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self.host.destroy_timer(self._timer)
            self._timer = None
        if self._master is not None:
            self._master.close()
            self._master = None

    def update(self) -> Status:
        if self._failed:
            return Status.FAILURE
        if self._done:
            return Status.SUCCESS
        return Status.RUNNING

    # ------------------------------------------------------------------

    def _current_target_ned(self) -> Tuple[float, float, float]:
        """North/east/down of the current waypoint target, relative to spawn."""
        wp = self._waypoints[self._target_index]
        spawn_x, spawn_y, spawn_z = self._spawn
        north = wp.y - spawn_y
        east = wp.x - spawn_x
        down = -(wp.z - spawn_z)
        return north, east, down

    def _step(self) -> None:
        if self._master is None or self._done:
            return

        from pymavlink import mavutil

        # PX4's GCS-connection pre-arm check (rcAndDataLinkCheck.cpp,
        # NAV_DLL_ACT-gated) requires an outgoing heartbeat from this side,
        # not just having received PX4's. 1 Hz is the MAVLink-standard
        # heartbeat rate; sent on every setpoint tick rather than a second
        # timer, which exceeds that minimum.
        self._master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0,
        )

        # Drain any pending telemetry (non-blocking): a position fix to judge
        # arrival/altitude, EXTENDED_SYS_STATE for the land-detector's own
        # airborne verdict, PARAM_VALUE confirming a param stage request, and
        # any COMMAND_ACK for the flight-sequence requests below.
        pos = None
        while True:
            msg = self._master.recv_match(
                type=["LOCAL_POSITION_NED", "EXTENDED_SYS_STATE", "COMMAND_ACK", "PARAM_VALUE"],
                blocking=False,
            )
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "LOCAL_POSITION_NED":
                pos = msg
            elif msg_type == "EXTENDED_SYS_STATE":
                if msg.landed_state != _MAV_LANDED_STATE_ON_GROUND:
                    self._airborne = True
            elif msg_type == "PARAM_VALUE":
                self._handle_param_value(msg)
            else:
                self._handle_command_ack(msg)

        # Even before OFFBOARD is actually engaged, keep the setpoint stream
        # warm targeting the first waypoint's lateral position at takeoff
        # altitude — required for PX4 to accept the eventual mode switch,
        # and means the patrol heads the right way the moment it does.
        if self._offboard_engaged:
            north, east, down = self._current_target_ned()
        else:
            wp0 = self._waypoints[0]
            spawn_x, spawn_y, _ = self._spawn
            north, east, down = wp0.y - spawn_y, wp0.x - spawn_x, -self._takeoff_alt_m
        self._master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms — unused by PX4 when receiving, 0 is fine
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _POSITION_ONLY_TYPE_MASK,
            north, east, down,
            0, 0, 0,  # vx, vy, vz — ignored (type_mask)
            0, 0, 0,  # afx, afy, afz — ignored (type_mask)
            0, 0,  # yaw, yaw_rate — ignored (type_mask)
        )
        self._setpoints_sent += 1

        # Airborne fallback: EXTENDED_SYS_STATE isn't guaranteed on every
        # mavlink instance/PX4 build. If altitude has clearly left the
        # ground, that's proof enough on its own.
        if pos is not None and -pos.z > 1.0:
            self._airborne = True

        if not self._params_confirmed:
            self._drive_param_stage()
            return

        if not self._offboard_engaged:
            if self._setpoints_sent >= self._arm_after_setpoints:
                self._drive_takeoff_sequence()
            return

        if pos is None:
            return

        dn = pos.x - north
        de = pos.y - east
        dd = pos.z - down
        distance = (dn * dn + de * de + dd * dd) ** 0.5
        if distance <= self._hold_radius_m:
            self._advance_target()

    def _ready_to_retry(self) -> bool:
        """1 Hz retry/step gate for the one-shot commands below, expressed in
        setpoint-timer ticks so it needs no separate timer/clock source."""
        retry_period_steps = max(1, round(1.0 / self._setpoint_period_s))
        if self._setpoints_sent - self._last_command_attempt_step < retry_period_steps:
            return False
        self._last_command_attempt_step = self._setpoints_sent
        return True

    def _drive_param_stage(self) -> None:
        """Send (and resend, until each is confirmed) every param in
        ``_pending_params``, gating the rest of the flight sequence.

        ``param_set_send`` is one-way UDP with no acknowledgment built in —
        unlike every other request this task makes, which pymavlink itself
        exposes as a ``COMMAND_ACK``'d ``command_long_send``. A dropped or
        premature (PX4 not yet ready) param set is otherwise indistinguishable
        from a successful one until the failsafe it was meant to prevent
        fires anyway, which is what made this flaky: identical params, same
        code, and whether PX4 actually applied them before arming varied
        run to run.
        """
        if not self._pending_params:
            self._params_confirmed = True
            return
        if not self._ready_to_retry():
            return
        for name, (value, mav_type) in self._pending_params.items():
            self._master.mav.param_set_send(
                self._master.target_system, self._master.target_component,
                name, value, mav_type,
            )

    def _handle_param_value(self, msg) -> None:
        # pymavlink decodes PARAM_VALUE's char[16] param_id to str, but
        # doesn't consistently strip the field's null-padding across
        # versions/dialects — normalize before comparing against the bytes
        # keys _pending_params was built with.
        raw_id = msg.param_id
        param_id_str = raw_id.decode() if isinstance(raw_id, bytes) else raw_id
        name = param_id_str.rstrip("\x00").encode()
        pending = self._pending_params.get(name)
        if pending is None:
            return
        # PX4 broadcasts PARAM_VALUE both as our set's echo and unsolicited
        # (e.g. its own startup param dump) — only the value actually
        # matching what was requested counts as confirmation, otherwise a
        # stale echo of the pre-set default would be mistaken for success.
        requested_value, _mav_type = pending
        if abs(msg.param_value - requested_value) > 1e-3:
            return
        del self._pending_params[name]
        logger = self.host.get_logger()
        logger.info(
            f"[{self.host.agent_name}] px4_offboard_patrol: {name.decode()} "
            f"confirmed = {msg.param_value}"
        )
        if not self._pending_params:
            self._params_confirmed = True
            logger.info(f"[{self.host.agent_name}] px4_offboard_patrol: all params confirmed.")

    def _drive_takeoff_sequence(self) -> None:
        """Step the arm -> takeoff -> wait-airborne -> OFFBOARD sequence.

        Each stage only advances once acked, and (for takeoff) once PX4's
        own telemetry confirms it's airborne, not once a request is sent.
        See the class docstring: plain arm-then-offboard leaves PX4 at zero
        thrust and landed_state=ON_GROUND indefinitely.
        """
        logger = self.host.get_logger()

        if self._offboard_acked:
            self._offboard_engaged = True
            return

        if self._airborne:
            if self._ready_to_retry():
                from pymavlink import mavutil

                self._master.mav.command_long_send(
                    self._master.target_system, self._master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    _PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, 0, 0, 0, 0,
                )
                logger.info(f"[{self.host.agent_name}] px4_offboard_patrol: airborne — requesting OFFBOARD...")
            return

        if self._takeoff_acked:
            return  # waiting to become airborne; nothing to (re)send

        if not self._ready_to_retry():
            return

        from pymavlink import mavutil

        if not self._arm_acked:
            self._master.mav.command_long_send(
                self._master.target_system, self._master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0,
            )
            logger.info(f"[{self.host.agent_name}] px4_offboard_patrol: requesting ARM...")
        else:
            # NaN altitude: fall back to the MIS_TAKEOFF_ALT param set in
            # on_enter rather than dealing with MAV_CMD_NAV_TAKEOFF's own
            # AMSL-referenced altitude field.
            self._master.mav.command_long_send(
                self._master.target_system, self._master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, float("nan"),
            )
            logger.info(f"[{self.host.agent_name}] px4_offboard_patrol: armed — requesting TAKEOFF...")

    def _handle_command_ack(self, ack) -> None:
        from pymavlink import mavutil

        logger = self.host.get_logger()
        result_name = mavutil.mavlink.enums["MAV_RESULT"].get(ack.result)
        result_name = result_name.name if result_name is not None else str(ack.result)
        accepted = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

        if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            self._arm_acked = self._arm_acked or accepted
            self._log_ack("ARM", result_name, accepted)
        elif ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            self._takeoff_acked = self._takeoff_acked or accepted
            self._log_ack("TAKEOFF", result_name, accepted)
        elif ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            was_acked = self._offboard_acked
            self._offboard_acked = self._offboard_acked or accepted
            self._log_ack("OFFBOARD", result_name, accepted)
            if self._offboard_acked and not was_acked:
                logger.info(f"[{self.host.agent_name}] px4_offboard_patrol: in OFFBOARD — patrol starting.")

    def _log_ack(self, request: str, result_name: str, accepted: bool) -> None:
        """Report a COMMAND_ACK at info (accepted) or warning (anything else).

        The two severities need their own physically distinct call sites:
        rclpy caches a logger's severity per caller location, so reusing one
        line for both (``log = logger.info if accepted else logger.warning``)
        raises "Logger severity cannot be changed between calls" the first
        time a request's result flips — which is exactly what OFFBOARD does,
        answering TEMPORARILY_REJECTED until PX4 accepts it.
        """
        logger = self.host.get_logger()
        message = f"[{self.host.agent_name}] px4_offboard_patrol: {request} request -> {result_name}"
        if accepted:
            logger.info(message)
        else:
            logger.warning(message)

    def _advance_target(self) -> None:
        logger = self.host.get_logger()
        reached = self._waypoints[self._target_index]
        logger.info(f"[{self.host.agent_name}] px4_offboard_patrol: reached '{reached.name}'.")

        self._target_index += 1
        if self._target_index >= len(self._waypoints):
            if self.loop:
                self._target_index = 0
            else:
                self._done = True
