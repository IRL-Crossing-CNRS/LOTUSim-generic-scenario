from __future__ import annotations

import math
from typing import Optional

from lotusim_sdk.tasks.waypoint_follower import WaypointFollowerTask


class WaypointFollowerAvoidanceTask(WaypointFollowerTask):
    """``WaypointFollowerTask`` plus a fake sonar and obstacle avoidance.

    "Fake sonar" means a ground-truth proximity check against other known
    entities' poses (``host.poses_of_others()``, fed by the same
    ``/<world>/poses`` topic the pursuit controller already reads for its
    own position) — not a simulated acoustic/Gazebo sensor. Any entity whose
    name starts with one of ``obstacle_prefixes`` (default: mines) within
    ``sonar_range_m`` is a "contact".

    Avoidance only ever bends the STEERING BEARING (via
    :meth:`WaypointFollowerTask._effective_goal_vector`): the nearest
    contact's repulsion vector is blended with the true goal direction,
    growing smoothly from zero at ``sonar_range_m`` to full strength at
    zero range, so there is no hard on/off chattering at the range boundary.
    Speed is not touched directly — the base controller's existing heading-
    coupled speed ceiling already slows the agent as the (now avoidance-
    bent) bearing diverges from the true goal.

    The repulsion is NOT purely radial (straight away from the obstacle):
    it's rotated by a fixed ``_AVOID_BIAS_RAD`` before blending. Pure radial
    repulsion + attraction is a well-known potential-field trap (Koren &
    Borenstein 1991) — the two can settle into a stable circular orbit at a
    fixed standoff radius, with the agent circling the obstacle at a fixed
    distance and heading rotating forever, distance-to-target never
    decreasing. Rotating the repulsion by a fixed angle breaks that radial
    symmetry and turns the would-be orbit into an outward spiral that
    clears the obstacle.

    Params (mission JSON ``"params"``, all optional, in addition to the base
    task's params):
        sonar_range_m       detection range in metres AND the distance over
                             which repulsion strength ramps to zero (default
                             40.0 — one knob, matching a single "sonar
                             range").
        obstacle_prefixes   list of agent-name prefixes treated as
                             obstacles (default ``["mine"]``).
        avoid_gain           repulsion weight relative to the (unit) goal
                             direction (default 1.5).
    """

    # Fixed rotation applied to the "away from obstacle" vector before it's
    # blended with the goal direction (see class docstring for why this is
    # needed, not optional polish). Always the same rotational sense so the
    # avoidance decision doesn't flip-flop. 70 deg is the centre of the safe
    # range (head-on/off-axis/multi-obstacle/drifting-obstacle cases escape
    # reliably for roughly 50-85 deg; below that it reduces to the orbiting
    # failure mode, above ~95 deg the repulsion overshoots into a different
    # stuck configuration).
    _AVOID_BIAS_RAD = math.radians(70.0)

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        self._sonar_range_m = float(p.get("sonar_range_m", 40.0))
        self._obstacle_prefixes = tuple(p.get("obstacle_prefixes", ["mine"]))
        self._avoid_gain = float(p.get("avoid_gain", 1.5))
        # Whether a contact is currently in range — tracked only so
        # _log_contact_change can print ONE line per start/clear event
        # instead of every 20 Hz control tick. Deliberately NOT the contact's
        # name/identity: a real sonar reports range, not "this is mine0 vs
        # mine1" — that only exists in the ground-truth CSV recorder (an
        # external observer/black-box recorder), never in this task's own
        # logic or its own terminal output.
        self._in_contact = False

    def _nearest_obstacle(self, x: float, y: float) -> Optional[tuple]:
        nearest = None
        nearest_dist = None
        for name, pose in self.host.poses_of_others().items():
            if not name.startswith(self._obstacle_prefixes):
                continue
            dist = math.hypot(pose.position.x - x, pose.position.y - y)
            if nearest_dist is None or dist < nearest_dist:
                nearest, nearest_dist = (pose.position.x, pose.position.y, dist), dist
        return nearest

    def _log_contact_change(self, in_contact: bool, dist: float) -> None:
        """Print ONE line to the terminal when sonar contact starts or
        clears — range only, no object identity (see __init__)."""
        if in_contact == self._in_contact:
            return
        if in_contact:
            self.host.get_logger().info(
                f"{self.host.agent_name}: sonar contact at {dist:.1f} m "
                f"(range {self._sonar_range_m:.0f} m) - engaging avoidance"
            )
        else:
            self.host.get_logger().info(f"{self.host.agent_name}: clear, resuming direct course")
        self._in_contact = in_contact

    def _effective_goal_vector(
        self,
        x: float,
        y: float,
        goal_x: float,
        goal_y: float,
        dx_true: float,
        dy_true: float,
    ) -> tuple:
        obstacle = self._nearest_obstacle(x, y)
        if obstacle is None:
            self._log_contact_change(False, 0.0)
            return dx_true, dy_true
        ox, oy, dist = obstacle
        if dist >= self._sonar_range_m:
            self._log_contact_change(False, 0.0)
            return dx_true, dy_true
        self._log_contact_change(True, dist)

        goal_dist = math.hypot(dx_true, dy_true) or 1.0
        gx, gy = dx_true / goal_dist, dy_true / goal_dist

        away_x, away_y = x - ox, y - oy
        away_norm = math.hypot(away_x, away_y) or 1.0
        away_x, away_y = away_x / away_norm, away_y / away_norm

        # Rotate the radial "away" vector by a fixed bias angle (see class
        # docstring) instead of using it as-is.
        cos_b, sin_b = math.cos(self._AVOID_BIAS_RAD), math.sin(self._AVOID_BIAS_RAD)
        biased_x = away_x * cos_b - away_y * sin_b
        biased_y = away_x * sin_b + away_y * cos_b

        strength = self._avoid_gain * (1.0 - dist / self._sonar_range_m)
        bx, by = gx + strength * biased_x, gy + strength * biased_y
        blend_norm = math.hypot(bx, by)
        if blend_norm < 1e-3:
            # Obstacle sits ~directly between the agent and the goal: the
            # goal-seeking and repulsion vectors nearly cancel. Break the tie
            # deterministically (always the same rotation) instead of
            # feeding a near-zero/undefined vector into atan2, which would
            # make the commanded heading unstable.
            bx, by = -gy, gx
            blend_norm = 1.0

        return bx / blend_norm * goal_dist, by / blend_norm * goal_dist
