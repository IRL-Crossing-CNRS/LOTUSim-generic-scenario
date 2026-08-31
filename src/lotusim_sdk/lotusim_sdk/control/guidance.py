#!/usr/bin/env python3
"""Vehicle-agnostic guidance geometry, shared by every LOTUSim agent that
needs to track a straight segment.

Nothing here depends on a vehicle's dynamics: it only converts a position
into a desired heading / depth / cross-track error. How that gets turned into
a force or thruster commands is the caller's job (see
``bluerov_gnc`` for the xdyn/PID example).

Two interchangeable guidance laws are provided, ``LineOfSightGuidance`` and
``PurePursuitGuidance``: same constructor, same ``update()``/``along()``
signature, different geometry for the desired heading.
"""

import math


class _SegmentGuidance:
    """Common geometry for a straight 3D segment (x1,y1,z1) -> (x2,y2,z2):
    along-track/cross-track decomposition and depth interpolation. Frame-
    agnostic: pass NED or ENU consistently and the outputs come back in the
    same convention. Subclasses implement only the desired-heading law.
    """

    def __init__(self, p1, p2, lookahead=5.0):
        self.x1, self.y1, self.z1 = p1
        self.x2, self.y2, self.z2 = p2
        dx, dy = self.x2 - self.x1, self.y2 - self.y1
        self.length_h = math.hypot(dx, dy)
        if self.length_h < 1e-9:
            raise ValueError("segment has zero horizontal length")
        self.heading = math.atan2(dy, dx)
        self.lookahead = lookahead

    def along(self, x, y):
        """Along-track distance from the start, in metres.

        Compare against ``length_h`` to detect arrival: ``update()`` clamps only
        the depth interpolation, so nothing else in this class tells a caller
        the segment is finished.
        """
        dx, dy = x - self.x1, y - self.y1
        return dx * math.cos(self.heading) + dy * math.sin(self.heading)

    def _along_cross(self, x, y):
        dx, dy = x - self.x1, y - self.y1
        ch, sh = math.cos(self.heading), math.sin(self.heading)
        along = dx * ch + dy * sh  # along-track distance
        cross = -dx * sh + dy * ch  # signed cross-track (>0 to port)
        return along, cross

    def _depth_at(self, along):
        frac = max(0.0, min(1.0, along / self.length_h))
        return self.z1 + frac * (self.z2 - self.z1)

    def update(self, x, y):
        """-> (desired heading [rad], desired depth/altitude [m], cross-track [m])"""
        raise NotImplementedError


class LineOfSightGuidance(_SegmentGuidance):
    """Line-of-sight guidance: aim at a point ``lookahead`` metres ahead of
    the vehicle's own along-track projection onto the line, with a heading
    offset proportional to `atan2(cross-track, lookahead)`.
    """

    def update(self, x, y):
        along, cross = self._along_cross(x, y)
        z_sp = self._depth_at(along)
        psi_sp = self.heading + math.atan2(-cross, self.lookahead)
        return psi_sp, z_sp, cross


class PurePursuitGuidance(_SegmentGuidance):
    """Classic pure-pursuit guidance: aim directly at the point on the line
    that is exactly ``lookahead`` metres from the vehicle's CURRENT position
    (not from its along-track projection), found as the forward intersection
    of that circle with the line. Differs from ``LineOfSightGuidance``
    especially at large cross-track error, where it steers back toward the
    line more aggressively.
    """

    def update(self, x, y):
        along, cross = self._along_cross(x, y)
        # Forward intersection of the lookahead circle (radius `lookahead`,
        # centred on the vehicle) with the line: Pythagoras on the
        # perpendicular distance |cross|. Always >= along (monotonic forward
        # progress), so the carrot never falls behind the vehicle.
        reach = max(self.lookahead * self.lookahead - cross * cross, 0.0)
        carrot_s = along + math.sqrt(reach)
        carrot_s_clamped = max(0.0, min(self.length_h, carrot_s))
        cx = self.x1 + carrot_s_clamped * math.cos(self.heading)
        cy = self.y1 + carrot_s_clamped * math.sin(self.heading)
        psi_sp = math.atan2(cy - y, cx - x)
        z_sp = self._depth_at(along)
        return psi_sp, z_sp, cross
