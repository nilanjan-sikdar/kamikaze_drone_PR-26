"""
control/slew.py — Slew Rate Limiter
=====================================
THIS FILE WAS MISSING — Gemini never created it but FSM was importing it.
That would cause an ImportError on startup.

Limits how fast velocity can change per control tick.
Prevents drone from pitching violently → camera stays stable → tracking stays stable.
"""

import numpy as np
import config


class SlewRateLimiter:
    """
    Limits velocity change to MAX_ACCEL * dt per tick.

    Without this:
      PID says "go 1.0 m/s" → drone pitches 20° instantly
      → camera tilts → target jumps in frame → PID overcorrects → oscillation

    With this:
      PID says "go 1.0 m/s" → velocity ramps: 0 → 0.04 → 0.08 ... → 1.0
      over ~1.25 seconds → gentle tilt → stable camera → stable tracking
    """

    def __init__(self, max_accel: float, dt: float):
        """
        max_accel : maximum acceleration in m/s² (from config.MAX_ACCEL)
        dt        : fixed control timestep in seconds (config.CONTROL_DT)
        """
        self.max_delta = max_accel * dt   # max velocity change per tick
        self.current_v = 0.0              # last output velocity

    def apply(self, desired_v: float) -> float:
        """
        Smooth desired_v through the slew limiter.

        desired_v : what PID computed (unclamped)
        returns   : smoothed velocity that respects acceleration limit
        """
        # How much change does PID want?
        requested_delta = desired_v - self.current_v

        # Clamp to physical acceleration limit
        safe_delta = float(np.clip(requested_delta, -self.max_delta, self.max_delta))

        self.current_v = self.current_v + safe_delta
        return self.current_v

    def reset(self):
        """Reset to zero — call when hovering or on target loss."""
        self.current_v = 0.0