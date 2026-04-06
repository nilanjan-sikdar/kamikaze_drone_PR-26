"""
control/pid.py — PID Controller
================================
BUG FIX: D-term now uses fixed CONTROL_DT instead of wall-clock dt.

WHY THIS MATTERS:
  Old code: dt = current_time - last_time  (varies per frame)
  Fast frame: dt=0.01s → d = error/0.01 = 10x spike
  Slow frame: dt=0.10s → d = error/0.10 = normal
  Same error, 10x different D-term → random lurching

  Fixed: always use CONTROL_DT = 0.05s
  D-term is now stable and predictable every tick.
"""

import numpy as np
import config


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self._prev_error  = 0.0
        self._integral    = 0.0
        self._initialized = False   # first call uses P only, no D spike

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, error: float) -> float:
        """
        Compute PID output using FIXED timestep (config.CONTROL_DT).
        Call exactly once per control tick at 20Hz.
        """
        dt = config.CONTROL_DT   # FIXED — never use wall-clock dt here

        # Proportional
        p = self.kp * error

        # Integral with basic windup clamp
        self._integral += error * dt
        self._integral  = float(np.clip(self._integral, -2.0, 2.0))
        i = self.ki * self._integral

        # Derivative — P-only on first call to prevent spike from prev_error=0
        if not self._initialized:
            d = 0.0
            self._initialized = True
        else:
            d = self.kd * (error - self._prev_error) / dt

        self._prev_error = error
        return float(np.clip(p + i + d, -config.MAX_VX, config.MAX_VY))

    def reset(self):
        self._integral    = 0.0
        self._prev_error  = 0.0
        self._initialized = False