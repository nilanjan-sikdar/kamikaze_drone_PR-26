"""
control/intercept_controller.py — Predictive Intercept Controller
==================================================================

Inspired by the MRS UAV System's Se(3) geometric controller philosophy
adapted for pymavlink + ArduPilot.

Key ideas taken from MRS:
  1. Separate the *reference generator* (where should I go?) from
     the *tracker* (how do I get there?).
  2. Use TIME-TO-INTERCEPT as the primary planning variable.
  3. Feed-forward the target's estimated velocity so the drone leads
     the target rather than chasing its current position (like a dog
     chasing a car by always running toward where the car *was*).
  4. Saturate commands smoothly with a sigmoid-style deadband so the
     drone doesn't jitter when almost on-target.

Control law (image-space, then projected to body velocity):
────────────────────────────────────────────────────────────
  error_px  = target_predicted_cx - frame_cx    (pixels, lateral)
  error_py  = target_predicted_cy - frame_cy    (pixels, vertical)
  error_sz  = target_predicted_w  - approach_w  (size error → forward vel)

  vel_x = Kp_lat  * error_px  +  Kd_lat  * target_vx   ← feed-forward
  vel_y = Kp_lat  * error_py  +  Kd_lat  * target_vy
  vel_z = Kp_fwd  * error_sz                            ← close the range

All velocities are clamped to configurable maxima.

The caller decides the lookahead time (intercept_dt).
A good default: distance_estimate / max_speed  (time to reach the target).
"""

import math
import config


class InterceptController:
    """
    Converts Kalman-estimated + predicted target pixel state → body-frame
    velocity commands (vx_body, vy_body, vz_body) ready to send via pymavlink
    SET_POSITION_TARGET_LOCAL_NED in velocity-only mode.

    Coordinate convention (body frame, ArduPilot):
        +X  = forward
        +Y  = right
        +Z  = down    ← note: positive Z means descend in ArduPilot NED
    """

    # ── Gains (tune these for your airframe & speed) ──────────────────────
    # Lateral: how hard to chase left/right pixel error
    Kp_LAT   = 0.003   # m/s per pixel
    Kd_LAT   = 0.8     # feed-forward weight on target pixel velocity

    # Forward: how hard to close range based on bounding-box size
    # Positive error_sz → target is smaller than desired → fly forward
    Kp_FWD   = 0.05    # m/s per pixel of size error

    # Vertical: separate gain for altitude alignment
    Kp_VERT  = 0.003   # m/s per pixel of vertical error

    # ── Saturation limits ─────────────────────────────────────────────────
    MAX_LAT_VEL  = getattr(config, 'MAX_LAT_VEL',  3.0)   # m/s lateral
    MAX_FWD_VEL  = getattr(config, 'MAX_FWD_VEL',  8.0)   # m/s forward
    MAX_VERT_VEL = getattr(config, 'MAX_VERT_VEL',  2.0)  # m/s vertical

    # Desired target bounding-box width at intercept (pixels).
    # When target fills this many pixels wide, we're close enough.
    # Tune for your camera FOV and desired impact distance.
    TARGET_W_AT_INTERCEPT = getattr(config, 'TARGET_W_AT_INTERCEPT', 240)

    # Deadband: ignore tiny errors to prevent jitter
    DEADBAND_PX = 5.0

    def __init__(self, frame_w: int = 640, frame_h: int = 480):
        self.frame_cx = frame_w / 2.0
        self.frame_cy = frame_h / 2.0

    # ── main interface ─────────────────────────────────────────────────────

    def compute(
        self,
        pred_cx: float, pred_cy: float,
        pred_w:  float,
        target_vx: float, target_vy: float,
        intercept_dt: float = 0.5,
    ) -> tuple[float, float, float]:
        """
        Args:
            pred_cx, pred_cy : predicted target centre (pixels) at intercept_dt ahead
            pred_w           : predicted target bounding-box width (pixels)
            target_vx, vy    : target pixel velocity (from Kalman)
            intercept_dt     : lookahead horizon (seconds); shrinks as we close in

        Returns:
            (vx_body, vy_body, vz_body) in m/s, ArduPilot NED convention
            +X=forward, +Y=right, +Z=down
        """
        # ── pixel errors ──────────────────────────────────────────────────
        err_lat  =  pred_cx - self.frame_cx   # + → target is right of centre
        err_vert =  pred_cy - self.frame_cy   # + → target is below centre
        err_fwd  =  pred_w  - self.TARGET_W_AT_INTERCEPT  # + → too far

        # ── deadband ──────────────────────────────────────────────────────
        err_lat  = self._deadband(err_lat,  self.DEADBAND_PX)
        err_vert = self._deadband(err_vert, self.DEADBAND_PX)

        # ── proportional + feed-forward ───────────────────────────────────
        #   vel_right  =  Kp * lateral_error  +  Kd * target_lateral_velocity
        #   feed-forward term makes the drone lead the target like a missile
        vel_right   = self.Kp_LAT  * err_lat   + self.Kd_LAT * target_vx
        vel_down    = self.Kp_VERT * err_vert  + self.Kd_LAT * target_vy
        vel_forward = self.Kp_FWD  * err_fwd

        # ── saturate ──────────────────────────────────────────────────────
        vel_right   = self._clamp(vel_right,   self.MAX_LAT_VEL)
        vel_down    = self._clamp(vel_down,    self.MAX_VERT_VEL)
        vel_forward = self._clamp(vel_forward, self.MAX_FWD_VEL)

        # ArduPilot body-frame NED: X=fwd, Y=right, Z=down
        return vel_forward, vel_right, vel_down

    def estimate_intercept_dt(self, current_w: float) -> float:
        """
        Heuristic time-to-intercept based on bounding box size.
        Smaller box → target is far → longer horizon.
        This shrinks naturally as we close in, tightening the prediction window.
        """
        if current_w <= 0:
            return 1.0
        size_ratio = self.TARGET_W_AT_INTERCEPT / current_w
        # map ratio [0.1 .. 10] → dt [0.1 .. 2.0] seconds
        return float(max(0.1, min(2.0, size_ratio * 0.3)))

    # ── helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _deadband(v: float, db: float) -> float:
        if abs(v) < db:
            return 0.0
        return v - math.copysign(db, v)

    @staticmethod
    def _clamp(v: float, limit: float) -> float:
        return max(-limit, min(limit, v))