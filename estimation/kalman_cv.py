"""
estimation/kalman_cv.py — Constant-Velocity Kalman Filter (MRS-inspired)
=========================================================================

Inspired by the MRS UAV System's approach to target state estimation.

Key upgrades over the original KalmanFilter8D:
- Clean separation of state transition (F) and observation (H) matrices
- Proper process noise (Q) tuning for a manoeuvring target
- Mahalanobis-distance gating to reject spurious detections
- get_predicted_state(dt) → lets the FSM ask "where will target be in Δt seconds?"
  This is the core of predictive intercept logic.

State vector x = [cx, cy, w, h, vx, vy, vw, vh]  (8D)
  cx, cy  = bbox centre in pixels
  w,  h   = bbox width/height
  vx, vy  = pixel velocity of centre
  vw, vh  = rate of change of size (proxy for closing speed)
"""

import numpy as np


class KalmanFilterCV:
    """
    8-state constant-velocity Kalman filter for 2-D bounding box tracking.

    Process model  : x_{k+1} = F * x_k  +  w_k,   w_k ~ N(0, Q)
    Observation    : z_k     = H * x_k  +  v_k,   v_k ~ N(0, R)
    """

    # ── tuneable noise parameters ──────────────────────────────────────────
    # Q_pos_std  : how much the target's *velocity* can change per second
    #              (high → trust measurements more; low → smoother but laggy)
    Q_VEL_STD = 80.0   # pixels/s²  — allows fast-manoeuvring targets
    Q_SIZE_STD = 20.0  # px/s²

    # R_std : measurement noise (pixel std of your HSV detector)
    R_POS_STD  = 8.0   # px
    R_SIZE_STD = 6.0   # px

    # Mahalanobis gate: reject updates farther than this (chi² dof=4, 99.9%)
    MAHA_GATE = 18.47

    def __init__(self, dt: float = 0.033):
        self.dt = dt
        self._build_matrices(dt)
        self.x  = np.zeros((8, 1))   # state
        self.P  = np.eye(8) * 500.0  # covariance — large initial uncertainty
        self.initialised = False

    # ── matrix construction ────────────────────────────────────────────────

    def _build_matrices(self, dt: float):
        dt = max(dt, 1e-4)
        # State transition: position += velocity * dt
        self.F = np.eye(8)
        for i in range(4):
            self.F[i, i + 4] = dt

        # Observation: we only see [cx, cy, w, h]
        self.H = np.zeros((4, 8))
        self.H[:4, :4] = np.eye(4)

        # Process noise — discrete white-noise acceleration model
        q_pos  = self.Q_VEL_STD  ** 2 * dt
        q_size = self.Q_SIZE_STD ** 2 * dt
        self.Q = np.diag([
            q_pos  * dt**2, q_pos  * dt**2,
            q_size * dt**2, q_size * dt**2,
            q_pos,          q_pos,
            q_size,         q_size,
        ])

        # Measurement noise
        self.R = np.diag([
            self.R_POS_STD ** 2,  self.R_POS_STD ** 2,
            self.R_SIZE_STD ** 2, self.R_SIZE_STD ** 2,
        ])

    # ── public API ─────────────────────────────────────────────────────────

    def initiate(self, cx: float, cy: float, w: float, h: float):
        """Cold-start the filter from a fresh detection."""
        self.x = np.array([[cx], [cy], [w], [h],
                           [0.0], [0.0], [0.0], [0.0]])
        self.P = np.eye(8) * 500.0
        self.initialised = True

    def predict(self, dt: float = None):
        """Time-update step.  Call once per control cycle."""
        if dt is not None and abs(dt - self.dt) > 1e-4:
            self.dt = dt
            self._build_matrices(dt)

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, cx: float, cy: float, w: float, h: float) -> bool:
        """
        Measurement-update step with Mahalanobis gating.
        Returns True if the measurement was accepted, False if gated out.
        """
        z = np.array([[cx], [cy], [w], [h]])
        y = z - self.H @ self.x                   # innovation
        S = self.H @ self.P @ self.H.T + self.R   # innovation covariance

        # Mahalanobis gate
        maha2 = float(y.T @ np.linalg.inv(S) @ y)
        if self.initialised and maha2 > self.MAHA_GATE:
            return False   # outlier — skip update

        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(8) - K @ self.H) @ self.P
        self.initialised = True
        return True

    def get_state(self):
        """Return (cx, cy, w, h, vx, vy) — current best estimate."""
        cx, cy, w, h, vx, vy = (float(self.x[i, 0]) for i in range(6))
        return cx, cy, w, h, vx, vy

    def get_predicted_state(self, lookahead_dt: float):
        """
        ★ MRS-style predictive query ★

        Returns the *predicted* (cx, cy, w, h, vx, vy) after `lookahead_dt`
        seconds WITHOUT modifying internal state.

        The FSM uses this to aim at where the target WILL BE, not where it IS.
        """
        F_ahead = np.eye(8)
        for i in range(4):
            F_ahead[i, i + 4] = lookahead_dt
        x_ahead = F_ahead @ self.x
        cx, cy, w, h, vx, vy = (float(x_ahead[i, 0]) for i in range(6))
        return cx, cy, w, h, vx, vy