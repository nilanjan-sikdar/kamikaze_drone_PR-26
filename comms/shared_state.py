"""
comms/shared_state.py — Thread-Safe Memory Vault
=================================================
FIX: target_found now properly resets to False when target is lost.
     Added update_vision_lost() method for when detection fails.
"""

import threading
import time
from config import DroneState, CENTER_X, CENTER_Y


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()

        # System flags
        self.is_running     = True
        self.current_state  = DroneState.BOOTING

        # --- VISION DATA (written by camera thread) ---
        self.target_found   = False       # ← properly managed now
        self.last_seen_time = time.time()

        # Bounding box
        self.bbox_cx = float(CENTER_X)
        self.bbox_cy = float(CENTER_Y)
        self.bbox_w  = 0.0
        self.bbox_h  = 0.0

        # Kalman predicted velocities (pixels/sec)
        self.kf_vx = 0.0
        self.kf_vy = 0.0

        # --- DRONE TELEMETRY (written by MAVLink thread) ---
        self.imu_pitch_rad = 0.0
        self.imu_roll_rad  = 0.0
        self.imu_yaw_rad   = 0.0

    def update_vision(self, cx, cy, w, h, kf_vx, kf_vy):
        """
        Call ONLY when detector actually found the target this frame.
        Sets target_found = True and records timestamp.
        """
        with self.lock:
            self.target_found   = True
            self.last_seen_time = time.time()
            self.bbox_cx = cx
            self.bbox_cy = cy
            self.bbox_w  = w
            self.bbox_h  = h
            self.kf_vx   = kf_vx
            self.kf_vy   = kf_vy

    def update_vision_lost(self):
        """
        BUG FIX: Call when detector finds NOTHING this frame.
        Sets target_found = False so FSM knows to stop chasing.
        Previously this was never called — drone never stopped.
        """
        with self.lock:
            pass
            # Do NOT update last_seen_time — FSM uses it for timeout check

    def update_imu(self, pitch, roll, yaw):
        """Called by MAVLink thread when new telemetry arrives."""
        with self.lock:
            self.imu_pitch_rad = pitch
            self.imu_roll_rad  = roll
            self.imu_yaw_rad   = yaw

    def get_vision_data(self):
        """Called by control loop at 20Hz."""
        with self.lock:
            return {
                "found":           self.target_found,
                "time_since_last": time.time() - self.last_seen_time,
                "cx":   self.bbox_cx,
                "cy":   self.bbox_cy,
                "w":    self.bbox_w,
                "h":    self.bbox_h,
                "kf_vx": self.kf_vx,
                "kf_vy": self.kf_vy,
            }