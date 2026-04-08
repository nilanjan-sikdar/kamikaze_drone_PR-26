"""
comms/mavlink_iface.py — MAVLink Interface (SITL fixed)
========================================================
Fixes:
  1. type_mask = 0b0000_0111_1100_0111 (0x07C7)
     - USE:    vx, vy, vz, yaw_rate  (bits 3,4,5,10 = 0)
     - IGNORE: x, y, z, ax, ay, az, yaw (all others = 1)
  2. MAV_FRAME_LOCAL_NED instead of BODY_NED for SITL
     (BODY_NED causes phantom yaw in SITL even at 0 velocity)
  3. set_guided_mode() waits for ACK so commands don't fire before mode change
  4. send_velocity_cmd signature matches FSM: (vx, vy, vz=0, yaw_rate=0)
"""

from pymavlink import mavutil
import time
import threading
import config


class MavlinkNode:

    def __init__(self, shared_state):
        self.state = shared_state
        print(f"[MAVLink] Connecting to {config.MAVLINK_CONNECTION_STRING}...")
        self.master = mavutil.mavlink_connection(
            config.MAVLINK_CONNECTION_STRING,
            baud=config.MAVLINK_BAUD_RATE
        )
        self.master.wait_heartbeat()
        print(f"[MAVLink] Heartbeat OK — system {self.master.target_system}")

        self.telemetry_thread = threading.Thread(
            target=self._read_telemetry_loop, daemon=True
        )
        self.telemetry_thread.start()

    def _read_telemetry_loop(self):
        while self.state.is_running:
            msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=0.1)
            if msg:
                self.state.update_imu(msg.pitch, msg.roll, msg.yaw)
            time.sleep(0.01)

    def set_guided_mode(self):
        """
        Set GUIDED mode and wait for confirmation before returning.
        Without waiting, velocity commands sent immediately after are IGNORED
        because the mode change hasn't been applied yet.
        """
        print("[MAVLink] Requesting GUIDED mode...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # ArduCopter GUIDED = 4
        )
        # Wait up to 3 seconds for mode confirmation
        deadline = time.time() + 3.0
        while time.time() < deadline:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg:
                mode = msg.custom_mode
                if mode == 4:
                    print("[MAVLink] GUIDED mode confirmed.")
                    return
        print("[MAVLink] WARNING: GUIDED mode not confirmed — commands may be ignored!")

    def send_velocity_cmd(self, vx: float, vy: float, vz: float = 0.0, yaw_rate: float = 0.0):
        """
        Send velocity command via SET_POSITION_TARGET_LOCAL_NED.

        Args:
            vx       : forward  velocity (m/s)  — LOCAL_NED: North
            vy       : rightward velocity (m/s)  — LOCAL_NED: East
            vz       : downward  velocity (m/s)  — positive = descend
            yaw_rate : yaw rate (rad/s)          — positive = clockwise

        type_mask bit layout (1 = ignore, 0 = use):
            bit 0  : x pos    → 1 (ignore)
            bit 1  : y pos    → 1 (ignore)
            bit 2  : z pos    → 1 (ignore)
            bit 3  : vx       → 0 (USE)
            bit 4  : vy       → 0 (USE)
            bit 5  : vz       → 0 (USE)
            bit 6  : ax       → 1 (ignore)
            bit 7  : ay       → 1 (ignore)
            bit 8  : az       → 1 (ignore)
            bit 9  : yaw pos  → 1 (ignore)
            bit 10 : yaw_rate → 0 (USE)
            bit 11 : unused   → 0

        = 0b0000_0111_1100_0111 = 0x07C7
        """

        # Failsafe override
        if self.state.current_state == config.DroneState.HOVER_FAILSAFE:
            vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0

        TYPE_MASK = 0b0000_0111_1100_0111  # 0x07C7

        self.master.mav.set_position_target_local_ned_send(
            0,                                      # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,    # LOCAL_NED, not BODY_NED
            TYPE_MASK,
            0, 0, 0,                                # x, y, z position (ignored)
            vx, vy, vz,                             # velocities (used)
            0, 0, 0,                                # accelerations (ignored)
            0,                                      # yaw position (ignored)
            yaw_rate,                               # yaw rate (used)
        )