from pymavlink import mavutil
import time
import threading
import config

class MavlinkNode:
    def __init__(self, shared_state):
        self.state = shared_state
        print(f"[MAVLink] Connecting to ArduPilot on {config.MAVLINK_CONNECTION_STRING} at {config.MAVLINK_BAUD_RATE} baud...")
        
        # 1. Establish the Serial Connection to the Pixhawk
        self.master = mavutil.mavlink_connection(
            config.MAVLINK_CONNECTION_STRING, 
            baud=config.MAVLINK_BAUD_RATE
        )
        
        # 2. Wait for the first Heartbeat to confirm the connection is alive
        self.master.wait_heartbeat()
        print("[MAVLink] Heartbeat detected! ArduPilot is online.")
        
        # Start the background thread that constantly reads the drone's tilt
        self.telemetry_thread = threading.Thread(target=self._read_telemetry_loop, daemon=True)
        self.telemetry_thread.start()

    def _read_telemetry_loop(self):
        """
        Runs in the background. Constantly fetches ATTITUDE (Pitch/Roll) from ArduPilot
        so the MathEngine can compensate for the camera wobble.
        """
        while self.state.is_running:
            # Grab the next MAVLink message
            msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=0.1)
            if msg:
                # Save it to the thread-safe vault
                self.state.update_imu(msg.pitch, msg.roll, msg.yaw)
            
            # Request high-frequency attitude streams if needed
            time.sleep(0.01)

    def set_guided_mode(self):
        """ArduPilot MUST be in GUIDED mode to accept Xavier commands."""
        print("[MAVLink] Requesting GUIDED mode...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4 # 4 is the custom mode number for GUIDED in ArduPilot Copter
        )

    def send_velocity_cmd(self, vx, vy, vz=0.0, yaw_rate=0.0):
        """
        The ArduPilot specific velocity command. 
        X is Forward/Back, Y is Right/Left (in meters per second).
        """
        # If the failsafe tripped, override and hit the brakes
        if self.state.current_state == config.DroneState.HOVER_FAILSAFE:
            vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0

        # CORRECTED MASK: 0b011111000111 (Decimal 1991)
        # Bit 11 is now '0', which tells ArduPilot to USE the yaw_rate!
        type_mask = 0b011111000111 

        self.master.mav.set_position_target_local_ned_send(
            0, # time_boot_ms (not used)
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, # Frame of reference
            type_mask,
            0, 0, 0,    # X, Y, Z positions (Ignored by mask)
            vx, vy, vz, # X, Y, Z velocities in m/s
            0, 0, 0,    # X, Y, Z accelerations (Ignored by mask)
            0,          # Yaw position (Ignored by mask)
            yaw_rate    # Yaw rate in rad/s <-- THIS WILL NOW BE ACCEPTED
        )   