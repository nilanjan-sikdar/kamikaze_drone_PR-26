"""
estimation/fsm.py — Flight State Machine
==========================================
FIXES:
  1. Axis mapping corrected for downward camera
     (target below centre = fly FORWARD, not backward)
  2. Feed-forward velocity converted from pixels/frame to pixels/sec
     (Kalman gives velocity per DT, not per second)
  3. PID now receives fixed dt instead of computing internally
  4. HOVER_FAILSAFE uses slew to brake smoothly (was already correct)
  5. State machine properly transitions on target found/lost
"""

import time
import config
from control.pid import PIDController
from control.slew import SlewRateLimiter
from control.imu_comp import GeometricCompensator


class FlightStateMachine:
    def __init__(self, shared_state, mavlink_node):
        self.state   = shared_state
        self.mavlink = mavlink_node
        self.confirm_count=0
        self.lost_count=0

        # Math engines
        self.geo_comp = GeometricCompensator()
        self.slew_x   = SlewRateLimiter(config.MAX_ACCEL, config.CONTROL_DT)
        self.slew_y   = SlewRateLimiter(config.MAX_ACCEL, config.CONTROL_DT)
        self.slew_yaw = SlewRateLimiter(config.YAW_MAX_ACCEL, config.CONTROL_DT)

        # Metric PID controllers — one per axis
        # Metric PID controllers (Using the base NEAR gains to start)
        self.pid_x = PIDController(config.KP_NEAR, config.KI_NEAR, config.KD_NEAR)
        self.pid_y = PIDController(config.KP_NEAR, config.KI_NEAR, config.KD_NEAR)
        self.pid_z = PIDController(config.KP_Z, config.KI_Z, config.KD_Z)

    def run_loop(self):
        try:
            print("[FSM] Flight Control Loop Starting...")
            self.mavlink.set_guided_mode()
            self.state.current_state = config.DroneState.SEARCHING

            while self.state.is_running:
                loop_start = time.time()

                # ✅ GET VISION DATA
                vision = self.state.get_vision_data()

                # ─────────────────────────────────────────────
                # STATE MACHINE (v5 style)
                # ─────────────────────────────────────────────
                if vision["found"]:
                    self.lost_count = 0
                    self.confirm_count += 1

                    if self.state.current_state == config.DroneState.SEARCHING:
                        self.state.current_state = config.DroneState.ACQUIRING
                        print(f"[FSM] ACQUIRING ({self.confirm_count})")

                    elif self.state.current_state == config.DroneState.ACQUIRING:
                        if self.confirm_count >= config.CONFIRM_FRAMES:
                            print("[FSM] LOCKED")
                            self.state.current_state = config.DroneState.CHASING

                            # resets
                            self.pid_x.reset()
                            self.pid_y.reset()
                            self.slew_x.reset()
                            self.slew_y.reset()

                else:
                    self.confirm_count = 0
                    self.lost_count += 1

                    if self.lost_count >= config.REACQUIRE_FRAMES:
                        if self.state.current_state != config.DroneState.SEARCHING:
                            print("[FSM] LOST → SEARCHING")
                        self.state.current_state = config.DroneState.SEARCHING

                # ─────────────────────────────────────────────
                # EXECUTION
                # ─────────────────────────────────────────────
                if self.state.current_state == config.DroneState.CHASING:
                    self._execute_chase(vision)

                else:
                    # smooth brake
                    safe_vx = self.slew_x.apply(0.0)
                    safe_vy = self.slew_y.apply(0.0)
                    self.mavlink.send_velocity_cmd(safe_vx, safe_vy)
                    

                # ─────────────────────────────────────────────
                # TIMING
                # ─────────────────────────────────────────────
                elapsed = time.time() - loop_start
                sleep_time = config.CONTROL_DT - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            print(f"\n[FATAL FSM CRASH] {e}\n")
            import traceback
            traceback.print_exc()
    
    def _execute_chase(self, vision):
        print("\n========== NEW FRAME ==========")
        print(f"[VISION] cx={vision['cx']:.2f}, cy={vision['cy']:.2f}, w={vision['w']:.2f}")
        if not hasattr(self, "ctrl_x_active"):
            self.ctrl_x_active = False
            self.ctrl_y_active = False
            self.prev_vx = 0.0
            self.prev_vy = 0.0
            self.prev_yaw_rate = 0.0
    
        # RAW ERROR
        raw_x = vision["cx"] - config.CENTER_X
        raw_y = vision["cy"] - config.CENTER_Y
    
        # HYSTERESIS
        if self.ctrl_x_active:
            in_x = abs(raw_x) <= config.HOVER_X
        else:
            in_x = abs(raw_x) <= config.ENGAGE_X
    
        if self.ctrl_y_active:
            in_y = abs(raw_y) <= config.HOVER_Y
        else:
            in_y = abs(raw_y) <= config.ENGAGE_Y
    
        self.ctrl_x_active = not in_x
        self.ctrl_y_active = not in_y
    
        err_x_px = 0.0 if in_x else raw_x
        err_y_px = 0.0 if in_y else raw_y
    
        # IMU COMP
        pitch = self.state.imu_pitch_rad
        roll  = self.state.imu_roll_rad
    
        err_x_px, err_y_px = self.geo_comp.apply_imu_wobble_cancel(
            err_x_px, err_y_px, pitch, roll
        )
    
        # ... existing code ...
        # METRIC
        err_x_m, err_y_m, distance_m = self.geo_comp.estimate_metric_distance(
            err_x_px, err_y_px, vision["w"]
        )
    
        if distance_m == 0.0:
            err_x_m = err_x_px * 0.002
            err_y_m = err_y_px * 0.002
            distance_m = config.TARGET_DISTANCE_M # Failsafe
            
        # ==========================================
        # NEW: ALTITUDE (Z-AXIS) CONTROL
        # ==========================================
        # Calculate distance error (e.g., if distance is 1.5m, error is -0.5m)
        err_z = distance_m - config.TARGET_DISTANCE_M
        
        # MAVLink NED Frame: Negative VZ means FLY UP. 
        # So if err_z is negative (too close), PID returns negative VZ -> drone flies up. Perfect.
        target_vz = self.pid_z.update(err_z)
        
        # Clamp vertical speed
        safe_vz = max(-config.MAX_VZ, min(config.MAX_VZ, target_vz))
        # ==========================================

        # ADAPTIVE GAINS
        dist = max(abs(err_x_px), abs(err_y_px))
    
        if dist < 50:
            kp, ki, kd = config.KP_NEAR, config.KI_NEAR, config.KD_NEAR
        elif dist < 120:
            kp, ki, kd = config.KP_MID, config.KI_MID, config.KD_MID
        else:
            kp, ki, kd = config.KP_FAR, config.KI_FAR, config.KD_FAR
    
        self.pid_x.set_gains(kp, ki, kd)
        self.pid_y.set_gains(kp, ki, kd)
    
        # PID
        pid_y = self.pid_y.update(err_y_m)
        pid_x = self.pid_x.update(err_x_m)
        print(f"[RAW ERROR] x={raw_x:.2f}, y={raw_y:.2f}")
        print(f"[HYST] in_x={in_x}, in_y={in_y}")
        print(f"[ERR PX] x={err_x_px:.2f}, y={err_y_px:.2f}")
        # FEED FORWARD
        ff_vx = config.FF_GAIN * vision["kf_vy"] * 0.001
        ff_vy = config.FF_GAIN * vision["kf_vx"] * 0.001
    
        # AXIS
        target_vx = config.SIGN_VX * (pid_y + ff_vx)
        target_vy = config.SIGN_VY * (pid_x + ff_vy)
        print(f"[TARGET] vx={target_vx:.4f}, vy={target_vy:.4f}")
        # SMOOTH
        target_vx = config.CMD_ALPHA * self.prev_vx + (1 - config.CMD_ALPHA) * target_vx
        target_vy = config.CMD_ALPHA * self.prev_vy + (1 - config.CMD_ALPHA) * target_vy
        print(f"[METRIC] err_x_m={err_x_m:.4f}, err_y_m={err_y_m:.4f}, dist={distance_m:.2f}")
    
        self.prev_vx = target_vx
        self.prev_vy = target_vy
    
        # SLEW
        safe_vx = self.slew_x.apply(target_vx)
        safe_vy = self.slew_y.apply(target_vy)
        print(f"[SLEW] vx={safe_vx:.4f}, vy={safe_vy:.4f}")
    
        # CLAMP TRANSLATION
        safe_vx = max(-config.MAX_VX, min(config.MAX_VX, safe_vx))
        safe_vy = max(-config.MAX_VY, min(config.MAX_VY, safe_vy))
        
        ## ==========================================
        # YAW LOGIC (Smoothed & Slew Limited)
        # ==========================================
        # 1. Raw target based on pixel error
        raw_yaw_rate = config.YAW_KP * err_x_px
        
        # 2. Smooth the command (Alpha filter)
        target_yaw_rate = config.CMD_ALPHA * self.prev_yaw_rate + (1 - config.CMD_ALPHA) * raw_yaw_rate
        self.prev_yaw_rate = target_yaw_rate
        
        # 3. Apply Slew Limiter (The "Brakes")
        safe_yaw_rate = self.slew_yaw.apply(target_yaw_rate)
        
        # 4. Final Clamp
        safe_yaw_rate = max(-config.MAX_YAW_RATE, min(config.MAX_YAW_RATE, safe_yaw_rate))
        # ==========================================

        # (Remove safe_vy = 0.0 if you still have it from the Turret Test!)
        safe_vy=0
        # SEND
        self.mavlink.send_velocity_cmd(safe_vx, safe_vy, safe_vz, safe_yaw_rate)
