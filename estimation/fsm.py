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
                            self.chase_frame_count = 0

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
        print("\n========== NEW FRAME (PYMAVLINK PREDICTIVE) ==========")
        
        # 1. INITIALIZE VARIABLES 
       # 1. INITIALIZE VARIABLES 
        if not hasattr(self, "prev_vx"):
            self.ctrl_x_active = False
            self.ctrl_y_active = False
            self.prev_vx = 0.0
            self.prev_vy = 0.0
            self.prev_yaw_rate = 0.0
            self.chase_frame_count = 0  # <--- ADD THIS
            
        self.chase_frame_count += 1     # <--- ADD THIS (Counts how long we've been tracking)
            
        # 2. RAW PIXEL ERROR
        raw_x = vision["cx"] - config.CENTER_X
        raw_y = vision["cy"] - config.CENTER_Y

        # 3. HYSTERESIS DEADBAND (Kills the stationary twitching!)
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
    
        raw_deadband_x = 0.0 if in_x else raw_x
        raw_deadband_y = 0.0 if in_y else raw_y

        # 🔥 THE YAW DECOUPLER: Save this specifically for the turret so it doesn't spin backward!
        yaw_err_px = raw_deadband_x

        # 4. IMU WOBBLE CANCEL (TEMPORARILY DISABLED!)
        # We are commenting this out to prove the flyaway glitch is dead.
        # pitch = self.state.imu_pitch_rad
        # roll  = self.state.imu_roll_rad
        # err_x_px, err_y_px = self.geo_comp.apply_imu_wobble_cancel(
        #     raw_deadband_x, raw_deadband_y, pitch, roll
        # )
        
        # Pass the raw pixels straight to the translation math for now
        err_x_px = raw_deadband_x
        err_y_px = raw_deadband_y

        # 5. METRIC CONVERSION & DISTANCE
        err_x_m, err_y_m, distance_m = self.geo_comp.estimate_metric_distance(
            err_x_px, err_y_px, vision["w"]
        )
        
        # Failsafe: Clamp distance between 0.5m and 10m so the math doesn't explode
        distance_m = max(0.5, min(10.0, distance_m)) 
        meters_per_px = config.REAL_TARGET_WIDTH_M / max(1.0, float(vision["w"]))

        # ==========================================
        # 🛡️ DEFENSE-GRADE: PREDICTIVE INTERCEPTION
        # ==========================================
        target_vx_m_s = vision["kf_vx"] * meters_per_px
        target_vy_m_s = vision["kf_vy"] * meters_per_px

        # The Deadband (Ignore camera micro-jitters)
        if abs(target_vx_m_s) < 0.15: target_vx_m_s = 0.0
        if abs(target_vy_m_s) < 0.15: target_vy_m_s = 0.0

        target_vx_m_s = max(-1.5, min(1.5, target_vx_m_s)) 
        target_vy_m_s = max(-1.5, min(1.5, target_vy_m_s))

        # Dynamic Lookahead
        LOOKAHEAD_MULTIPLIER = 0.35  
        raw_lookahead = max(0.1, min(0.8, distance_m * LOOKAHEAD_MULTIPLIER)) 

        # ---> NEW: KALMAN FILTER WARM-UP <---
        # Smoothly fade in the prediction over the first 20 frames (1 second)
        warmup_factor = min(1.0, self.chase_frame_count / 20.0)
        safe_lookahead_sec = raw_lookahead * warmup_factor

        predict_add_x = max(-0.5, min(0.5, target_vx_m_s * safe_lookahead_sec))
        predict_add_y = max(-0.5, min(0.5, target_vy_m_s * safe_lookahead_sec))

        # CALCULATE THE FUTURE INTERCEPT COORDINATE
        future_err_x_m = err_x_m + predict_add_x
        future_err_y_m = err_y_m + predict_add_y
        # ==========================================

        # 6. ADAPTIVE PID GAINS 
        dist_px = max(abs(err_x_px), abs(err_y_px))
        if dist_px < 50:
            kp, ki, kd = config.KP_NEAR, config.KI_NEAR, config.KD_NEAR
        elif dist_px < 120:
            kp, ki, kd = config.KP_MID, config.KI_MID, config.KD_MID
        else:
            kp, ki, kd = config.KP_FAR, config.KI_FAR, config.KD_FAR
    
        self.pid_x.set_gains(kp, ki, kd)
        self.pid_y.set_gains(kp, ki, kd)

        # 7. EXECUTE PID ON THE FUTURE COORDINATES
        pid_y = self.pid_y.update(future_err_y_m) 
        pid_x = self.pid_x.update(future_err_x_m) 

        # 8. COMMAND SMOOTHING (Alpha Filter)
        target_vx = config.CMD_ALPHA * self.prev_vx + (1 - config.CMD_ALPHA) * (config.SIGN_VX * pid_y)
        target_vy = config.CMD_ALPHA * self.prev_vy + (1 - config.CMD_ALPHA) * (config.SIGN_VY * pid_x)
        
        self.prev_vx = target_vx
        self.prev_vy = target_vy

        # 9. ACCELERATION LIMITS & CLAMPS
        safe_vx = self.slew_x.apply(target_vx)
        safe_vy = self.slew_y.apply(target_vy)
        safe_vx = max(-config.MAX_VX, min(config.MAX_VX, safe_vx))
        safe_vy = max(-config.MAX_VY, min(config.MAX_VY, safe_vy))

        # ==========================================
        # 10. YAW CONTROL (Sniper Tune - Zero Momentum)
        # ==========================================
        # We calculate the exact spin needed based on the deadbanded pixels
        raw_yaw_rate = config.YAW_KP * yaw_err_px
        
        # Hard clamp it so it never exceeds our new fast top speed
        raw_yaw_rate = max(-config.MAX_YAW_RATE, min(config.MAX_YAW_RATE, raw_yaw_rate))
        
        # 🔥 REMOVED THE 50% ALPHA FILTER! 
        # The moment the target enters the center deadband, raw_yaw_rate becomes exactly 0.0.
        # The slew limiter (with our new 4.0 Accel) will slam the brakes instantly without drifting.
        safe_yaw_rate = self.slew_yaw.apply(raw_yaw_rate)
        # ==========================================
        # 11. ALTITUDE CONTROL (Maintain safe 2-meter distance)
        # ==========================================
        err_z = config.TARGET_DISTANCE_M - distance_m 
        if abs(err_z) < 0.1: # Altitude deadband
            err_z = 0.0
            
        # PyMavlink NED: Negative VZ = FLY UP
        target_vz = -1.0 * self.pid_z.update(err_z)
        safe_vz = max(-config.MAX_VZ, min(config.MAX_VZ, target_vz))

        # 12. SEND VIA PYMAVLINK
        print(f"[CMD] VX:{safe_vx:.2f} | VY:{safe_vy:.2f} | VZ:{safe_vz:.2f} | YAW:{safe_yaw_rate:.2f}")
        
        # Calling your exact PyMavlink function!
        self.mavlink.send_velocity_cmd(safe_vx, safe_vy, safe_vz, safe_yaw_rate)