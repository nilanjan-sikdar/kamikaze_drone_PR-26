import cv2
import config

class DroneHUD:
    def __init__(self):
        self.color_target = (0, 255, 0)      # Green for locked target
        self.color_info = (255, 255, 255)    # White for text
        self.color_center = (0, 0, 255)      # Red for frame center

    def draw(self, frame, state_data):
        """
        Draws the targeting reticle, Kalman predictions, and flight telemetry.
        """
        # 1. Draw Frame Center (The "Crosshair" we want to align with)
        cv2.circle(frame, (int(config.CENTER_X), int(config.CENTER_Y)), 5, self.color_center, -1)

        if state_data["found"]:
            cx, cy = int(state_data["cx"]), int(state_data["cy"])
            w, h = int(state_data["w"]), int(state_data["h"])

            # 2. Draw Bounding Box (The smoothed Kalman output) [cite: 36]
            cv2.rectangle(frame, (cx - w//2, cy - h//2), (cx + w//2, cy + h//2), self.color_target, 2)
            
            # 3. Draw Predictive Vector (Where the 8D Kalman thinks it's going) [cite: 13, 14]
            # We multiply velocity by a factor so the line is visible
            vx, vy = state_data["kf_vx"], state_data["kf_vy"]
            end_point = (int(cx + vx * 0.2), int(cy + vy * 0.2))
            cv2.line(frame, (cx, cy), end_point, (255, 0, 0), 2) # Blue velocity line
            
            # 4. Display Metric Distance [cite: 66, 102]
            # We calculate this in the FSM, but we can show the estimate here
            dist_text = f"Dist: {((config.FOCAL_LENGTH_PX * config.REAL_TARGET_WIDTH_M) / w):.2f}m"
            cv2.putText(frame, dist_text, (cx - w//2, cy - h//2 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_info, 1)

        # 5. System Status Overlay
        # Pure Python f-string. No brackets at the end.
        status_text = f"FPS: {config.CONTROL_LOOP_HZ} | Accel_Limit: {config.MAX_ACCEL}m/s2"
        cv2.putText(frame, status_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.color_info, 2)
        
        return frame