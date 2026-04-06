import math
import config

class GeometricCompensator:
    def __init__(self):
        # Pre-calculate Field of View radians to save CPU cycles during flight
        self.fov_x_rad = math.radians(config.FOV_X_DEG)
        self.fov_y_rad = math.radians(config.FOV_Y_DEG)
        
        # Calculate how many pixels the image shifts per 1 radian of drone tilt
        self.px_per_rad_x = config.FRAME_WIDTH / self.fov_x_rad
        self.px_per_rad_y = config.FRAME_HEIGHT / self.fov_y_rad

    def apply_imu_wobble_cancel(self, raw_cx, raw_cy, pitch_rad, roll_rad):
        """
        Mathematical Gimbal: Shifts the target's pixel coordinates to perfectly 
        cancel out the physical tilt of the drone body.
        """
        # If drone rolls right, camera sweeps right, target appears to move left.
        # We add the shift back to find the "true" position in the frame.
        false_shift_x = roll_rad * self.px_per_rad_x
        false_shift_y = pitch_rad * self.px_per_rad_y
        
        true_cx = raw_cx + false_shift_x
        true_cy = raw_cy + false_shift_y
        
        return true_cx, true_cy

    def estimate_metric_distance(self, err_x_px, err_y_px, bbox_width_px):
        """
        Converts 2D pixel errors into 3D metric distances (meters).
        Prevents overshooting when the target gets very close.
        """
        # Prevent division by zero if we lose the bounding box
        if bbox_width_px <= 0:
            return 0.0, 0.0, 0.0
            
        # Pinhole Camera Math: Distance (Z-axis) in meters
        distance_m = (config.FOCAL_LENGTH_PX * config.REAL_TARGET_WIDTH_M) / bbox_width_px
        
        # Convert X and Y pixel errors into real-world meters
        err_x_m = (err_x_px * distance_m) / config.FOCAL_LENGTH_PX
        err_y_m = (err_y_px * distance_m) / config.FOCAL_LENGTH_PX
        
        return err_x_m, err_y_m, distance_m