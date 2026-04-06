from enum import Enum

# ==========================================
# 1. HARDWARE & MAVLINK SETTINGS
# ==========================================
MAVLINK_CONNECTION_STRING = "udpin:127.0.0.1:14550"
MAVLINK_BAUD_RATE = 921600
UDP_PORT        = 5599
CONTROL_LOOP_HZ = 20
CONTROL_DT      = 1.0 / CONTROL_LOOP_HZ

# ==========================================
# 2. CAMERA & VISION SETTINGS
# ==========================================
FRAME_WIDTH  = 400
FRAME_HEIGHT = 240
CENTER_X     = FRAME_WIDTH // 2
CENTER_Y     = FRAME_HEIGHT // 2

# Webots Tracker Drone Intrinsics
FOCAL_LENGTH_PX = 482.84  
REAL_TARGET_WIDTH_M = 0.20 

FOV_X_DEG = 62.2
FOV_Y_DEG = 48.8

# ==========================================
# 3. HSV DETECTION (Direct from v5)
# ==========================================
HSV_LOWER1 = (0,   50, 40)
HSV_UPPER1 = (15,  255, 255)
HSV_LOWER2 = (165, 50, 40)
HSV_UPPER2 = (180, 255, 255)
MIN_BLOB_RADIUS = 5.0
TARGET_RADIUS   = 7.0

# ==========================================
# 4. LOCK-ON STATE MACHINE (Direct from v5)
# ==========================================
class DroneState(Enum):
    BOOTING        = 0
    SEARCHING      = 1
    ACQUIRING      = 2   # ✅ MUST EXIST
    CHASING        = 3
    HOVER_FAILSAFE = 4

CONFIRM_FRAMES   = 4
REACQUIRE_FRAMES = 12
TARGET_TIMEOUT_SEC = 1.5  # Grace period so it doesn't drop instantly

# ==========================================
# 5. HYSTERESIS ZONES (Direct from v5)
# ==========================================
HOVER_X  = 20
HOVER_Y  = 20
ENGAGE_X = 40
ENGAGE_Y = 40

# ==========================================
# 6. VELOCITY & SLEW LIMITS
# ==========================================
MAX_VX    = 0.40  # From v5
MAX_VY    = 0.40  # From v5
MAX_ACCEL = 0.8   # Slew rate limit to prevent pitch vibration
CMD_ALPHA = 0.30  # Command smoothing
MOMENTUM_DECAY = 0.70

# ── SIGN CHECK (Direct from v5) ──
SIGN_VX = -1
SIGN_VY = 1

# ==========================================
# 7. ADAPTIVE METRIC PID GAINS
# ==========================================
# Translated your v5 pixel zones into metric physical distances.
ZONE_NEAR_MAX_M = 0.5   
ZONE_MID_MAX_M  = 2.0   

# NEW (start here)
KP_NEAR = 0.8
KI_NEAR = 0.0
KD_NEAR = 0.1

KP_MID  = 1.2
KI_MID  = 0.0
KD_MID  = 0.15

KP_FAR  = 1.8
KI_FAR  = 0.0
KD_FAR  = 0.2

# In your config.py, update the Yaw settings to this:
YAW_KP = 0.005          # Lower this from 0.01 so it doesn't overshoot
MAX_YAW_RATE = 0.8       # rad/s
YAW_MAX_ACCEL = 1.5      # rad/s^2 (How fast it's allowed to speed up/slow down its spin)

# ==========================================
# 8. FEED-FORWARD (Direct from v5)
# ==========================================
FF_GAIN      = 0.35
FF_VEL_ALPHA = 0.30
FF_MAX_PX    = 80.0

# ==========================================
# 9. ALTITUDE / SAFE DISTANCE CONTROL
# ==========================================
TARGET_DISTANCE_M = 2.0  # Maintain exactly 2 meters above target
MAX_VZ = 0.5             # Max climb/drop speed (m/s)

# Z-axis (Altitude) PID Gains
KP_Z = 0.8
KI_Z = 0.0
KD_Z = 0.1