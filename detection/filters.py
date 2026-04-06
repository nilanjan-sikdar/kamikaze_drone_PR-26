import numpy as np
from typing import Optional, Tuple

class LowPassFilter:
    """Basic Exponential Moving Average (EMA) to stop jitter."""
    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self.prev_value: Optional[float] = None

    def update(self, current_value: float) -> float:
        if self.prev_value is None:
            self.prev_value = current_value
            return current_value
        filtered = (self.alpha * current_value) + ((1.0 - self.alpha) * self.prev_value)
        self.prev_value = filtered
        return filtered

    def reset(self):
        self.prev_value = None


class KalmanFilter8D:
    """
    The ByteTrack Brain. Tracks [cx, cy, aspect_ratio, height] 
    and predicts their velocities [vx, vy, va, vh].
    """
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        
        # Explicitly typing as np.ndarray kills the "tuple[slice...]" errors
        self.state: np.ndarray = np.zeros(8)
        
        # Covariance (Uncertainty)
        self.covariance: np.ndarray = np.eye(8) * 10.0
        self.covariance[4:, 4:] *= 1000.0  
        
        # State Transition Matrix (F) - Physics Model
        self.F: np.ndarray = np.eye(8)
        for i in range(4):
            self.F[i, i + 4] = self.dt
            
        # Measurement Matrix (H) - Mapping 8D state to 4D observation
        # This tells the filter we only 'see' the first 4 variables
        self.H: np.ndarray = np.zeros((4, 8))
        for i in range(4):
            self.H[i, i] = 1.0
        
        # Noise tuning
        self.process_noise: np.ndarray = np.eye(8) * 0.01
        self.measurement_noise: np.ndarray = np.eye(4) * 0.1

    def initiate(self, cx: float, cy: float, w: float, h: float):
        """Initialize state when target is first detected."""
        aspect_ratio = w / float(h) if h != 0 else 1.0
        self.state[:4] = [cx, cy, aspect_ratio, h]
        self.state[4:] = 0.0 
        self.covariance = np.eye(8) * 10.0

    def predict(self):
        """Predict the future state: X = Fx, P = FPF' + Q"""
        self.state = np.dot(self.F, self.state)
        self.covariance = np.linalg.multi_dot([self.F, self.covariance, self.F.T]) + self.process_noise

    def update(self, cx: float, cy: float, w: float, h: float):
        """Correct the prediction with actual data: K = PH'(HPH' + R)^-1"""
        aspect_ratio = w / float(h) if h != 0 else 1.0
        measurement = np.array([cx, cy, aspect_ratio, h])
        
        # Innovation (Error)
        y = measurement - np.dot(self.H, self.state)
        
        # Innovation Covariance
        S = np.linalg.multi_dot([self.H, self.covariance, self.H.T]) + self.measurement_noise
        
        # Kalman Gain
        K = np.linalg.multi_dot([self.covariance, self.H.T, np.linalg.inv(S)])
        
        # Update State and Covariance
        self.state = self.state + np.dot(K, y)
        self.covariance = self.covariance - np.linalg.multi_dot([K, self.H, self.covariance])

    def get_state(self) -> Tuple[float, float, float, float, float, float]:
        """Returns (cx, cy, w, h, vx, vy)"""
        cx, cy, a, h = self.state[:4]
        vx, vy, va, vh = self.state[4:]
        w = a * h
        return float(cx), float(cy), float(w), float(h), float(vx), float(vy)