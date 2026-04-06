# from ultralytics import YOLO
# import time
# import config

# class YoloDetector:
#     def __init__(self, model_path="yolov8n.engine"): # Use .pt if testing on laptop
#         print(f"[YOLO] Loading Ultralytics model: {model_path}")
#         self.model = YOLO(model_path)
        
#         # Memory to calculate our own velocity since Ultralytics hides it
#         self.last_cx = 0.0
#         self.last_cy = 0.0
#         self.last_time = time.time()

#     def process_frame(self, frame):
#         """
#         Runs native Ultralytics ByteTrack.
#         """
#         current_time = time.time()
#         dt = current_time - self.last_time
#         if dt <= 0: dt = 0.01

#         # NATIVE ULTRALYTICS BYTETRACK!
#         # persist=True tells it to remember IDs between frames
#         results = self.model.track(frame, tracker="bytetrack.yaml", persist=True, verbose=False)
        
#         # Grab the annotated frame for the HUD
#         annotated_frame = results[0].plot()
        
#         # If nothing is detected, return None
#         if results[0].boxes.id is None:
#             return None, None, 0.0, 0.0, 0.0, 0.0, annotated_frame
            
#         # We assume we are tracking the FIRST object in the list for now
#         # (Later, you can filter this by a specific results[0].boxes.id)
#         box = results[0].boxes[0]
        
#         # Get coordinates [x_center, y_center, width, height]
#         cx, cy, w, h = box.xywh[0].cpu().numpy()
        
#         # Calculate pixel velocity since Ultralytics won't give us the Kalman vx/vy
#         kf_vx = (cx - self.last_cx) / dt
#         kf_vy = (cy - self.last_cy) / dt
        
#         # Update memory for next frame
#         self.last_cx = cx
#         self.last_cy = cy
#         self.last_time = current_time
        
#         return cx, cy, w, h, kf_vx, kf_vy, annotated_frame