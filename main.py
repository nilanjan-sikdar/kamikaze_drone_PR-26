"""
main.py — Entry Point
======================
FIX: camera_thread_loop now calls state.update_vision_lost() when
     no target is found. Previously it never reset target_found to False,
     so the drone would never stop chasing even when target was gone.
"""

import cv2
import time
import threading
import sys
import numpy as np
import socket

import config
from comms.shared_state import SharedState
from comms.mavlink_iface import MavlinkNode
from estimation.fsm import FlightStateMachine
from detection.hsv_detector import HSVDetector
from detection.filters import KalmanFilter8D
from display.hud import DroneHUD


def camera_thread_loop(state):

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", config.UDP_PORT))
    sock.settimeout(0.5)

    detector     = HSVDetector()
    tracker      = KalmanFilter8D(dt=config.CONTROL_DT)
    hud          = DroneHUD()
    target_locked = False

    while state.is_running:
        try:
            data, _ = sock.recvfrom(65507)
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # Run detection
            cx, cy, w, h, mask = detector.process_frame(frame)
            if cx is not None and cy is not None and w > 0:
                print("[VISION] TARGET FOUND")
            else:
                print("[VISION] TARGET LOST")
            if cx is not None and cy is not None and w > 0:
                # ── Target FOUND ──────────────────────────────────────────────
                
                if not target_locked:
                    tracker.initiate(cx, cy, w, h)
                    target_locked = True
                else:
                    tracker.predict()
                    tracker.update(cx, cy, w, h)

                scx, scy, sw, sh, kvx, kvy = tracker.get_state()

                # Write real detection to shared state
                state.update_vision(scx, scy, sw, sh, kvx, kvy)
                print("[STATE] Vision updated")

            else:
                # ── Target NOT FOUND ──────────────────────────────────────────
                # BUG FIX: tell shared state target is gone
                # Previously: called update_vision with Kalman prediction
                #             → target_found stayed True forever → drone never stopped
                state.update_vision_lost()
                print("[STATE] Vision LOST sent to FSM")

                # Still run Kalman prediction to maintain tracker state
                # (so it doesn't cold-start when target reappears)
                if target_locked:
                    tracker.predict()
                    # NOTE: do NOT write predicted position to shared state
                    # The FSM should be braking, not chasing a ghost

            # Draw HUD and display
            state_data    = state.get_vision_data()
            annotated     = hud.draw(frame, state_data)
            cv2.imshow("Drone Tracker", annotated)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                state.is_running = False

        except socket.timeout:
            # No frame — ensure state reflects lost target
            state.update_vision_lost()
            continue
        except Exception as e:
            print(f"[Vision Error] {e}")
            continue

    sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print("=" * 45)
    print("      DRONE TRACKER — BOOTING")
    print("=" * 45)

    shared_state  = SharedState()
    mavlink_node  = MavlinkNode(shared_state)
    fsm           = FlightStateMachine(shared_state, mavlink_node)

    vision_thread = threading.Thread(
        target=camera_thread_loop,
        args=(shared_state,),
        daemon=True
    )
    vision_thread.start()

    try:
        fsm.run_loop()
    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutting down...")
        shared_state.is_running = False

    vision_thread.join(timeout=2.0)
    print("=== OFFLINE ===")