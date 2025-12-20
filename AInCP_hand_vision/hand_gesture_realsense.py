"""RealSense + MediaPipe hand gesture recognition with depth overlay.

Usage:
    & C:/realsense_project/rs_env/Scripts/python.exe c:/realsense_project/hand_gesture_realsense.py

Requirements:
    pip install mediapipe opencv-python pyrealsense2

Hardware setup:
    - Recommended cameras: Intel RealSense D435i (RGB + depth).
    - RealSense computes depth using a stereo IR pair (+ IR projector on some models).
    - Connect the camera to a USB 3.0 port for full throughput and stable depth.
    - This script reads the device `depth_scale` and multiplies raw depth to obtain
        meters; values are reported as mm/cm to the user.
    - If using other RGB-D sensors (ToF, stereo rigs like ZED), ensure you can
        provide an aligned depth frame (depth aligned to color) with per-pixel
        distances in meters before using this script.
    - Typical operating ranges vary by sensor (e.g., ~0.2m–10m). Accuracy degrades
        with distance; stereo-based sensors prefer controlled IR/ambient lighting.

Description:
    This script detects hands via MediaPipe, classifies simple gestures
    (open palm, fist, point) by counting fingers, and shows depth (mm/cm)
    for the index fingertip using the aligned depth frame.
"""

import sys
import time
try:
    import mediapipe as mp
except Exception as e:
    print("mediapipe is required. Install with: pip install mediapipe")
    raise

import pyrealsense2 as rs
import numpy as np
import cv2

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


def fingers_up(hand_landmarks, handedness_str):
    # Returns number of fingers up (thumb included) based on landmark positions
    lm = hand_landmarks.landmark
    fingers = []

    # Thumb: compare tip and IP in x depending on handedness
    if handedness_str == 'Right':
        fingers.append(lm[mp_hands.HandLandmark.THUMB_TIP].x < lm[mp_hands.HandLandmark.THUMB_IP].x)
    else:
        fingers.append(lm[mp_hands.HandLandmark.THUMB_TIP].x > lm[mp_hands.HandLandmark.THUMB_IP].x)

    # Other four fingers: tip y lower (smaller) than PIP y => finger is up
    tips = [mp_hands.HandLandmark.INDEX_FINGER_TIP,
            mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            mp_hands.HandLandmark.RING_FINGER_TIP,
            mp_hands.HandLandmark.PINKY_TIP]
    pips = [mp_hands.HandLandmark.INDEX_FINGER_PIP,
            mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            mp_hands.HandLandmark.RING_FINGER_PIP,
            mp_hands.HandLandmark.PINKY_PIP]

    for t, p in zip(tips, pips):
        fingers.append(lm[t].y < lm[p].y)

    return int(sum(fingers))


def gesture_from_count(count, index_extended):
    # Simple mapping: 0 -> Fist, 5 -> Open, 1 and index extended -> Point
    if count == 0:
        return 'Fist'
    if count == 5:
        return 'Open'
    if count == 1 and index_extended:
        return 'Point'
    return f'{count} fingers'


def run():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    align = rs.align(rs.stream.color)

    hands = mp_hands.Hands(static_image_mode=False,
                           max_num_hands=2,
                           min_detection_confidence=0.5,
                           min_tracking_confidence=0.5)

    try:
        prev_time = time.time()
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)

            h, w, _ = color_image.shape

            if results.multi_hand_landmarks:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    # draw landmarks
                    mp_drawing.draw_landmarks(color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                    hand_label = handedness.classification[0].label  # 'Left' or 'Right'
                    count = fingers_up(hand_landmarks, hand_label)

                    # index fingertip
                    ix = int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * w)
                    iy = int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * h)

                    # Depth from depth frame (meters)
                    try:
                        depth_m = depth_frame.get_distance(ix, iy)
                    except Exception:
                        depth_m = 0.0

                    # Mean depth in small neighborhood for stability
                    r = 5
                    x0 = max(0, ix - r)
                    x1 = min(w - 1, ix + r)
                    y0 = max(0, iy - r)
                    y1 = min(h - 1, iy + r)
                    local = depth_image[y0:y1+1, x0:x1+1].astype(float)
                    local = local * depth_scale
                    local_nonzero = local[local > 0]
                    if local_nonzero.size:
                        mean_local_m = float(np.mean(local_nonzero))
                    else:
                        mean_local_m = 0.0

                    mean_mm = mean_local_m * 1000.0

                    # Determine if index is extended (simple check vs PIP)
                    index_extended = (hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y <
                                      hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y)

                    gesture = gesture_from_count(count, index_extended)

                    # Draw info
                    cv2.putText(color_image, f'{hand_label} {gesture}', (ix+10, iy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    if mean_local_m > 0:
                        cv2.putText(color_image, f'{mean_mm:.0f} mm / {mean_mm/10:.1f} cm', (ix+10, iy+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
                    else:
                        cv2.putText(color_image, 'Depth: N/A', (ix+10, iy+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

                    # circle at fingertip
                    cv2.circle(color_image, (ix, iy), 6, (255,0,0), -1)

            # FPS
            now = time.time()
            fps = 1.0 / (now - prev_time) if now != prev_time else 0.0
            prev_time = now
            cv2.putText(color_image, f'FPS: {fps:.1f}', (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            combined = np.hstack((color_image, depth_colormap))

            cv2.imshow('Hand Gesture + Depth', combined)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

    finally:
        hands.close()
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    run()
