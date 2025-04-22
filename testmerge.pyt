import cv2
from djitellopy import Tello
import time
import pygame
import os
import numpy as np

# === Init ===
os.environ["SDL_VIDEODRIVER"] = "dummy"
cv2.setUseOptimized(True)

tello = Tello()
tello.connect()
print(f"Battery: {tello.get_battery()}%")
tello.streamon()
frame_read = tello.get_frame_read()
time.sleep(2)

# === HSV Trackbars ===
def nothing(x): pass

cv2.namedWindow("Trackbars")
for i, label in enumerate(["H Min", "S Min", "V Min", "H Max", "S Max", "V Max"]):
    default = [160, 100, 100, 175, 255, 255][i]
    max_val = 179 if "H" in label else 255
    cv2.createTrackbar(label, "Trackbars", default, max_val, nothing)

# === Pygame Setup ===
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Tello Control")

# === Helpers ===
def estimate_distance_cm(radius):
    return float('inf') if radius == 0 else 800 / radius

# === Color Sequence & LED ===
detection_sequence = ["Purple", "Orange", "Yellow", "Orange", "Blue", "Red", "Purple", "Green", "Red", "Yellow", "Blue", "Green"]
color_ranges = {
    "Purple": ((125, 50, 50), (150, 255, 255)),
    "Orange": ((10, 100, 100), (25, 255, 255)),
    "Yellow": ((25, 100, 100), (35, 255, 255)),
    "Red": ((0, 100, 100), (10, 255, 255)),
    "Blue": ((100, 100, 100), (130, 255, 255)),
    "Green": ((40, 50, 50), (80, 255, 255))
}
color_to_led = {
    "Purple": (127, 0, 255),
    "Orange": (255, 165, 0),
    "Yellow": (255, 255, 0),
    "Red": (255, 0, 0),
    "Blue": (0, 0, 255),
    "Green": (0, 255, 0)
}

current_target_index = 0
auto_mode = True

# === Takeoff Attempt ===
try:
    tello.takeoff()
except Exception as e:
    print(f"Takeoff failed: {e}")
    time.sleep(2)
    tello.takeoff()

# === Main Loop ===
try:
    while True:
        frame = frame_read.frame
        if frame is None:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # === Manual HSV Trackbars ===
        lower = np.array([cv2.getTrackbarPos("H Min", "Trackbars"),
                          cv2.getTrackbarPos("S Min", "Trackbars"),
                          cv2.getTrackbarPos("V Min", "Trackbars")])
        upper = np.array([cv2.getTrackbarPos("H Max", "Trackbars"),
                          cv2.getTrackbarPos("S Max", "Trackbars"),
                          cv2.getTrackbarPos("V Max", "Trackbars")])
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center, radius = None, 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest)
            center = (int(x), int(y))
            radius = int(radius)

        # === Control Vars ===
        lr, fb, ud, yaw = 0, 0, 0, 0

        # === Auto Tracking ===
        if auto_mode and center:
            dx = center[0] - frame.shape[1] // 2
            dy = center[1] - frame.shape[0] // 2
            ideal_radius = 45
            distance_cm = estimate_distance_cm(radius)

            yaw = int(np.clip(0.45 * dx, -30, 30))
            ud = int(np.clip(-0.45 * dy, -30, 30))
            fb = int(np.clip((radius - ideal_radius), -30, 30)) if distance_cm > 5 else -30

            print(f"[AUTO] dx={dx}, dy={dy}, radius={radius}, dist={int(distance_cm)} => YAW: {yaw}, UD: {ud}, FB: {fb}")

        # === Manual Override ===
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        keys = pygame.key.get_pressed()
        if any(keys):
            auto_mode = False
            if keys[pygame.K_w]: fb = 30
            if keys[pygame.K_s]: fb = -30
            if keys[pygame.K_a]: lr = -30
            if keys[pygame.K_d]: lr = 30
            if keys[pygame.K_UP]: ud = 30
            if keys[pygame.K_DOWN]: ud = -30
            if keys[pygame.K_LEFT]: yaw = -30
            if keys[pygame.K_RIGHT]: yaw = 30
            if keys[pygame.K_f]: tello.flip('f')
            if keys[pygame.K_ESCAPE]: raise KeyboardInterrupt

        tello.send_rc_control(lr, fb, ud, yaw)

        # === Hoop Sequence Detection ===
        if current_target_index < len(detection_sequence):
            target_color = detection_sequence[current_target_index]
            lower, upper = color_ranges[target_color]
            target_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            color_contours, _ = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if color_contours:
                largest = max(color_contours, key=cv2.contourArea)
                (_, _), c_radius = cv2.minEnclosingCircle(largest)
                if estimate_distance_cm(c_radius) <= 60:
                    r, g, b = color_to_led[target_color]
                    tello.set_led(r, g, b, 2, 2)
                    print(f"[SEQUENCE] Detected: {target_color} ({current_target_index+1}/{len(detection_sequence)})")
                    with open("detected_colors.txt", "a") as f:
                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {target_color} detected\n")
                    current_target_index += 1
                    time.sleep(1.5)

        # === HUD Display ===
        cv2.putText(frame, f"Battery: {tello.get_battery()}%", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        if current_target_index < len(detection_sequence):
            cv2.putText(frame, f"Next: {detection_sequence[current_target_index]}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("Color Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Interrupted. Landing...")

finally:
    if tello.is_flying:
        tello.land()
    tello.end()
    cv2.destroyAllWindows()
    pygame.quit()
