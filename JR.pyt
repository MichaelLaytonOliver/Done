import cv2
from djitellopy import Tello
import time
import pygame
import os
import numpy as np

os.environ["SDL_VIDEODRIVER"] = "dummy"
cv2.setUseOptimized(True)


# Connect to the Tello drone
tello = Tello()
tello.connect()
print(f"Battery: {tello.get_battery()}%")
tello.streamon()
time.sleep(2)

frame_read = tello.get_frame_read()

# Default HSV range for pink (narrowed to avoid purple)
lower_pink_default = np.array([160, 100, 100])
upper_pink_default = np.array([175, 255, 255])

def nothing(x):
    pass

cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", lower_pink_default[0], 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", lower_pink_default[1], 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", lower_pink_default[2], 255, nothing)
cv2.createTrackbar("H Max", "Trackbars", upper_pink_default[0], 179, nothing)
cv2.createTrackbar("S Max", "Trackbars", upper_pink_default[1], 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", upper_pink_default[2], 255, nothing)

pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Tello Control")

# === Distance estimation ===
def estimate_distance_cm(radius):
    if radius == 0:
        return float('inf')
    return 800 / radius  # tweak this constant based on real-world size

try:
    tello.takeoff()
except Exception as e:
    print(f"Takeoff failed: {e}. Retrying in 2 seconds...")
    time.sleep(2)
    tello.takeoff()

tello.send_rc_control(0, 0, 0, 0)

auto_mode = True
# Sequence of hula hoop colors to detect (in order)
target_color_sequence = [
    "Purple", "Orange", "Yellow", "Orange","Blue", "Red", "Purple", "Green","Red", "Yellow", "Blue", "Green"]
sequence_index = 0  # Start with first color

try:
    while True:
        frame = frame_read.frame
        if frame is None:
            print("Failed to get frame. Retrying...")
            time.sleep(0.1)
            continue

        h_min = cv2.getTrackbarPos("H Min", "Trackbars")
        s_min = cv2.getTrackbarPos("S Min", "Trackbars")
        v_min = cv2.getTrackbarPos("V Min", "Trackbars")
        h_max = cv2.getTrackbarPos("H Max", "Trackbars")
        s_max = cv2.getTrackbarPos("S Max", "Trackbars")
        v_max = cv2.getTrackbarPos("V Max", "Trackbars")

        lower_pink = np.array([h_min, s_min, v_min])
        upper_pink = np.array([h_max, s_max, v_max])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        center = None
        radius = 0
        distance_cm = float('inf')

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            radius = int(radius)

            distance_cm = estimate_distance_cm(radius)

            if radius > 10 and distance_cm <= 60:  # max distance ~2ft (60 cm)
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.putText(frame, f"{int(distance_cm)}cm", (center[0], center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                center = None

        left_right_velocity = 0
        forward_backward_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0

        if auto_mode and center:
            frame_center_x = frame.shape[1] // 2
            frame_center_y = frame.shape[0] // 2
            dx = center[0] - frame_center_x  # Horizontal distance from the center of the frame
            dy = center[1] - frame_center_y  # Vertical distance from the center of the frame

            # Ideal radius and other tuning parameters
            ideal_radius = 45
            radius_error = radius - ideal_radius
            threshold = 5  # Minimum pixel deviation to trigger yaw control
            k_pos = 45  # Yaw sensitivity coefficient (adjust to control rotation speed)
            k_radius = 1  # Forward/backward control sensitivity

            if distance_cm < 5:
                # Too close, back up slowly
                forward_backward_velocity = -40
                yaw_velocity = 0
                up_down_velocity = 0
                print(f"[TOO CLOSE] distance={int(distance_cm)}cm — backing up...")
            else:
                # Normal object tracking logic
                if abs(dx) > threshold:  # If object is not centered, adjust yaw
                    yaw_velocity = int(k_pos * dx)  # Rotate the drone based on horizontal position (dx)

                if abs(dy) > threshold:  # Adjust up/down motion based on vertical position (dy)
                    up_down_velocity = int(-k_pos * dy)

                if abs(radius_error) > 5:  # Adjust forward/backward movement based on object size
                    forward_backward_velocity = int(k_radius * radius_error)

            # Constrain the velocities to ensure safe speeds
            yaw_velocity = max(-30, min(30, yaw_velocity))
            up_down_velocity = max(-30, min(30, up_down_velocity))
            forward_backward_velocity = max(-30, min(30, forward_backward_velocity))

            print(f"[AUTO] dx={dx}, dy={dy}, radius={radius}, dist={int(distance_cm)}cm => "f"YAW: {yaw_velocity}, UD: {up_down_velocity}, FB: {forward_backward_velocity}")

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
        
        keys = pygame.key.get_pressed()
        if any(keys):
            auto_mode = False
        
            if keys[pygame.K_w]:
                forward_backward_velocity = 30
            elif keys[pygame.K_s]:
                forward_backward_velocity = -30
        
            if keys[pygame.K_a]:
                left_right_velocity = -30
            elif keys[pygame.K_d]:
                left_right_velocity = 30
        
            if keys[pygame.K_UP]:
                up_down_velocity = 30
            elif keys[pygame.K_DOWN]:
                up_down_velocity = -30
        
            if keys[pygame.K_LEFT]:
                yaw_velocity = -30
            elif keys[pygame.K_RIGHT]:
                yaw_velocity = 30
        
            if keys[pygame.K_ESCAPE]:
                raise KeyboardInterrupt
        
            # Check if "F" key is pressed for a flip
            if keys[pygame.K_f]:
                tello.flip('f')  # Perform a forward flip
                print("[FLIP] Performing a forward flip.")
        
            print(f"[MANUAL] LR: {left_right_velocity}, FB: {forward_backward_velocity}, "f"UD: {up_down_velocity}, YAW: {yaw_velocity}")
        
        tello.send_rc_control(
            left_right_velocity,
            forward_backward_velocity,
            up_down_velocity,
            yaw_velocity
        )

        battery = tello.get_battery()
        cv2.putText(frame, f"Battery: {battery}%", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Define additional color HSV ranges (adjust as needed)
        color_ranges = {
            "Purple": ((125, 50, 50), (150, 255, 255)),
            "Orange": ((10, 100, 100), (25, 255, 255)),
            "Yellow": ((25, 100, 100), (35, 255, 255)),
            "Red": ((0, 100, 100), (10, 255, 255)),
            "Blue": ((100, 100, 100), (130, 255, 255)),
            "Green": ((40, 50, 50), (80, 255, 255))
        }

        # Keep track of detections
        if 'color_detection_count' not in locals():
            color_detection_count = {}
        if 'last_detected_color' not in locals():
            last_detected_color = None

        # Keep track of visible colors from the last frame
        if 'prev_visible_colors' not in locals():
            prev_visible_colors = set()

        current_visible_colors = set()

# Define your target sequence (in order)
        target_sequence = ["Purple", "Orange", "Yellow", "Orange", "Blue", "Red", "Purple", "Green", "Red", "Yellow", "Blue", "Green"]
        current_target_index = 0

        # Define expected color detection sequence
        detection_sequence = [
            "Purple", "Orange", "Yellow", "Orange", "Blue", "Red",
            "Purple", "Green", "Red", "Yellow", "Blue", "Green"
        ]

        if 'current_target_index' not in locals():
            current_target_index = 0

        # Color to LED RGB mapping
        color_to_led = {
            "Purple": (127, 0, 255),   # Magenta
            "Orange": (255, 165, 0),
            "Yellow": (255, 255, 0),
            "Red": (255, 0, 0),
            "Blue": (0, 0, 255),
            "Green": (0, 255, 0)
        }

        target_color = detection_sequence[current_target_index] if current_target_index < len(detection_sequence) else None

        with open("detected_colors.txt", "a") as f:
            for color_name, (lower, upper) in color_ranges.items():
                if color_name != target_color:
                    continue  # Skip if it's not the current target

                color_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                color_contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if not color_contours:
                    continue
                
                largest_color_contour = max(color_contours, key=cv2.contourArea)
                (_, _), color_radius = cv2.minEnclosingCircle(largest_color_contour)
                distance_cm = estimate_distance_cm(color_radius)

                if distance_cm <= 60:
                    # Log and update progress if this is the correct color in sequence
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                    f.write(f"{timestamp} - {color_name} detected (Sequence {current_target_index+1}/{len(detection_sequence)})\n")
                    print(f"[SEQUENCE] {color_name} hoop detected in order ({current_target_index+1}/{len(detection_sequence)})")

                    # Set LED color based on hoop
                    r, g, b = color_to_led[color_name]
                    tello.set_led(r, g, b, 2, 2)  # Flash LED with that color

                    current_target_index += 1  # Move to next hoop

                    # Small delay but keep checking next hoop
                start = time.time()
                while time.time() - start < 1.5:
                    frame = frame_read.frame  # Refresh frame
                    continue  # Just wait out delay, but don't let detection repeat for same hoop
                



                    current_visible_colors.add(color_name)  # Only mark as visible if within range
                if sequence_index < len(target_color_sequence):
                            cv2.putText(frame, f"Next Hoop: {target_color}", (10, 70),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)



        cv2.imshow("Color Detection", frame)
        


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Keyboard interrupt received. Landing drone...")

finally:
    if tello.is_flying:
        tello.land()
    tello.end()
    cv2.destroyAllWindows()
    pygame.quit()
