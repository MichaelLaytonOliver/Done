import time
import cv2
import numpy as np
from djitellopy import Tello
import keyboard

# === Drone Init ===
tello = Tello()
tello.connect()
tello.streamon()
tello.takeoff()
is_flying = True
frame_read = tello.get_frame_read()

# === Color Sequence ===
color_to_led = {
    'purple': (148, 0, 211),
    'orange': (255, 165, 0),
    'yellow': (255, 255, 0),
    'blue': (0, 0, 255),
    'red': (255, 0, 0),
    'green': (0, 255, 0)
}

detection_sequence = [
    "purple", "orange", "yellow", "orange",
    "blue", "red", "purple", "green",
    "red", "yellow", "blue", "green"
]
current_target_index = 0

color_ranges = {
    'purple': ([120, 50, 50], [150, 255, 255]),
    'orange': ([10, 100, 100], [25, 255, 255]),
    'yellow': ([25, 100, 100], [35, 255, 255]),
    'red': ([0, 120, 70], [10, 255, 255]),
    'blue': ([100, 150, 0], [140, 255, 255]),
    'green': ([40, 70, 70], [80, 255, 255])
}

def estimate_distance_cm(radius):
    if radius <= 0:
        return float('inf')
    return 5000 / radius

# === Trackbars ===
cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", 140, 179, lambda x: None)
cv2.createTrackbar("S Min", "Trackbars", 110, 255, lambda x: None)
cv2.createTrackbar("V Min", "Trackbars", 150, 255, lambda x: None)
cv2.createTrackbar("H Max", "Trackbars", 179, 179, lambda x: None)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, lambda x: None)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, lambda x: None)

# === Main Loop ===
auto_mode = False
last_color_detection_time = 0
last_auto_toggle_time = 0
last_rc_command_time = 0
rc_send_interval = 0.01
last_l_press_time = 0

try:
    while True:
        now = time.time()
        frame = frame_read.frame
        if frame is None:
            print("No frame detected.")
            continue

        # === HSV Trackbar Detection ===
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

            if radius > 10 and distance_cm <= 120:
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.putText(frame, f"{int(distance_cm)}cm", (center[0], center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                center = None

        # === Movement Variables ===
        left_right_velocity = 0
        forward_backward_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0

        # === Auto Mode Logic ===
        if auto_mode and center:
            frame_center_x = frame.shape[1] // 2
            frame_center_y = frame.shape[0] // 2
            dx = center[0] - frame_center_x
            dy = center[1] - frame_center_y

            ideal_radius = 45
            radius_error = radius - ideal_radius
            threshold = 10
            k_pos = 0.2
            k_radius = 0.5

            if distance_cm < 30:
                forward_backward_velocity = -100
            else:
                if abs(dx) > threshold:
                    yaw_velocity = int(k_pos * dx)
                if abs(dy) > threshold:
                    up_down_velocity = int(-k_pos * dy)
                if abs(radius_error) > 5:
                    forward_backward_velocity = int(k_radius * radius_error)

            yaw_velocity = max(-100, min(100, yaw_velocity))
            up_down_velocity = max(-100, min(100, up_down_velocity))
            forward_backward_velocity = max(-100, min(100, forward_backward_velocity))

            print(f"[AUTO] dx={dx}, dy={dy}, r={radius}, dist={int(distance_cm)} => "f"YAW: {yaw_velocity}, UD: {up_down_velocity}, FB: {forward_backward_velocity}")

        # === Keyboard Controls (Work Always) ===
        if keyboard.is_pressed('q'):
            raise KeyboardInterrupt

        if keyboard.is_pressed('w'):
            forward_backward_velocity = 60
        elif keyboard.is_pressed('s'):
            forward_backward_velocity = -60

        if keyboard.is_pressed('a'):
            left_right_velocity = -60
        elif keyboard.is_pressed('d'):
            left_right_velocity = 60

        if keyboard.is_pressed('up'):
            up_down_velocity = 100
        elif keyboard.is_pressed('down'):
            up_down_velocity = -100

        if keyboard.is_pressed('left'):
            yaw_velocity = -100
        elif keyboard.is_pressed('right'):
            yaw_velocity = 100

        if keyboard.is_pressed('f'):
            tello.flip_forward()
            time.sleep(1)
            
        if keyboard.is_pressed('l') and time.time() - last_l_press_time > 2:
            if is_flying == False:
                print("[COMMAND] Landing...")
                tello.takeoff()
                is_flying = True
            if is_flying == True:
                print("[COMMAND] Taking off...")
                tello.land()
                is_flying = False
                last_l_press_time = time.time()
        

        if keyboard.is_pressed('m') and now - last_auto_toggle_time > 1:
            auto_mode = not auto_mode
            print(f"[MODE] Auto mode {'ON' if auto_mode else 'OFF'}")
            last_auto_toggle_time = now

        # === RC Command Send Timing ===
        if now - last_rc_command_time > rc_send_interval:
            tello.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
            last_rc_command_time = now

        # === Hoop Detection (Non-blocking) ===
        target_color = detection_sequence[current_target_index] if current_target_index < len(detection_sequence) else None
        if target_color and now - last_color_detection_time > 2:
            lower, upper = color_ranges[target_color]
            color_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                (_, _), r = cv2.minEnclosingCircle(largest)
                dist = estimate_distance_cm(r)
                if dist <= 60:
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                    with open("detected_colors.txt", "a") as f:
                        f.write(f"{timestamp} - {target_color} detected (Sequence {current_target_index+1}/{len(detection_sequence)})\n")
                    print(f"[SEQUENCE] {target_color} hoop detected in order ({current_target_index+1}/{len(detection_sequence)})")
                    current_target_index += 1
                    last_color_detection_time = now

        # === UI Overlays ===
        battery = tello.get_battery()
        cv2.putText(frame, f"Battery: {battery}%", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        if current_target_index < len(detection_sequence):
            cv2.putText(frame, f"Next Hoop: {target_color}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("Color Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('x'):
            raise KeyboardInterrupt

except KeyboardInterrupt:
    print("Landing drone...")
    tello.land()
    tello.streamoff()
    cv2.destroyAllWindows()