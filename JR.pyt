import time
import cv2
import numpy as np
from djitellopy import Tello
import keyboard
import tkinter as tk
from tkinter import messagebox
import threading
from threading import Lock

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
    'purple': ([120, 50, 255], [162, 0, 255]),
    'orange': ([255, 213, 0], [25, 255, 255]),
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


# Thread-safe approval popup handler
class ApprovalHandler:
    def __init__(self):
        self.result = None
        self.waiting = False
        self.lock = Lock()

    def ask(self, color):
        with self.lock:
            if self.waiting:
                return  # Prevent multiple popups
            self.result = None
            self.waiting = True
        threading.Thread(target=self._popup, args=(color,), daemon=True).start()

    def _popup(self, color):
        def run_popup():
            result = messagebox.askyesno("Hoop Detection", f"Detected {color} hoop.\nApprove detection?")
            with self.lock:
                self.result = result
                self.waiting = False
            root.quit()
            root.destroy()

        root = tk.Tk()
        root.withdraw()
        root.after(0, run_popup)
        root.mainloop()

# Initialize once here
approval_handler = ApprovalHandler()

try:
    while True:
        now = time.time()
        frame = frame_read.frame
        if frame is None:
            print("No frame detected.")
            continue
        # Fix for null frame safety
        frame = frame_read.frame
        if frame is None or frame.shape[0] == 0 or frame.shape[1] == 0:
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

        # === Hoop Detection (Non-blocking) ===
        target_color = detection_sequence[current_target_index] if current_target_index < len(detection_sequence) else None
        if target_color and now - last_color_detection_time > 2:
            lower, upper = color_ranges[target_color]
            color_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                (x, y), r = cv2.minEnclosingCircle(largest)
                dist = estimate_distance_cm(r)

                center = (int(x), int(y))
                radius = int(r)
                distance_cm = dist

                if radius > 10:
                    cv2.circle(frame, center, radius, (0, 255, 255), 2)
                    cv2.putText(frame, f"{target_color} {int(dist)}cm", (center[0]-20, center[1]-radius-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if dist <= 60 and not approval_handler.waiting:
                    tello.send_rc_control(0, 0, 0, 0)
                    print(f"[DETECTED] {target_color} hoop. Awaiting user confirmation...")
                    approval_handler.ask(target_color)
                    last_color_detection_time = now

                # Prevent race condition: only handle result when not waiting anymore
            if not approval_handler.waiting and approval_handler.result is not None:
                if approval_handler.result:
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                    with open("detected_colors.txt", "a") as f:
                        f.write(f"{timestamp} - {target_color} detected (Sequence {current_target_index+1}/{len(detection_sequence)})\n")
                    print(f"[SEQUENCE APPROVED] {target_color} confirmed ({current_target_index+1}/{len(detection_sequence)})")
                    current_target_index += 1
                else:
                    print(f"[SEQUENCE REJECTED] {target_color} detection denied. Retrying...")
                approval_handler.result = None


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

        # === Keyboard Controls ===
        speed_multiplier = 1.5 if keyboard.is_pressed('shift') or keyboard.is_pressed('right shift') else 1
        base_speed_fb_lr = int(60 * speed_multiplier)
        base_speed_ud_yaw = int(100 * speed_multiplier)

        if keyboard.is_pressed('p'):
            with open("detected_colors.txt", "w") as f:
                f.write("")
            print("[FILE] detected_colors.txt has been cleared.")
            time.sleep(0.5)

        if keyboard.is_pressed('w'):
            forward_backward_velocity = base_speed_fb_lr
        elif keyboard.is_pressed('s'):
            forward_backward_velocity = -base_speed_fb_lr

        if keyboard.is_pressed('a'):
            left_right_velocity = -base_speed_fb_lr
        elif keyboard.is_pressed('d'):
            left_right_velocity = base_speed_fb_lr

        if keyboard.is_pressed('up'):
            up_down_velocity = base_speed_ud_yaw
        elif keyboard.is_pressed('down'):
            up_down_velocity = -base_speed_ud_yaw

        if keyboard.is_pressed('left'):
            yaw_velocity = -base_speed_ud_yaw
        elif keyboard.is_pressed('right'):
            yaw_velocity = base_speed_ud_yaw

        if keyboard.is_pressed('l') and time.time() - last_l_press_time > 2:
            if not is_flying:
                print("[COMMAND] Taking off...")
                tello.takeoff()
                is_flying = True
            else:
                print("[COMMAND] Landing...")
                tello.land()
                is_flying = False
            last_l_press_time = time.time()

        if keyboard.is_pressed('m') and now - last_auto_toggle_time > 1:
            auto_mode = not auto_mode
            print(f"[MODE] Auto mode {'ON' if auto_mode else 'OFF'}")
            last_auto_toggle_time = now

        if keyboard.is_pressed('t'):
            test_color = detection_sequence[current_target_index]
            lower, upper = color_ranges[test_color]
            test_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            cv2.imshow(f"{test_color.upper()} Mask", test_mask)
            
        # Add a timestamp for flip command
            last_flip_time = 0
            flip_cooldown = 1  # 1 second cooldown between flips

            if keyboard.is_pressed('f') and time.time() - last_flip_time > flip_cooldown:
                tello.flip_back()  # Perform a forward flip
                last_flip_time = time.time()  # Update the last flip time
                print("[COMMAND] Flip executed")



        if now - last_rc_command_time > rc_send_interval:
            tello.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
            last_rc_command_time = now

        battery = tello.get_battery()

                # === Battery Indicator Positioning (Top-Right Corner) ===
        # === Battery Indicator Positioning (Top-Right Corner) ===
        frame_height, frame_width = frame.shape[:2]
        width, height = 100, 30
        cap_width = 6
        margin = 20
        x = frame_width - width - cap_width - margin
        y = frame_height - height - margin  # Lower the battery indicator by changing this value
        
        # Choose color based on battery percentage
        if battery >= 50:
            color = (0, 255, 0)       # Green
        elif battery >= 20:
            color = (0, 255, 255)     # Yellow
        else:
            color = (0, 0, 255)       # Red
        
        # Draw battery outline
        cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 0), 2)
        # Draw battery cap
        cv2.rectangle(frame, (x + width, y + height // 4), (x + width + cap_width, y + 3 * height // 4), (0, 0, 0), -1)
        # Fill battery based on charge
        fill_width = int((battery / 100) * width)
        cv2.rectangle(frame, (x, y), (x + fill_width, y + height), color, -1)
        # Label battery %
        cv2.putText(frame, f"{battery}%", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        

        

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
