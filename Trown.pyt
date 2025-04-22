import time
import cv2
import numpy as np
import keyboard
import tkinter as tk
from tkinter import messagebox
import threading
from threading import Lock
from djitellopy import Tello  # 💡 Import Tello class

# === Tello Init ===
tello = Tello()
tello.connect()
print(f"[TELLO] Battery: {tello.get_battery()}%")
tello.streamon()
cap = tello.get_frame_read()

# === Tello flying flag ===
is_flying = False

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
green_count = 0
reversing = False

# === Robust HSV Color Ranges ===
color_ranges = {
    'purple': ((125, 50, 50), (160, 255, 255)),
    'orange': ((5, 150, 150), (20, 255, 255)),
    'yellow': ((20, 100, 100), (35, 255, 255)),
    'red':    ((0, 120, 70), (10, 255, 255)),
    'blue':   ((90, 100, 100), (130, 255, 255)),
    'green':  ((35, 100, 100), (85, 255, 255))
}

def estimate_distance_cm(radius):
    return 5000 / radius if radius > 0 else float('inf')

# === Trackbars ===
cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", 140, 179, lambda x: None)
cv2.createTrackbar("S Min", "Trackbars", 110, 255, lambda x: None)
cv2.createTrackbar("V Min", "Trackbars", 150, 255, lambda x: None)
cv2.createTrackbar("H Max", "Trackbars", 179, 179, lambda x: None)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, lambda x: None)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, lambda x: None)

# === Thread-safe Approval Handler ===
class ApprovalHandler:
    def __init__(self):
        self.result = None
        self.waiting = False
        self.lock = Lock()

    def ask(self, color):
        with self.lock:
            if self.waiting or self.result is not None:
                return
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

approval_handler = ApprovalHandler()
last_hoop_visible = False
rc_send_interval = 0.01
hoop_was_detected = False

# === Initializing necessary variables ===
now = time.time()
last_l_press_time = now
last_auto_toggle_time = now
auto_mode = True
last_rc_command_time = now

left_right_velocity = 0
forward_backward_velocity = 0
up_down_velocity = 0
yaw_velocity = 0

# === Throw detection ===
last_velocity = [0, 0, 0]  # [x, y, z] velocities (assuming z is vertical speed)
velocity_threshold = 15  # Velocity change threshold to detect a throw

# === Main Loop ===
try:
    while True:
        frame = cap.frame
        ret = frame is not None
        if not ret:
            now = time.time()
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

        # === Hoop Detection ===
        target_color = detection_sequence[current_target_index] if current_target_index < len(detection_sequence) else None

        if not hasattr(approval_handler, 'last_detection_time'):
            approval_handler.last_detection_time = 0

        if target_color and time.time() - approval_handler.last_detection_time > 2:
            lower, upper = color_ranges[target_color]

            if target_color == 'red':
                mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
                mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
                color_mask = cv2.bitwise_or(mask1, mask2)
            else:
                color_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                (x, y), r = cv2.minEnclosingCircle(largest)
                center = (int(x), int(y))
                radius = int(r)
                distance_cm = estimate_distance_cm(radius)

                if radius > 10:
                    cv2.circle(frame, center, radius, (0, 255, 255), 2)
                    cv2.putText(frame, f"{target_color} {int(distance_cm)}cm", (center[0]-20, center[1]-radius-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if radius > 10 and distance_cm >= 60:
                    hoop_was_detected = True
                    last_hoop_visible = True
                else:
                    if last_hoop_visible and hoop_was_detected and not approval_handler.waiting and approval_handler.result is None:
                        print(f"[POPUP TRIGGERED] {target_color} hoop disappeared. Awaiting user confirmation...")
                        approval_handler.ask(target_color)
                        approval_handler.last_detection_time = time.time()
                        hoop_was_detected = False
                    last_hoop_visible = False

        # === Throw detection logic ===
        current_velocity = tello.get_speed()  # Get current speed in x, y, z (use Tello's get_speed() or similar method)
        velocity_change = np.linalg.norm(np.array(current_velocity) - np.array(last_velocity))

        if velocity_change > velocity_threshold and not is_flying:
            print("[THROW DETECTED] Starting flight...")
            tello.takeoff()
            is_flying = True

        last_velocity = current_velocity  # Update last velocity for the next iteration

        # === Handle Approval Response ===
        if approval_handler.result is not None:
            if approval_handler.result:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                with open("detected_colors.txt", "a") as f:
                    f.write(f"{timestamp} - {target_color} detected (Sequence {current_target_index+1}/{len(detection_sequence)})\n")

                print(f"[SEQUENCE APPROVED] {target_color} confirmed ({current_target_index+1}/{len(detection_sequence)})")

                # === Tello actions ===
                if not is_flying:
                    tello.takeoff()
                    is_flying = True
                    time.sleep(1)

                tello.move_forward(30)
                time.sleep(1)

                if target_color == "green":
                    green_count += 1

                current_target_index += 1

                if green_count == 2 and not reversing:
                    reversing = True
                    detection_sequence = detection_sequence[::-1]
                    current_target_index = 0
                    print("[SEQUENCE] Green detected twice. Reversing sequence and starting over.")
            else:
                print(f"[SEQUENCE REJECTED] {target_color} detection denied. Retrying...")

            approval_handler.result = None

        # === Auto Mode Logic ===
        if 'center' in locals() and center:
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

        # Reset velocities if no keys are pressed
        left_right_velocity = 0
        forward_backward_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0

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

        if now - last_rc_command_time > rc_send_interval:
            tello.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
            last_rc_command_time = now

        # === Display battery info ===
        battery = tello.get_battery()
        frame_height, frame_width = frame.shape[:2]
        cv2.rectangle(frame, (10, 10), (210, 50), (0, 0, 0), -1)
        cv2.putText(frame, f'Battery: {battery}%', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Show frame with battery info
        cv2.imshow("Color Detection", frame)

        # Quit when 'x' is pressed
        if cv2.waitKey(1) & 0xFF == ord('x'):
            break

finally:
    cap.stop()
    cv2.destroyAllWindows()
    tello.end()
