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
tello.takeoff()
cap = tello.get_frame_read()

# === Flags & State ===
is_flying = True
auto_mode = True
rc_send_interval = 0.05
last_rc_command_time = time.time()
last_l_press_time = time.time()
last_auto_toggle_time = time.time()

# === Velocity Placeholders ===
left_right_velocity = 0
forward_backward_velocity = 0
up_down_velocity = 0
yaw_velocity = 0

# === Detection Configs ===
detection_sequence = ["purple", "orange", "yellow", "orange", "blue", "red", "purple", "green", "red", "yellow", "blue", "green"]
current_target_index = 0
green_count = 0
reversing = False
last_hoop_visible = False
last_flip_time = time.time() - 5  # allows flip immediately

# === HSV Color Ranges ===
color_ranges = {
    'purple': ((125, 50, 50), (160, 255, 255)),
    'orange': ((5, 150, 150), (20, 255, 255)),
    'yellow': ((20, 100, 100), (35, 255, 255)),
    'red':    ((0, 120, 70), (10, 255, 255)),
    'blue':   ((90, 100, 100), (130, 255, 255)),
    'green':  ((35, 100, 100), (85, 255, 255))
}

# === Trackbars ===
cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", 140, 179, lambda x: None)
cv2.createTrackbar("S Min", "Trackbars", 110, 255, lambda x: None)
cv2.createTrackbar("V Min", "Trackbars", 150, 255, lambda x: None)
cv2.createTrackbar("H Max", "Trackbars", 179, 179, lambda x: None)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, lambda x: None)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, lambda x: None)

# === Approval Handler ===
class ApprovalHandler:
    def __init__(self):
        self.result = None
        self.waiting = False
        self.lock = Lock()
        self.last_detection_time = 0  # Add this line

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

# === Distance Estimation ===
def estimate_distance_cm(radius):
    return 5000 / radius if radius > 0 else float('inf')

# === Main Loop ===
try:
    while True:
        frame = cap.frame
        if frame is None:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        target_color = detection_sequence[current_target_index] if current_target_index < len(detection_sequence) else None

        # === HSV Trackbar Detection ===
        h_min = cv2.getTrackbarPos("H Min", "Trackbars")
        s_min = cv2.getTrackbarPos("S Min", "Trackbars")
        v_min = cv2.getTrackbarPos("V Min", "Trackbars")
        h_max = cv2.getTrackbarPos("H Max", "Trackbars")
        s_max = cv2.getTrackbarPos("S Max", "Trackbars")
        v_max = cv2.getTrackbarPos("V Max", "Trackbars")
        lower_pink = np.array([h_min, s_min, v_min])
        upper_pink = np.array([h_max, s_max, v_max])

        # === Hoop Detection ===
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

        # === Handle Approval Response ===
        if approval_handler.result is not None:
            if approval_handler.result:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                with open("detected_colors.txt", "a") as f:
                    f.write(f"{timestamp} - {target_color} detected (Sequence {current_target_index+1}/{len(detection_sequence)})\n")

                print(f"[SEQUENCE APPROVED] {target_color} confirmed ({current_target_index+1}/{len(detection_sequence)})")

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

        # === Keyboard Controls ===
        speed_multiplier = 1.5 if keyboard.is_pressed('shift') else 1
        base_speed_fb_lr = int(60 * speed_multiplier)
        base_speed_ud_yaw = int(100 * speed_multiplier)

        left_right_velocity = 0
        forward_backward_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0

        # Movement keys
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
        if keyboard.is_pressed(';') and is_flying and time.time() - last_flip_time > 3:
            if tello.get_battery() > 30 and tello.get_height() > 0:
                try:
                    print("[COMMAND] Backflip initiated...")
                    tello.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.5)
                    tello.flip_back()
                    last_flip_time = time.time()
                    time.sleep(1)
                except Exception as e:
                    print(f"[ERROR] Flip failed: {e}")
            else:
                print("[WARNING] Flip aborted: low battery or altitude.")


        # Takeoff / Land
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

        # Mode Toggle
        if keyboard.is_pressed('m') and time.time() - last_auto_toggle_time > 2:
            auto_mode = not auto_mode
            print(f"[MODE] {'Auto' if auto_mode else 'Manual'} mode.")
            last_auto_toggle_time = time.time()

        # Send velocity commands
        if time.time() - last_rc_command_time > rc_send_interval:
            if is_flying:
                tello.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
            last_rc_command_time = time.time()

        # Display frame
        cv2.imshow("Hoop Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if current_target_index < len(detection_sequence):
                cv2.putText(frame, f"Next Hoop: {target_color}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        battery = tello.get_battery()
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

finally:
    tello.land()
    cv2.destroyAllWindows()
