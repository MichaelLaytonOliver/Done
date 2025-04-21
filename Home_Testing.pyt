import time
import cv2
import numpy as np
import keyboard
import tkinter as tk
from tkinter import messagebox
import threading
from threading import Lock

# === Tello Init ===
tello = Tello()
tello.connect()
print(f"[TELLO] Battery: {tello.get_battery()}%")

tello.streamon()
cap = tello.get_frame_read()

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
    'red':    ((0, 120, 70), (10, 255, 255)),   # Red wraps around, we'll handle two ranges below
    'blue':   ((90, 100, 100), (130, 255, 255)),
    'green':  ((35, 100, 100), (85, 255, 255))
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

# === Thread-safe Approval Handler ===
class ApprovalHandler:
    def __init__(self):
        self.result = None
        self.waiting = False
        self.lock = Lock()

    def ask(self, color):
        with self.lock:
            if self.waiting:
                return
            if self.result is not None:  # Don't ask again if already handled
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
hoop_was_detected = False


# === Main Loop ===
try:
    while True:
        now = time.time()
        frame = cap.frame
        ret = frame is not None
        
        if not ret or frame is None:
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

        center = None
        radius = 0
        distance_cm = float('inf')

        # === Hoop Detection ===
        target_color = detection_sequence[current_target_index] if current_target_index < len(detection_sequence) else None
        
        # Delay between detections (in seconds)
        if not hasattr(approval_handler, 'last_detection_time'):
            approval_handler.last_detection_time = 0
        
        if target_color and time.time() - approval_handler.last_detection_time > 2:
            lower, upper = color_ranges[target_color]
        
            if target_color == 'red':
                lower1 = np.array([0, 120, 70])
                upper1 = np.array([10, 255, 255])
                lower2 = np.array([170, 120, 70])
                upper2 = np.array([180, 255, 255])
                mask1 = cv2.inRange(hsv, lower1, upper1)
                mask2 = cv2.inRange(hsv, lower2, upper2)
                color_mask = cv2.bitwise_or(mask1, mask2)
            else:
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


        
        if approval_handler.result is not None:
            if approval_handler.result:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                with open("detected_colors.txt", "a") as f:
                    f.write(f"{timestamp} - {target_color} detected (Sequence {current_target_index+1}/{len(detection_sequence)})\n")

                print(f"[SEQUENCE APPROVED] {target_color} confirmed ({current_target_index+1}/{len(detection_sequence)})")

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

            approval_handler.result = None  # ✅ Only clear after handling


                

        # === Display info ===
        if keyboard.is_pressed('p'):
            with open("detected_colors.txt", "w") as f:
                f.write("")
            print("[FILE] detected_colors.txt has been cleared.")
            time.sleep(0.5)

        if keyboard.is_pressed('t') and current_target_index < len(detection_sequence):
            test_color = detection_sequence[current_target_index]
            lower = np.array(color_ranges[test_color])
            upper = lower
            test_mask = cv2.inRange(hsv, lower, upper)
            cv2.imshow(f"{test_color.upper()} Mask", test_mask)

        # === Display Target Info ===
        if current_target_index < len(detection_sequence):
            cv2.putText(frame, f"Next Hoop: {target_color}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("Color Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('x'):
            break

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    cv2.destroyAllWindows()