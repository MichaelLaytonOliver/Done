import cv2
import numpy as np
import pygame
import time
from tkinter import Tk, messagebox
from drone_interface import TelloAdapter  # or DummyAdapter

# Load drone
drone = TelloAdapter()  # Or DummyAdapter() for testing
drone.connect()
print(f"[DRONE] Battery: {drone.get_battery()}%")
drone.streamon()
drone.takeoff()

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((400, 200))
pygame.display.set_caption("Drone Control")

# Colors & Setup
pink_lower = np.array([160, 100, 100])
pink_upper = np.array([180, 255, 255])
font = pygame.font.Font(None, 36)
manual_mode = False
velocity = [0, 0, 0, 0]
frame_center = (360, 240)
last_detected_colors = []
auto_flight = True
sequence = ["Purple", "Orange", "Yellow", "Orange", "Blue", "Red", "Purple", "Green", "Red", "Yellow", "Blue", "Green"]
sequence_index = 0
reversed_once = False

# HSV Trackbars
def nothing(x): pass
cv2.namedWindow("Trackbars")
for c in ['H', 'S', 'V']:
    cv2.createTrackbar(f"{c} Min", "Trackbars", pink_lower[['H', 'S', 'V'].index(c)], 255, nothing)
    cv2.createTrackbar(f"{c} Max", "Trackbars", pink_upper[['H', 'S', 'V'].index(c)], 255, nothing)

# Tkinter popup for confirmation
def ask_approval(color_name):
    root = Tk()
    root.withdraw()
    result = messagebox.askyesno("Color Detected", f"Detected {color_name}. Confirm detection?")
    root.destroy()
    return result

# Main loop
try:
    while True:
        # Pygame event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                drone.land()
                pygame.quit()
                exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    manual_mode = not manual_mode
                    auto_flight = not manual_mode
                elif event.key == pygame.K_ESCAPE:
                    drone.land()
                    pygame.quit()
                    exit()
        keys = pygame.key.get_pressed()
        if manual_mode:
            velocity = [int(keys[pygame.K_RIGHT]) - int(keys[pygame.K_LEFT]),
                        int(keys[pygame.K_UP]) - int(keys[pygame.K_DOWN]),
                        int(keys[pygame.K_w]) - int(keys[pygame.K_s]),
                        int(keys[pygame.K_d]) - int(keys[pygame.K_a])]
            drone.send_rc_control(*[v * 50 for v in velocity])
        else:
            drone.send_rc_control(0, 0, 0, 0)

        # Get HSV values from trackbars
        h_min = cv2.getTrackbarPos("H Min", "Trackbars")
        h_max = cv2.getTrackbarPos("H Max", "Trackbars")
        s_min = cv2.getTrackbarPos("S Min", "Trackbars")
        s_max = cv2.getTrackbarPos("S Max", "Trackbars")
        v_min = cv2.getTrackbarPos("V Min", "Trackbars")
        v_max = cv2.getTrackbarPos("V Max", "Trackbars")
        pink_lower = np.array([h_min, s_min, v_min])
        pink_upper = np.array([h_max, s_max, v_max])

        # Frame processing
        frame = drone.get_frame()
        if frame is None:
            continue
        frame = cv2.resize(frame, (720, 480))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, pink_lower, pink_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                if auto_flight and abs(x - frame_center[0]) < 100 and ask_approval(sequence[sequence_index]):
                    print(f"[SEQ] Confirmed: {sequence[sequence_index]}")
                    last_detected_colors.append(sequence[sequence_index])
                    sequence_index += 1

                    if sequence_index == len(sequence):
                        if not reversed_once:
                            sequence.reverse()
                            sequence_index = 0
                            reversed_once = True
                            print("[SEQ] Reversed sequence for second run.")
                        else:
                            print("[SEQ] All hoops detected. Ending sequence.")
                            drone.land()
                            break
                    drone.move_forward(100)
        else:
            if not manual_mode:
                drone.send_rc_control(0, 0, 0, 30)

        cv2.imshow("Drone Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break
finally:
    drone.land()
    cv2.destroyAllWindows()
    pygame.quit()
