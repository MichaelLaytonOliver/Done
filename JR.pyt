import cv2
from djitellopy import Tello
import time
import pygame
import os

# Fix for Windows with no video device
os.environ["SDL_VIDEODRIVER"] = "dummy"

# Connect to Tello
tello = Tello()
tello.connect()
tello.streamon()
frame_read = tello.get_frame_read()

# Define blue color range for detection
lower_blue = (100, 50, 50)
upper_blue = (130, 255, 255)

# Init Pygame and Joystick
pygame.init()
pygame.joystick.init()

joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Joystick detected:", joystick.get_name())
else:
    print("No joystick detected.")

try:
    tello.takeoff()
    time.sleep(2)  # Give it a moment to stabilize

    while True:
        frame = frame_read.frame
        if frame is None:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(frame, center, radius, (0, 255, 0), 2)

        cv2.imshow("Color Detection", frame)

        # Autonomous tracking based on object position
        if center:
            frame_center_x = frame.shape[1] // 2
            if center[0] < frame_center_x - 50:
                tello.move_left(20)
            elif center[0] > frame_center_x + 50:
                tello.move_right(20)

        # Manual control with joystick
        if joystick:
            pygame.event.pump()  # Process internal joystick events

            axis_lr = joystick.get_axis(0)  # Left/right
            axis_fb = joystick.get_axis(1)  # Forward/back
            axis_ud = joystick.get_axis(3)  # Up/down
            axis_yaw = joystick.get_axis(2)  # Rotate

            threshold = 0.2  # Deadzone to ignore small movements

            if abs(axis_lr) > threshold:
                direction = "right" if axis_lr > 0 else "left"
                tello.move_right(20) if axis_lr > 0 else tello.move_left(20)

            if abs(axis_fb) > threshold:
                direction = "forward" if axis_fb < 0 else "back"
                tello.move_forward(20) if axis_fb < 0 else tello.move_back(20)

            if abs(axis_ud) > threshold:
                tello.move_up(20) if axis_ud < 0 else tello.move_down(20)

            if abs(axis_yaw) > threshold:
                tello.rotate_clockwise(20) if axis_yaw > 0 else tello.rotate_counter_clockwise(20)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

finally:
    if tello.is_flying:
        tello.land()
    tello.end()
    cv2.destroyAllWindows()
