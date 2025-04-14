import cv2
from djitellopy import Tello
import time
import pygame
import os

os.environ["SDL_VIDEODRIVER"] = "dummy"

# Connect to Tello
tello = Tello()
tello.connect()
tello.streamon()
frame_read = tello.get_frame_read()


# Define blue color range for detection
lower_blue = (100, 50, 50)
upper_blue = (130, 255, 255)

# Init Pygame
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Tello Control")

try:
    tello.takeoff()
    #

    auto_mode = True  # Toggle for autonomous behavior

    while True:
        frame = frame_read.frame
        if frame is None:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        center = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            radius = int(radius)
            if radius > 10:
                cv2.circle(frame, center, radius, (0, 255, 0), 2)

        # Velocities
        left_right_velocity = 0
        forward_backward_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0

        if auto_mode and center:
            frame_center_x = frame.shape[1] // 2
            frame_center_y = frame.shape[0] // 2
            dx = center[0] - frame_center_x
            dy = center[1] - frame_center_y

            threshold = 30  # Only move if offset is bigger than this
            k = 0.3  # Proportional control constant

            if abs(dx) > threshold:
                left_right_velocity = int(k * dx)
            if abs(dy) > threshold:
                forward_backward_velocity = int(-k * dy)

            # Clamp velocities
            left_right_velocity = max(-30, min(30, left_right_velocity))
            forward_backward_velocity = max(-30, min(30, forward_backward_velocity))

            print(f"[AUTO] Moving to center. dx={dx}, dy={dy} => LR: {left_right_velocity}, FB: {forward_backward_velocity}")

        # Manual override
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        keys = pygame.key.get_pressed()
        if any(keys):
            auto_mode = False  # Turn off auto if manual keys pressed

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

            print(f"[MANUAL] LR: {left_right_velocity}, FB: {forward_backward_velocity}, UD: {up_down_velocity}, YAW: {yaw_velocity}")

        tello.send_rc_control(
            left_right_velocity,
            forward_backward_velocity,
            up_down_velocity,
            yaw_velocity
        )

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
