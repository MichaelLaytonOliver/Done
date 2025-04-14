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

# Init Pygame for input
pygame.init()
screen = pygame.display.set_mode((400, 300))  # Small window to receive keyboard events
pygame.display.set_caption("Tello Control")

try:
    tello.takeoff()
    time.sleep(2)  # Stabilize

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

        # Autonomous tracking
        if center:
            frame_center_x = frame.shape[1] // 2
            if center[0] < frame_center_x - 50:
                tello.move_left(20)
            elif center[0] > frame_center_x + 50:
                tello.move_right(20)
                

        # Handle keyboard input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        keys = pygame.key.get_pressed()

        # Movement
        if keys[pygame.K_w]:
            tello.move_forward(20)
        if keys[pygame.K_s]:
            tello.move_back(20)
        if keys[pygame.K_a]:
            tello.move_left(20)
        if keys[pygame.K_d]:
            tello.move_right(20)
        if keys[pygame.K_UP]:
            tello.move_up(20)
        if keys[pygame.K_DOWN]:
            tello.move_down(20)
        if keys[pygame.K_LEFT]:
            tello.rotate_counter_clockwise(30)
        if keys[pygame.K_RIGHT]:
            tello.rotate_clockwise(30)
        if keys[pygame.K_ESCAPE]:
            raise KeyboardInterrupt

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Keyboard interrupt received. Landing drone...")

finally:
    if tello.is_flying:
        tello.land()
    tello.end()
    cv2.destroyAllWindows()
    pygame.quit()
