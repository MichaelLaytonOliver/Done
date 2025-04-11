import cv2
from djitellopy import Tello
import time

tello = Tello()
tello.connect()
tello.streamon()  # Move this BEFORE get_frame_read()
frame_read = tello.get_frame_read()

lower_blue = (100, 50, 50)
upper_blue = (130, 255, 255)

try:
    tello.takeoff()
    
    while True:
        frame = frame_read.frame
        if frame is None:
            continue  # Skip if frame is invalid

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

        if center:
            if center[0] < frame.shape[1] // 2 - 50:
                tello.move_left(20)
            elif center[0] > frame.shape[1] // 2 + 50:
                tello.move_right(20)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.1)

finally:
    tello.end()
    cv2.destroyAllWindows()
