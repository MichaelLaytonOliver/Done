import cv2
from djitellopy import Tello

tello = Tello()
tello.connect()
tello.streamon()

frame_read = tello.get_frame_read()
lower_blue = (100, 50, 50)
upper_blue = (130, 255, 255)

while True:
    frame = frame_read.frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    center = None  # <- define it early
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
        # You can add move_up/down/forward/backward here based on center[1]

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

tello.end()
cv2.destroyAllWindows()
