import cv2
import numpy as np
from djitellopy import Tello

tello = Tello()
tello.connect()
tello.streamon()

try:
    while True:
        frame = tello.get_frame_read().frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
                err_x = cX - frame.shape[1] // 2
                err_y = cY - frame.shape[0] // 2
                tello.send_rc_control(err_x//10, 0, -err_y//10, 0)
            else:
                 tello.send_rc_control(0, 0, 0, 0)
        else:
            tello.send_rc_control(0, 0, 0, 0)

        cv2.imshow("Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except Exception as e:
    print(e)
finally:
    tello.streamoff()
    tello.land()
    tello.disconnect()
    cv2.destroyAllWindows()