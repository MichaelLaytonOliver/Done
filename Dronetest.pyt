from djitellopy import Tello
import time
import threading
import cv2

# Initialize the Tello drone
tello = Tello()

# Connect to the drone
lower_blue = (100, 50, 50)
upper_blue = (130, 255, 255)
frame_read = tello.get_frame_read().frame

def show_camera_frames():
    # Shows the camera frames through cv2
    while True:
        frame_read = tello.get_frame_read().frame
        cv2.imshow("frame", frame_read)
        # While it is running, cv2 gets the current frame on the camera and outputs it to a window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    
tello.connect()
tello.takeoff()
time.sleep(1)
tello.streamon()
time.sleep(2) 


t1 = threading.Thread(target=show_camera_frames)


t1.start()

tello.set_speed()
time.sleep(5)


mask = cv2.inRange(hsv, lower_blue, upper_blue)
result = cv2.bitwise_and(frame_read, frame_read, mask=mask)

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(frame_read, center, radius, (0, 255, 0), 2)

cv2.imshow("Color Detection", frame_read)

tello.end() 
