from djitellopy import Tello
import time
import threading
import cv2
 
 
tello = Tello()

# Connect to the drone



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

time.sleep(1)
tello.streamon()
time.sleep(2) 


t1 = threading.Thread(target=show_camera_frames)


t1.start()
tello.takeoff()

time.sleep(5)

tello.move_up(100)

time.sleep(10)

tello.flip_left()

time.sleep(5)

tello.end() 