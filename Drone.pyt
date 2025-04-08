from djitellopy import Tello
import time
import cv2
import threading



def show_camera_frames():
    # Shows the camera frames through cv2
    while True:
        frame = tello.get_frame_read().frame
        cv2.imshow("Frame", frame)
        # While it is running, cv2 gets the current frame on the camera and outputs it to a window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()


def main():
    # Initialize the Tello drone
    tello = Tello()

    # Connect to the drone
    tello.connect()

    tello.streamon()
    frame_read = tello.get_frame_read()
    tello.takeoff()
    t1 = threading.Thread(target=show_camera_frames, args=())

    time.sleep(15)

    tello.land()
