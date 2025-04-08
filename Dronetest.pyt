from djitellopy import Tello
import time
import cv2

# Initialize and connect to the Tello drone
tello = Tello()
tello.connect()
print(f"Battery: {tello.get_battery()}%")

# Start video stream
tello.streamon()
frame_read = tello.get_frame_read()

try:
    # Takeoff
    tello.takeoff()

    start_time = time.time()

    while True:
        frame = frame_read.frame
        if frame is not None:
            frame = cv2.resize(frame, (840, 620))
            cv2.imshow("Tello Camera Feed", frame)

        # Break loop after 15 seconds or if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if time.time() - start_time > 15:
            break

    # Land after loop
    tello.land()

finally:
    # Cleanup
    tello.streamoff()
    cv2.destroyAllWindows()
