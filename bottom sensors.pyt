from djitellopy import Tello
import cv2
import numpy as np
import time

def detect_dominant_color(image):
    image = cv2.resize(image, (100, 100))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    pixels = image.reshape((-1, 3))
    unique, counts = np.unique(pixels, axis=0, return_counts=True)
    dominant_color = unique[counts.argmax()]
    return dominant_color

def color_name(rgb):
    r, g, b = rgb
    if r > 200 and g < 80 and b < 80:
        return 'Red'
    elif r < 80 and g > 200 and b < 80:
        return 'Green'
    elif r < 80 and g < 80 and b > 200:
        return 'Blue'
    elif r > 200 and g > 200 and b < 80:
        return 'Yellow'
    elif r < 80 and g > 200 and b > 200:
        return 'Cyan'
    elif r > 200 and g < 80 and b > 200:
        return 'Magenta'
    elif r > 230 and g > 130 and g < 180 and b < 80:
        return 'Orange'
    elif r > 150 and g < 80 and b > 150:
        return 'Purple'
    elif r > 230 and g > 180 and b > 200:
        return 'Pink'
    elif r > 200 and g > 200 and b > 200:
        return 'White'
    elif 80 < r < 180 and 80 < g < 180 and 80 < b < 180:
        return 'Gray'
    elif r < 50 and g < 50 and b < 50:
        return 'Black'
    else:
        return f'RGB({r},{g},{b})'

def main():
    tello = Tello()
    tello.connect()
    print("Battery:", tello.get_battery())

    tello.streamon()
    time.sleep(2)

    tello.takeoff()
    tello.move_up(60)

    frame_read = tello.get_frame_read()

    try:
        print("Press 'q' to quit.")
        while True:
            frame = frame_read.frame

            # Simulate bottom camera by using lower center region of the front camera
            h, w, _ = frame.shape
            crop_top = int(0.6 * h)
            crop_left = int(0.4 * w)
            crop_right = int(0.6 * w)
            cropped = frame[crop_top:h, crop_left:crop_right]

            # Detect dominant color
            dominant = detect_dominant_color(cropped)
            color = color_name(dominant)

            # Get sensor data
            tof = tello.get_distance_tof()
            height = tello.get_height()

            # Draw overlays
            cv2.putText(frame, f"Detected: {color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"TOF: {tof}cm", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, f"Height: {height}cm", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # Rectangle over analyzed region
            cv2.rectangle(frame, (crop_left, crop_top), (crop_right, h), (255, 0, 0), 2)

            # Show video
            cv2.imshow("Tello Camera Feed (Simulated Bottom View)", frame)

            # Save detected color
            with open("detected_color.txt", "w") as file:
                file.write(f"Detected Color: {color}")

            # Quit with 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        tello.land()
        tello.streamoff()
        tello.end()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
