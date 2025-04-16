from djitellopy import Tello
import cv2
import numpy as np

tello = Tello()
tello.connect()
tello.streamon()

prev_frame = None

try:
    while True:
        frame = tello.get_frame_read().frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if prev_frame is not None:
            # Calculate dense optical flow
            flow = cv2.calcOpticalFlowFarneback(prev_frame, gray,
                                                None,
                                                0.5, 3, 15, 3, 5, 1.2, 0)
            # Visualize flow
            magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            mask = np.zeros_like(frame)
            mask[..., 1] = 255
            mask[..., 0] = angle * 180 / np.pi / 2
            mask[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)
            flow_visual = cv2.cvtColor(mask, cv2.COLOR_HSV2BGR)

            cv2.imshow("Optical Flow", flow_visual)

        prev_frame = gray

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

tello.streamoff()
cv2.destroyAllWindows()
