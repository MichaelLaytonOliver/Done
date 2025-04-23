import cv2
import time

# Common IP stream URL patterns for drones / IP cameras
common_streams = [
    "tcp://192.168.10.1:8080"           # Rarely used
]

def try_stream(url):
    print(f"Trying: {url}")
    cap = cv2.VideoCapture(url)
    time.sleep(2)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"[SUCCESS] Stream found at: {url}")
            cv2.imshow("Live Stream", frame)
            cv2.waitKey(5000)
            cv2.destroyAllWindows()
            cap.release()
            return True
    cap.release()
    return False

print("[INFO] Starting stream scan...")
for stream_url in common_streams:
    try:
        if try_stream(stream_url):
            break
    except Exception as e:
        print(f"[ERROR] {stream_url}: {e}")

print("[INFO] Scan complete.")
