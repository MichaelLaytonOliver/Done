import numpy as np

class DroneInterface:
    def connect(self): pass
    def get_battery(self): return 100
    def streamon(self): pass
    def takeoff(self): pass
    def land(self): pass
    def move_forward(self, cm): pass
    def flip_back(self): pass
    def get_height(self): return 100
    def send_rc_control(self, lr, fb, ud, yaw): pass
    def get_frame(self): return None

class TelloAdapter(DroneInterface):
    def __init__(self):
        from djitellopy import Tello
        self.drone = Tello()
        self.cap = None

    def connect(self):
        self.drone.connect()

    def get_battery(self):
        return self.drone.get_battery()

    def streamon(self):
        self.drone.streamon()
        self.cap = self.drone.get_frame_read()

    def takeoff(self):
        self.drone.takeoff()

    def land(self):
        self.drone.land()

    def move_forward(self, cm):
        self.drone.move_forward(cm)

    def flip_back(self):
        self.drone.flip_back()

    def get_height(self):
        return self.drone.get_height()

    def send_rc_control(self, lr, fb, ud, yaw):
        self.drone.send_rc_control(lr, fb, ud, yaw)

    def get_frame(self):
        return self.cap.frame if self.cap else None

class DummyAdapter(DroneInterface):
    def connect(self): print("[DUMMY] Connected.")
    def get_battery(self): return 75
    def streamon(self): pass
    def takeoff(self): print("[DUMMY] Takeoff.")
    def land(self): print("[DUMMY] Land.")
    def move_forward(self, cm): print(f"[DUMMY] Move forward {cm}cm.")
    def flip_back(self): print("[DUMMY] Flip back.")
    def get_height(self): return 120
    def send_rc_control(self, lr, fb, ud, yaw): pass
    def get_frame(self): return np.zeros((480, 640, 3), dtype=np.uint8)
