import cv2
import numpy as np
import threading
from vision_cameras.camera import Camera

class StereoCamera(threading.Thread):
    def __init__(
        self,
        device1,
        device2,
        name="",
        res=[1920,1080],
        fps=30,
        crop=[1080,1080]
    ):
        super().__init__()
        # properties
        self.name = name
        self.resolution = res
        self.fps = fps
        self.crop = crop

        # setup cameras
        self.cam1 = Camera(
            device1,
            f"{self.name}1",
            self.resolution,
            self.fps,
            self.crop
        )

        self.cam2 = Camera(
            device2,
            f"{self.name}2",
            self.resolution,
            self.fps,
            self.crop
        )

        self.stop_capture = threading.Event()

    def get_resolution(self):
        self.cam1.get_resolution()

    def get_fps(self):
        return self.fps

    def get_frame(self, cam):
        if cam == 1:
            return self.cam1.get_frame()
        elif cam == 2:
            return self.cam2.get_frame()

    def stop(self):
        self.stop_capture.set()

    def run(self):
        self.cam1.start()
        self.cam2.start()
        while not self.stop_capture.is_set():
            pass
        self.cam1.stop()
        self.cam2.stop()


if __name__ == '__main__':
    stereo = StereoCamera(3, 1, "stereo_example")
    stereo.start()

    # example 2
    frame = np.ndarray(stereo.get_resolution())
    while True:
        cv2.imshow("camera 1", stereo.get_frame(1))
        cv2.imshow("camera 2", stereo.get_frame(2))
        if cv2.waitKey(1) & 0xFF == 27:
            stereo.stop()
            break #27 is ESC key.

    cv2.destroyAllWindows()
    stereo.join()
