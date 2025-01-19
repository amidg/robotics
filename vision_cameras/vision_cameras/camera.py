import cv2
import numpy as np
import threading

class Camera(threading.Thread):
    def __init__(
        self,
        device,
        name="",
        res=[1920,1080],
        fps=30,
        crop=[1080,1080]
    ):
        super().__init__()
        self.camera = device
        self.name = name
        self.resolution = res
        self.fps = fps
        self.crop = crop
        self.lock = threading.Lock()

        self.stop_capture = threading.Event()
        self.frame = np.ndarray(self.get_resolution())

    def get_resolution(self):
        if self.crop != []:
            return (self.crop[0], self.crop[1], 3)
        else:
            return (self.resolution[0], self.resolution[1], 3)

    def get_fps(self):
        return self.fps

    def get_frame(self):
        with self.lock:
            return self.frame

    def stop(self):
        self.stop_capture.set()

    def run(self):
        # set camera properties
        cap = cv2.VideoCapture(self.camera, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        # wait for camera to become available
        while not cap.isOpened():
            print(f"waiting for camera {self.name}")

        # read frames
        while not self.stop_capture.is_set():
            # read frames from stream
            ret, frame = cap.read()

            if not ret:
                raise RuntimeError(f"Camera {self.name} not working")

            # crop to square
            # Note: camera calibration parameters are set to this resolution.If you change this, make sure to also change camera intrinsic parameters
            if self.crop != [] and frame.shape[1] != self.resolution[1]: # shape[1] is width, self.resoution[1] is height
                frame = frame[:,self.resolution[0]//2 - self.resolution[1]//2:self.resolution[0]//2 + self.resolution[1]//2]

            # write frame
            with self.lock:
                self.frame = frame

        # stop event called 
        cap.release()

if __name__ == '__main__':
    # example 1
    #cam = Camera(1, "example 1")
    #cam.start()
    #cam.join()

    # example 2
    cam = Camera(3, "example 2")
    cam.start()
    frame = np.ndarray(cam.get_resolution())
    while True:
        frame = cam.get_frame()
        cv2.imshow("example", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            cam.stop()
            break #27 is ESC key.

    cv2.destroyAllWindows()
    cam.join()
