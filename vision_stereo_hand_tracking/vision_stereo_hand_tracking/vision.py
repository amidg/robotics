import cv2
import mediapipe as mp
import numpy as np
import multiprocessing
from multiprocessing.shared_memory import SharedMemory
from vision_stereo_hand_tracking.utils import DLT, get_projection_matrix

class HandTracking(multiprocessing.Process):
    def __init__(self, camera0, camera1, config_path):
        super().__init__()
        # camera parameters
        self.cam0 = camera0
        self.cam1 = camera1
        self.resolution = [1080, 1920] # opencv convention
        self.P0 = get_projection_matrix(0, config_path)
        self.P1 = get_projection_matrix(1, config_path)

        # data points
        self.queue = multiprocessing.Queue()
        self.kpts_cam0 = []
        self.kpts_cam1 = []

    def get_data(self, shm_name):
        shm = SharedMemory(name=shm_name)
        data = np.ndarray((21, 3), dtype=np.float32, buffer=shm.buf)
        if not self.queue.empty():
            np.copyto(data, self.queue.get())
            return True
        return False

    def run(self):
        # mediapipe stuff
        mp_hands = mp.solutions.hands
        mp_drawing = mp.solutions.drawing_utils
        hands0 = mp_hands.Hands(min_detection_confidence=0.5, max_num_hands=1, min_tracking_confidence=0.5)
        hands1 = mp_hands.Hands(min_detection_confidence=0.5, max_num_hands=1, min_tracking_confidence=0.5)

        # set camera 0
        cap0 = cv2.VideoCapture(self.cam0, cv2.CAP_V4L2)
        cap0.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap0.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[1])
        cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[0])
        cap0.set(cv2.CAP_PROP_FPS, 30)

        # set camera 1
        cap1 = cv2.VideoCapture(self.cam1, cv2.CAP_V4L2)
        cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[1])
        cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[0])
        cap1.set(cv2.CAP_PROP_FPS, 30)

        # wait for cameras
        while not (cap0.isOpened() and cap1.isOpened()):
            if not cap0.isOpened():
                print("waiting for camera 0")
            if not cap1.isOpened():
                print("waiting for camera 1")

        # read frames
        while True:
            #read frames from stream
            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()

            # print error message
            if not ret0 or not ret1:
                print("Failed to acquire frame, retrying")
                continue

            #crop to 1080x1080.
            #Note: camera calibration parameters are set to this resolution.If you change this, make sure to also change camera intrinsic parameters
            if frame0.shape[1] != self.resolution[0]:
                frame0 = frame0[:,self.resolution[1]//2 - self.resolution[0]//2:self.resolution[1]//2 + self.resolution[0]//2]
                frame1 = frame1[:,self.resolution[1]//2 - self.resolution[0]//2:self.resolution[1]//2 + self.resolution[0]//2]

            # the BGR image to RGB.
            frame0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2RGB)
            frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            #frame0.flags.writeable = False
            #frame1.flags.writeable = False
            results0 = hands0.process(frame0)
            results1 = hands1.process(frame1)

            #prepare list of hand keypoints of this frame
            #frame0 kpts
            frame0_keypoints = []
            if results0.multi_hand_landmarks:
                for hand_landmarks in results0.multi_hand_landmarks:
                    for p in range(21):
                        #print(p, ':', hand_landmarks.landmark[p].x, hand_landmarks.landmark[p].y)
                        pxl_x = int(round(frame0.shape[1]*hand_landmarks.landmark[p].x))
                        pxl_y = int(round(frame0.shape[0]*hand_landmarks.landmark[p].y))
                        kpts = [pxl_x, pxl_y]
                        frame0_keypoints.append(kpts)
            # if no keypoints are found
            else:
                # simply fill the frame data with [-1,-1] for each kpt
                frame0_keypoints = [[-1, -1]]*2

            # update keypoints container
            self.kpts_cam0.append(frame0_keypoints)

            #frame1 kpts
            frame1_keypoints = []
            if results1.multi_hand_landmarks:
                for hand_landmarks in results1.multi_hand_landmarks:
                    for p in range(21):
                        #print(p, ':', hand_landmarks.landmark[p].x, hand_landmarks.landmark[p].y)
                        pxl_x = int(round(frame1.shape[1]*hand_landmarks.landmark[p].x))
                        pxl_y = int(round(frame1.shape[0]*hand_landmarks.landmark[p].y))
                        kpts = [pxl_x, pxl_y]
                        frame1_keypoints.append(kpts)
            else:
                # if no keypoints are found
                # simply fill the frame data with [-1,-1] for each kpt
                frame1_keypoints = [[-1, -1]]*21
            #update keypoints container
            self.kpts_cam1.append(frame1_keypoints)

            #calculate 3d position
            frame_p3ds = []
            for uv1, uv2 in zip(frame0_keypoints, frame1_keypoints):
                if uv1[0] == -1 or uv2[0] == -1:
                    _p3d = [-1, -1, -1]
                else:
                    _p3d = DLT(self.P0, self.P1, uv1, uv2) #calculate 3d position of keypoint
                frame_p3ds.append(_p3d)

            '''
            This contains the 3d position of each keypoint in current frame.
            For real time application, this is what you want.
            Then place data into the shared memory object
            '''
            try:
                frame_p3ds = np.array(frame_p3ds).reshape((21, 3))
                self.queue.put(frame_p3ds)
            except ValueError:
                continue

            # Draw the hand annotations on the image.
            frame0.flags.writeable = True
            frame1.flags.writeable = True
            frame0 = cv2.cvtColor(frame0, cv2.COLOR_RGB2BGR)
            frame1 = cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR)

            if results0.multi_hand_landmarks:
                for hand_landmarks in results0.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        frame0,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS
                    )

            if results1.multi_hand_landmarks:
                for hand_landmarks in results1.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        frame1,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS
                    )
            cv2.imshow('cam1', frame1)
            cv2.imshow('cam0', frame0)

            if cv2.waitKey(1) & 0xFF == 27:
                break #27 is ESC key.

        cv2.destroyAllWindows()
        cap0.release()
        cap1.release()

if __name__ == '__main__':
    import time
    hand_tracker = HandTracking(3, 1, "../")
    hand_tracker.start()

    # print data
    fingers = np.empty((21, 3))
    for _ in range(10):
        fingers = hand_tracker.get_data()
        point = 0
        for value in fingers:
            print(f"Point: {point}")
            print(f"X: {value[0]}")
            print(f"Y: {value[1]}")
            print(f"Z: {value[2]}")
            point = point + 1
        time.sleep(1)

    # finish thread
    hand_tracker.join()
