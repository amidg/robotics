import cv2
import mediapipe as mp
import time
import threading
import multiprocessing

# use threading instead of multiprocessing for IO bound tasks
class HandTracking(multiprocessing.Process):
    def __init__(self, camera, name, tracking=True):
        super().__init__()
        self.camera = camera
        self.name = name
        self.tracking = tracking

    def run(self):
        if self.tracking:
            self.run_tracking()
        else:
            self.run_preview()

    def run_tracking(self):
        # camera setting
        cap = cv2.VideoCapture(self.camera, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FPS, 30)

        # mediapipe
        mpHands = mp.solutions.hands
        hands = mpHands.Hands()
        mpDraw = mp.solutions.drawing_utils

        while not cap.isOpened():
            print("waiting for " + self.name)

        pTime = 0
        cTime = 0

        while True:
            success, img = cap.read()
            img = cv2.flip(img, 1)
            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = hands.process(imgRGB)

            if results.multi_hand_landmarks:
                for handLms in results.multi_hand_landmarks:
                    for id, lm in enumerate(handLms.landmark):
                        h, w, c = img.shape
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        print(id, cx, cy)
                        if id == 4 :
                            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
                        if id == 8 :
                            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
                        if id == 12 :
                            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
                        if id == 16 :
                            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
                        if id == 20 :
                            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

                    mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime

            cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,
                        (255, 0, 255), 3)
        
            cv2.imshow("Image - "+self.name, img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()

    def run_preview(self):
        cap = cv2.VideoCapture(self.camera, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)

        while not cap.isOpened():
            print("waiting for " + self.name)

        while True:
            ret, frame = cap.read()
            if not ret:
                print(self.name + " is not working")
                break
            cv2.imshow(self.name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    camera1 = HandTracking(1, "Camera 0", True)
    camera2 = HandTracking(3, "Camera 1", True)
    camera1.start()
    camera2.start()
