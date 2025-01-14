# Import the necessary modules.
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2
import time
import numpy as np
from mediapipe.framework.formats import landmark_pb2
from mediapipe import solutions

# Constants
MARGIN = 10
FONT_SIZE = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54)
FONT_THICKNESS = 1

# Function to draw landmarks on the image
def draw_landmarks_on_image(rgb_image, detection_result, fps):
    hand_landmarks_list = detection_result.hand_landmarks
    handedness_list = detection_result.handedness
    annotated_image = np.copy(rgb_image)
    
    # Loop through the detected hands to visualize
    for idx in range(len(hand_landmarks_list)):
        hand_landmarks = hand_landmarks_list[idx]
        handedness = handedness_list[idx]

        # Draw the hand landmarks
        hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z)
            for landmark in hand_landmarks
        ])
        solutions.drawing_utils.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            solutions.hands.HAND_CONNECTIONS,
            solutions.drawing_styles.get_default_hand_landmarks_style(),
            solutions.drawing_styles.get_default_hand_connections_style()
        )

        # Get the top left corner of the detected hand's bounding box
        height, width, _ = annotated_image.shape
        x_coordinates = [landmark.x for landmark in hand_landmarks]
        y_coordinates = [landmark.y for landmark in hand_landmarks]
        text_x = int(min(x_coordinates) * width)
        text_y = int(min(y_coordinates) * height) - MARGIN

        # Draw handedness (left or right hand) on the image
        cv2.putText(annotated_image, f"{handedness[0].category_name}",
                    (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                    FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
        cv2.putText(annotated_image, "FPS:" + str(int(fps)),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (255, 0, 0), 2, cv2.LINE_AA)

    return annotated_image

# Initialize HandLandmarker
model_path = 'hand_landmarker.task'
cap = cv2.VideoCapture("/dev/video1")  # Capture from /dev/video1 camera

base_options = mp.tasks.BaseOptions(model_asset_path=model_path,
                                    delegate=mp.tasks.BaseOptions.Delegate.GPU)
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=mp.tasks.vision.RunningMode.VIDEO,
    num_hands=2
)
detector = vision.HandLandmarker.create_from_options(options)

# Video processing loop
while True:
    try:
        ret, frame = cap.read()
    except KeyboardInterrupt:
        break

    if not ret:
        break

    # Prepare the frame for hand landmark detection
    mp_image = mp.Image(
        image_format=mp.ImageFormat.SRGB,
        data=np.array(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    )
    timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)

    # Detect hand landmarks
    start_time = time.time()
    try:
        detection_result = detector.detect_for_video(mp_image, int(timestamp))
    except KeyboardInterrupt:
        break
    end_time = time.time()
    fps = 1 / (end_time - start_time)

    # Annotate the image with landmarks and display
    annotated_image = draw_landmarks_on_image(mp_image.numpy_view(), detection_result, fps)
    cv2.imshow("frame", cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
    
    # Exit on pressing 'Esc'
    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()

