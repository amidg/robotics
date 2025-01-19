## Node architecture:
camera.py -> single camera class similar to openreality project
stereo_camera.py -> stereo camera class
calibration.py -> CV calibration functions
utils.py -> utility functions for YAML read/write and math
hand_tracking.py -> mediapipe based hand tracking
main.py -> main ROS2 node


## Run this node:
### Build this container
podman build -t vision_stereo_hand_tracking .

### Basic CPU only container:
podman run -it --rm $CONTAINER_GUI -v /dev:/dev -v $(pwd):/app:z vision_stereo_hand_tracking:latest

### Basic gpu container for cameras
podman-run-gpu -it --rm $CONTAINER_GUI -v /dev:/dev -v $(pwd):/app:z vision_stereo_hand_tracking:latest
