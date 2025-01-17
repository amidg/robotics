# Build this container
podman build -t vision_stereo_hand_tracking .

# Basic CPU only container
podman run -it --rm $CONTAINER_GUI -v /dev:/dev -v $(pwd):/app:z vision_stereo_hand_tracking:latest

# Basic gpu container for cameras
podman-run-gpu -it --rm $CONTAINER_GUI -v /dev:/dev -v $(pwd):/app:z vision_stereo_hand_tracking:latest
