# Build this container
podman build -t vision_hand_tracking .

# Basic gpu container for cameras
podman-run-gpu -it --rm $CONTAINER_GUI -v /dev:/dev -v $(pwd):/app:z vision_hand_tracking:latest
