# Use this for depth pro
podman-run-gpu -it --rm -v /dev:/dev -v ~/Pictures/DepthPro/:/pics:Z -v /var/home/dmitrii/Projects/Robotics/robotics/vision_depth_pro:/app:Z -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY --security-opt label=type:container_runtime_t vision_depth_pro:latest

# Basic gpu container for cameras
podman-run-gpu -it --rm -v /dev:/dev -v /var/home/dmitrii/Projects/Robotics/robotics/vision_depth_pro:/app:Z -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY --security-opt label=type:container_runtime_t vision_depth_pro:latest
