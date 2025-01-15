FROM ghcr.io/amidg/ros2_cv:latest

# create virtual environment
ENV VIRTUAL_ENV=/opt/tf
RUN python3 -m venv $VIRTUAL_ENV

# install mediapipe
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN python3 -m pip install -U pip && \
    python3 -m pip install -U tensorflow[and-cuda] mediapipe

# make sure colcon ignores
RUN touch ./venv/COLCON_IGNORE
