FROM ghcr.io/amidg/ros2_cv:latest

RUN python3 -m pip install --break-system-packages mediapipe empy

# create virtual environment
#ENV VIRTUAL_ENV=/opt/tf
#RUN python3 -m venv $VIRTUAL_ENV --system-site-packages --symlinks

# install mediapipe
#ENV PATH="$VIRTUAL_ENV/bin:$PATH"
#RUN python3 -m pip install -U pip && \
#    python3 -m pip install -U numpy mediapipe empy
    #python3 -m pip install -U tensorflow[and-cuda] mediapipe empy

# make sure colcon ignores
#RUN echo "source /opt/tf/bin/activate" >> ~/.bashrc
#RUN touch /opt/tf/COLCON_IGNORE
