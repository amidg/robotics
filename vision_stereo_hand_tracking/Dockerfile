FROM ghcr.io/amidg/opencv_gpu:latest

# create virtual environment
ENV VIRTUAL_ENV=/opt/tf
RUN python3 -m venv $VIRTUAL_ENV

# install mediapipe
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN python3 -m pip install -U pip && \
    python3 -m pip install -U tensorflow[and-cuda] mediapipe

RUN echo "source /opt/tf/bin/activate" >> ~/.bashrc
ENTRYPOINT ["bash"]
