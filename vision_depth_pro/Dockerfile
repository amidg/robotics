# default python 3.12 in ubuntu 24.04
FROM ghcr.io/amidg/opencv_gpu:latest AS base

# install depth pro
WORKDIR /app
RUN rm *
RUN apt update -qqy && apt install -qqy python3-pip
RUN git clone https://github.com/apple/ml-depth-pro.git && \
    cd ml-depth-pro && \
    python3 -m pip install --break-system-packages -e .
WORKDIR /app/ml-depth-pro
RUN /bin/bash -c "source get_pretrained_models.sh"

# install mediapipe
RUN python3 -m pip install --break-system-packages mediapipe
