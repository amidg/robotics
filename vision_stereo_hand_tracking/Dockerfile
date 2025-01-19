FROM ghcr.io/amidg/ros2_cv:latest AS dev
# setup env variables used by ROS2
ENV GUSEVTECH_PATH=/opt/gusevtech
ENV COLCON_BUILD_PATH=$GUSEVTECH_PATH/build
ENV COLCON_INSTALL_PATH=$GUSEVTECH_PATH/install
ENV COLCON_LOG_PATH=$GUSEVTECH_PATH/log
RUN mkdir -p $GUSEVTECH_PATH

RUN python3 -m pip install --break-system-packages \
    #--upgrade --upgrade-strategy only-if-needed \
    mediapipe empy PyYAML

# create virtual environment
#ENV VIRTUAL_ENV=/opt/venv
#RUN python3 -m venv $VIRTUAL_ENV --system-site-packages --symlinks
#RUN touch $VIRTUAL_ENV/COLCON_IGNORE
#
## install mediapipe
#ENV PATH="$VIRTUAL_ENV/bin:$PATH"
#RUN python3 -m pip install numpy mediapipe empy
#    #python3 -m pip install tensorflow[and-cuda] mediapipe empy
#RUN echo "source $VIRTUAL_ENV/bin/activate" >> ~/.bashrc


FROM dev AS deploy
WORKDIR /app
RUN rm *
COPY .. .
RUN colcon build --build-base=$COLCON_BUILD_PATH --install-base=$COLCON_INSTALL_PATH --packages-select msg_hands vision_stereo_hand_tracking
