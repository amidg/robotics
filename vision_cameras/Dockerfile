FROM ghcr.io/amidg/ros2_cv:latest AS dev

# install python3 packages needed by colcon
RUN python3 -m pip install --break-system-packages \
    #--upgrade --upgrade-strategy only-if-needed \
    empy PyYAML

# add colcon build command
RUN echo 'alias cb="colcon build --install-base=$COLCON_INSTALL_PATH --build-base=$COLCON_BUILD_PATH"' >> ~/.bashrc

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
