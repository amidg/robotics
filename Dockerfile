FROM ubuntu:24.04 AS base
SHELL ["/bin/bash", "-e", "-c"]

# configuration
ENV TZ="America/Vancouver"
ENV LANG="en_US.UTF-8"

# Install Core Tools
# NOTE: debian packaged python packages are installed as a part of PEP668
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update -qqy && \
    apt upgrade -qy && \
    apt install -qy --no-install-recommends \
    sudo \
    apt-utils \
    build-essential \
    software-properties-common \
    python3-full \
    python3-setuptools \
    python3-wheel \
    python3-pip \
    iputils-ping \
    curl \
    gnupg \
    apt-transport-https \
    ca-certificates \
    nodejs \
    npm \
    wget \
    git \
    vim \
    dh-make \
    file \
    gdb \
    ruby \
    jq

# Install locales
RUN apt-get install -qqy --no-install-recommends locales && \
    ln -snf "/usr/share/zoneinfo/$TZ" /etc/localtime && \
    echo "$TZ" >/etc/timezone && \
    echo "en_US.UTF-8 UTF-8" >/etc/locale.gen && \
    locale-gen && \
    update-locale LANG=${LANG} LC_ALL=${LANG} LANGUAGE=

# Create a non-root user in according to
# Least Privilege Principle
# new user has root access
ENV USERNAME=user \
    USER_UID=1001
ENV USER_GID=${USER_UID} \
    HOME=/home/${USERNAME}
ENV BASHRC=${HOME}/.bashrc

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL >/etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    echo "export USERNAME=${USERNAME}" >>${BASHRC}
USER ${USERNAME}:${USER_GID}

# install setup-cpp stuff
RUN sudo npm install -g setup-cpp && \
    sudo setup-cpp \
    --compiler llvm \
    --gcc true \
    --g++ true \
    --cmake true \
    --ninja true \
    --task true \
    --vcpkg true \
    --python true \
    --make true \
    --cppcheck true \
    --doxygen true \
    --ccache true \
    --gcovr true && \
    sudo npm cache clean --force

# CPM
ENV CPM_CMAKE_FILE="$HOME/cmake-modules/get_cpm.cmake"
RUN mkdir -p ~/cmake-modules/
RUN curl -LJo "$CPM_CMAKE_FILE" https://github.com/cpm-cmake/CPM.cmake/releases/latest/download/get_cpm.cmake

# clean up
RUN sudo apt clean
RUN sudo rm -rf /tmp/* || true

ENTRYPOINT ["/bin/bash", "-c", "source $HOME/.cpprc && bash"]

FROM base AS fastdds-builder
# install FastDDS
# setup versions
ARG MEMORY_VERSION=1.3.1
ARG FASTCDR_VERSION=2.2.5
ARG FASTDDS_VERSION=3.1.0
ARG FASTDDSGEN_VERSION=4.0.2

ENV MEMORY_VERSION=${MEMORY_VERSION}
ENV FASTCDR_VERSION=${FASTCDR_VERSION}
ENV FASTDDS_VERSION=${FASTDDS_VERSION}
ENV FASTDDSGEN_VERSION=${FASTDDSGEN_VERSION}

# install eProsima Dependencies
# most of the C++ depencies are installed as a part of controls_base
# this step installs FastDDS dependencies and useful tools
RUN sudo apt update -qqy && \
    sudo apt install -qqy \
    swig4.1 \
    libpython3-dev \
    libasio-dev \
    libtinyxml2-dev \
    libssl-dev \
    openjdk-11-jdk \
    libp11-dev \
    softhsm2 \
    libengine-pkcs11-openssl


# complile FastDDS
FROM base AS fastdds-builder
# install FastDDS globally
# foonathan memory
RUN mkdir -p /tmp/fastdds/install
WORKDIR /tmp/fastdds
RUN git clone --depth 1 --branch v$MEMORY_VERSION \
    https://github.com/eProsima/foonathan_memory_vendor.git && \
    mkdir foonathan_memory_vendor/build && \
    cd foonathan_memory_vendor/build && \
    source $HOME/.cpprc && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/tmp/fastdds/install -DBUILD_SHARED_LIBS=ON && \
    cmake --build . --target install

# Fast CDR
RUN git clone --depth 1 --branch v$FASTCDR_VERSION \
    https://github.com/eProsima/Fast-CDR.git && \
    mkdir Fast-CDR/build && \
    cd Fast-CDR/build && \
    source $HOME/.cpprc && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/tmp/fastdds/install && \
    cmake --build . --target install

# Fast DDS
RUN git clone --depth 1 --branch v$FASTDDS_VERSION \
    https://github.com/eProsima/Fast-DDS.git && \
    mkdir Fast-DDS/build && \
    cd Fast-DDS/build && \
    source $HOME/.cpprc && \
    cmake ..  -DCMAKE_INSTALL_PREFIX=/tmp/fastdds/install && \
    cmake --build . --target install

# FastDDS Gen
RUN git clone --recursive --depth 1 --branch v$FASTDDSGEN_VERSION \
    https://github.com/eProsima/Fast-DDS-Gen.git fastddsgen && \
    cd fastddsgen && \
    ./gradlew assemble


# create final that contains only pre-compiled fastdds
FROM base AS robotics_base

# install libraries
COPY --from=fastdds-builder \
    /tmp/fastdds/install/ \
    /usr/local/
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc

# install fastddsgen
COPY --from=fastdds-builder \
    /tmp/fastdds/fastddsgen/share/fastddsgen/java/fastddsgen.jar \
    /usr/local/bin/
RUN echo 'alias fastddsgen="java -jar /usr/local/bin/fastddsgen.jar"' >> ~/.bashrc
