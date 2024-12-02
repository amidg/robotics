FROM nvidia/cuda:12.6.2-devel-ubuntu24.04 AS base

# update
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
    curl \
    gnupg \
    apt-transport-https \
    ca-certificates \
    wget \
    git \
    vim

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

# create python 3.9 container
FROM base AS python39

RUN sudo add-apt-repository ppa:deadsnakes/ppa -y && \
    sudo apt install -qy python3.9 \
    python3-pip \
    python3-setuptools \
    python3.9-distutils

FROM python39 AS depth_pro

WORKDIR $HOME
#RUN python3.9 -m pip install distutils setuptools
RUN git clone https://github.com/apple/ml-depth-pro.git && \
    cd ml-depth-pro && \
    python3.9 -m pip install -e .
WORKDIR $HOME/ml-depth-pro
RUN /bin/bash -c "source get_pretrained_models.sh"
RUN echo PATH="$HOME/.local/bin:$HOME/bin:$PATH" >> ~/.bashrc
