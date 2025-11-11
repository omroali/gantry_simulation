FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo git build-essential \
    python3 python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep python3-rosdep-modules && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y sudo git \
    && addgroup --gid ${GID} ${USERNAME} \
    && adduser  --uid ${UID} --gid ${GID} --disabled-password --gecos "" ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && addgroup dialout || true && usermod -a -G dialout ${USERNAME}

ENV BASE_WS=/home/${USERNAME}/catkin_ws
RUN mkdir -p ${BASE_WS}/src \
    && chown -R ${USERNAME}:${USERNAME} ${BASE_WS}

ENV BASH_SCRIPTS=/home/${USERNAME}/bash_scripts
RUN mkdir -p ${BASH_SCRIPTS} \
    && chown -R ${USERNAME}:${USERNAME} ${BASH_SCRIPTS}

# Convenience CLI tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    vim nano htop tmux net-tools tree && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-xfixes0 \
    libxcb-shape0 \
    libxcb-cursor0 \
    libgl1-mesa-glx \
    libegl1 \
    libxcb-xkb1 \
    libfontconfig1 \
    libfreetype6 \
    xterm \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade \
    pip wheel \
    setuptools==58.2.0 \
    packaging==24.2 \
    numpy==1.23.5 \
    scipy \
    matplotlib \
    pyproj \
    pygame \
    scikit-learn \
    opencv-python \
    pyserial \
    tmule==1.5.9

USER ${USERNAME}
WORKDIR /home/${USERNAME}
RUN sed -i 's/^#\(force_color_prompt\)/\1/' /home/${USERNAME}/.bashrc
ENTRYPOINT ["/home/ros/bash_scripts/container/entry.sh"]
CMD ["bin/bash", ]
