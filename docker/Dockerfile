ARG from

FROM ${from}

LABEL Ruben Contreras "rubenandres,contreras25@gmail.com"


# Timezone Configuration
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND = noninteractive

# Local install
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Apps install
RUN apt-get update && apt-get install -y apt-utils \
    software-properties-common \
    lsb-release \
    mesa-utils \
    gnupg2 \
    net-tools \
    build-essential \
    wget \
    unzip \
    curl \
    git \
    mc \
    vim \
    gedit \
    python2 \
    python-dev \
    python3-pip \
    gcc \
    gfortran \
    libopenblas-dev \ 
    cmake \
    ffmpeg \
    libeigen3-dev 


# Install ROS desktop and Gazebo
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y ros-noetic-desktop-full \
    ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
    ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3

# GUI tunnel from docker
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
    libxau6 libxau6:i386 \
    libxdmcp6 libxdmcp6:i386 \
    libxcb1 libxcb1:i386 \
    libxext6 libxext6:i386 \
    libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

RUN apt-get update && apt-get install -y --no-install-recommends \
    libglvnd0 libglvnd0:i386 \
    libgl1 libgl1:i386 \
    libglx0 libglx0:i386 \
    libegl1 libegl1:i386 \
    libgles2 libgles2:i386 && \
    rm -rf /var/lib/apt/lists/*

COPY docker/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
COPY docker/requirements.txt requirements.txt
RUN pip install -r requirements.txt

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64


RUN curl -s https://packagecloud.io/install/repositories/groove-x/ros/script.deb.sh | bash
RUN apt-get install ros-noetic-octomap-msgs
                 
RUN echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc 

ENV source /opt/ros/noetic/setup.bash


RUN echo 'source ~/../noetic_ws/devel/setup.bash' >> ~/.bashrc \
    && echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

