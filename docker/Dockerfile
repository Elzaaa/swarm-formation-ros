ARG from=ubuntu:22.04

FROM ${from}

LABEL maintainer="rubenandres.contreras25@gmail.com"

# Timezone Configuration
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive

# Local install
RUN apt update && apt install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8


# Install dependencies
RUN apt-get update && apt-get install -y apt-utils\
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
    python3 \
    python3-pip \
    gcc \
    cmake \
    libssl-dev \
    libncurses5-dev \
    libsqlite3-dev \
    libreadline-dev \
    libffi-dev \
    uuid-dev \
    zlib1g-dev \
    libbullet-dev 

# Install ROS 2 Humble Desktop and Packages
RUN echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list 
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
    
RUN apt-get update && apt-get install -y ros-humble-desktop \
    ros-humble-ros-base \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-eigen-stl-containers \
    ros-humble-random-numbers \
    ros-humble-moveit-common \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    python3-argcomplete \
    python3-pytest-cov \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-colcon-mixin \
    python3-colcon-common-extensions 

# Configure colcon mixin
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && colcon mixin update default

# Installing Python3 components 
RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \ 
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest 

# Initialize rosdep
RUN rosdep init && rosdep update

#----------------------------------------------------------------------------------
# Create workspace and download MoveIt 2 source code
RUN mkdir -p ~/humble_ws/src && cd ~/humble_ws/src && \
    git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
#     git clone -b humble https://github.com/moveit/moveit2_tutorials && \
#     vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos

# Install dependencies for workspace
RUN cd ~/humble_ws && \
    sudo apt update && \
    rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

# Build the Colcon Workspace with proper sourcing
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ~/humble_ws && \
    colcon build --mixin release --executor sequential"

# Source ROS 2 and Colcon Workspace on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/humble_ws/install/setup.bash" >> ~/.bashrc && \
    echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc && \
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Entry point: Open a bash shell with sourced ROS environment
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source ~/humble_ws/install/setup.bash && exec bash"]

