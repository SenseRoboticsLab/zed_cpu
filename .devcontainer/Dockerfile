# Use the official ROS 2 Foxy image as the base
FROM ros:foxy

# Set the timezone non-interactively to India (Asia/Kolkata)
ENV DEBIAN_FRONTEND=noninteractive

# Install ros-foxy-desktop package
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    && rm -rf /var/lib/apt/lists/*

    # Clone NLopt repository
RUN git clone https://github.com/stevengj/nlopt.git && \
    cd nlopt && \
    git checkout 09b3c2a6da71cabcb98d2c8facc6b83d2321ed71 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install


RUN apt-get update && apt-get install -y \
    ros-foxy-cv-bridge \
    ros-foxy-visualization-msgs \
    libeigen3-dev \
    libnlopt-dev \
    libpcl-dev \
    python3-argcomplete \
    ros-dev-tools \
    python3-argcomplete \
    sudo\
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-foxy-compressed-image-transport \
    ros-foxy-image-transport-plugins \
    libhidapi-dev \
    libusb-1.0-0-dev


RUN useradd -m user && echo "user:123" | chpasswd
RUN echo "user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/user
RUN chsh -s /bin/bash user
RUN usermod -s /bin/bash user
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/user/.bashrc

# (Optional) Set up your workspace directory
USER user
RUN mkdir -p /home/user/ros2_ws/src

WORKDIR /home/user/ros2_ws

# (Optional) Source your workspace setup file on container startup
CMD bash
