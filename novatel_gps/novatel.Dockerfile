FROM ros:foxy-ros-core

# https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
RUN apt update || echo hi
RUN apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install colcon
RUN apt update \
    && apt install -y *colcon*
    # unbelievable that these aren't also installed...
RUN apt install -y gcc g++ build-essential
RUN apt install -y ros-foxy-novatel-gps-driver