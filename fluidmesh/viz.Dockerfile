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
RUN apt install -y gcc g++ build-essential git

# https://github.com/eclipse-zenoh/zenoh-plugin-dds
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN apt install -y llvm-dev libclang-dev
RUN git clone https://github.com/eclipse-zenoh/zenoh-plugin-dds
RUN cd zenoh-plugin-dds && /root/.cargo/bin/cargo build --release
RUN cp zenoh-plugin-dds/target/release/dzd /usr/bin

# ADD Executables / config
RUN mkdir /etc/cyclone
ADD fm/cyclonedds.xml /etc/cyclone/cyclonedds.xml
ADD fm/adlink_fm.sh /usr/local/bin
ADD fm/bst_fm.sh /usr/local/bin
