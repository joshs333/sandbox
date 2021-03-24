FROM ubuntu:18.04

RUN DEBIAN_FRONTEND=noninteractive && apt update
RUN DEBIAN_FRONTEND=noninteractive && apt-get install -y libboost-all-dev cmake libtbb-dev software-properties-common
RUN DEBIAN_FRONTEND=noninteractive && apt update

RUN DEBIAN_FRONTEND=noninteractive && add-apt-repository ppa:borglab/gtsam-develop
RUN DEBIAN_FRONTEND=noninteractive && apt update
RUN DEBIAN_FRONTEND=noninteractive && apt install -y libgtsam-dev libgtsam-unstable-dev