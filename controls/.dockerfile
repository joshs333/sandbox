FROM ubuntu:18.04

RUN apt update
RUN apt-get install -y libboost-all-dev cmake libtbb-dev software-properties-common
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y libeigen3-dev python-dev python-numpy python-matplotlib