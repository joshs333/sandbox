FROM ubuntu:18.04

RUN apt update
RUN apt-get install -y libboost-all-dev cmake libtbb-dev software-properties-common
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y libeigen3-dev python-dev python-numpy python-matplotlib
RUN apt install -y libgtest-dev google-mock

RUN cd /usr/src/gtest && cmake CMakeLists.txt && make && cp *.a /usr/lib
RUN cd /usr/src/googletest/googlemock && cmake CMakeLists.txt && make && cp *.a /usr/lib