FROM ubuntu:18.04
LABEL maintainer="asoragna at irobot dot com"

# working directory
ENV HOME /root
WORKDIR $HOME

# general utilities
RUN apt-get update && apt-get install -y \
    curl \
    vim \
    nano

# Set timezone otherwise tz will ask for it
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata

# Locale options
RUN apt-get install -y locales
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Setup sources
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release dirmngr bash-completion
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update

# install development tools and ROS tools
RUN apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  wget


# install Fast-DDS dependencies
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# pip
RUN pip3 install --upgrade pip

# Install debug tools
RUN apt-get update && apt-get install -y \
  gdb \
  netcat \
  valgrind \
  linux-tools-common \
  linux-tools-generic

# Download perf visualizer: FlameGraph
RUN git clone https://github.com/brendangregg/FlameGraph

# Install DDS GEN dependencies
RUN apt-get install -y \
  default-jdk

# FastCDR
RUN git clone https://github.com/eProsima/Fast-CDR.git
WORKDIR Fast-CDR/build
RUN cmake ..
RUN cmake --build . --target install

# Foonathan memory
WORKDIR $HOME
RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git
WORKDIR foonathan_memory_vendor/build
RUN cmake ..
RUN cmake --build . --target install

# Once all dependencies are installed, compile and install Fast DDS.
WORKDIR $HOME
RUN git clone https://github.com/eProsima/Fast-DDS.git
WORKDIR Fast-DDS/build
RUN cmake ..
RUN cmake --build . --target install

# Gradle - Manual install
WORKDIR $HOME
RUN wget https://services.gradle.org/distributions/gradle-6.5.1-bin.zip
RUN apt install -y zip
RUN unzip gradle-6.5.1-bin.zip
RUN gradle-6.5.1/bin/gradle -v

# Fast DDS-Gen
WORKDIR $HOME
RUN git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git
WORKDIR Fast-DDS-Gen
RUN /root/gradle-6.5.1/bin/gradle assemble

WORKDIR $HOME

