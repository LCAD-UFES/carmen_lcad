FROM osrf/ros:noetic-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    build-essential \
    cmake \
    libboost-all-dev \
    libeigen3-dev \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-tf \
    ros-noetic-cv-bridge \
    ros-noetic-navigation \
    ros-noetic-robot-localization \
    ros-noetic-robot-state-publisher \
    libpcl-dev \
    software-properties-common \
    libopencv-dev

# GTSAM
RUN add-apt-repository ppa:borglab/gtsam-release-4.0 && \
    apt-get update && \
    apt-get install -y libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

ARG CATKIN_WS_DIR=/root/catkin_ws
RUN mkdir -p ${CATKIN_WS_DIR}/src

# ceres-solver (dependência do lt-mapper)
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://ceres-solver.googlesource.com/ceres-solver -b 2.0.0 ${CATKIN_WS_DIR}/src/ceres-solver && \
    cd ${CATKIN_WS_DIR}/src/ceres-solver && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

# SC-LIO-SAM (fork compatível com lt-mapper — troque aqui se quiser manter o gisbi-kim)
WORKDIR ${CATKIN_WS_DIR}/src
RUN git clone https://github.com/dongjae0107/SC-LIO-SAM.git

RUN sed -i 's|#include <opencv/cv.h>|#include <opencv2/opencv.hpp>|' \
    ${CATKIN_WS_DIR}/src/SC-LIO-SAM/SC-LIO-SAM/include/utility.h

# LT-mapper
RUN git clone https://github.com/dongjae0107/lt-mapper ${CATKIN_WS_DIR}/src/lt-mapper

WORKDIR ${CATKIN_WS_DIR}
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build lio_sam ltslam removert"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source ${CATKIN_WS_DIR}/devel/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]