FROM ros:noetic

RUN apt-get update && apt-get install --no-install-recommends -y \
    && apt-get install --no-install-recommends -y wget nano build-essential \
    git libgtest-dev libboost-all-dev libopencv-dev \
    ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
    ros-noetic-eigen-conversions ros-noetic-tf-conversions ros-noetic-pcl-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace'

RUN git clone https://github.com/koide3/ndt_omp
RUN git clone https://github.com/SMRT-AIST/fast_gicp --recursive
RUN git clone https://github.com/koide3/hdl_global_localization

COPY . /root/catkin_ws/src/hdl_localization/
WORKDIR /root/catkin_ws/src/hdl_localization

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
