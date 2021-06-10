FROM ros:melodic

RUN apt-get update && apt-get install --no-install-recommends -y \
    && apt-get install --no-install-recommends -y wget nano build-essential \
    libomp-dev libgtest-dev libboost-all-dev libopencv-dev \
    ros-melodic-tf2 ros-melodic-tf2-ros ros-melodic-tf2-geometry-msgs \
    ros-melodic-eigen-conversions ros-melodic-tf-conversions ros-melodic-pcl-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace'

RUN git clone https://github.com/koide3/ndt_omp -b melodic
RUN git clone https://github.com/SMRT-AIST/fast_gicp --recursive
RUN git clone https://github.com/koide3/hdl_global_localization

COPY . /root/catkin_ws/src/hdl_localization/
WORKDIR /root/catkin_ws/src/hdl_localization

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
