#   
# Title: LeManchot-DC - it is a multimodal data platform including data acquisition and data transmission components to be used inside a drone
# Repository: https://github.com/parham/lemanchot-dc
# Developed by: Parham Nooralishahi, PhD. candidate @ Computer and Electrical Engineering Department, Université Laval
# Supervisor: Professor Xavier Maldague
# University: Université Laval
# 

FROM nvidia/opengl:1.2-glvnd-devel-ubuntu18.04

LABEL developer="Parham Nooralishahi"
LABEL email="parham.nooralishahi@gmail.com"
LABEL supervisor="Xavier Maldague"
LABEL organization="Université Laval"

ARG DEBIAN_FRONTEND=noninteractive

ENV TZ=America/Montreal

RUN apt-get update && \
    apt-get install -y apt-transport-https
RUN apt-get remove python-* && apt-get autoremove

RUN apt-get install -y git wget zip build-essential cmake sysvbanner

RUN echo "" && \
    banner UNIVERSITE LAVAL && \
    echo "-----------------------------------------" && \
    banner LEMANCHOT

# Installing Python 3.7
RUN apt update && \
    apt-get install -y software-properties-common lsb-release \
        build-essential zlib1g-dev libncurses5-dev libgdbm-dev \ 
        libnss3-dev libssl-dev libreadline-dev libffi-dev wget && \
    apt update && lsb_release -a

RUN apt-get install -y python3 python3-dev python3-pip

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt install -y curl && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && apt install -y ros-melodic-desktop-full && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

RUN apt-get install -y python3-pip python3-yaml build-essential && \
    pip3 install rospkg catkin_pkg rosdep rosinstall-generator wstool rosinstall && \
    rosdep init && rosdep fix-permissions && rosdep update

RUN apt-get install -y python3-dev libsdl2-dev python3-catkin-pkg-modules python3-rospkg-modules \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-ttf-dev \
    libportmidi-dev \
    libswscale-dev \
    libavformat-dev \
    libavcodec-dev \
    zlib1g-dev \
    libgstreamer1.0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good

RUN add-apt-repository ppa:kivy-team/kivy && \
    apt-get update && apt-get install -y python3-kivy

RUN python3 -m pip install --upgrade pip && \ 
    pip3 install kivymd imutils flask psutil imageio pillow console-menu opencv-python

RUN apt-get update
RUN apt-get -y install \
    libcanberra-gtk-module \
    libcanberra-gtk3-module firefox && apt-get clean

# Add the sourcecodes 
RUN mkdir -p /home/lemanchot-dc
ADD . /home/lemanchot-dc

#nvidia-driver-455
RUN apt-get update && \
    apt-get install -y  mesa-utils \
        libglvnd-dev libgl1-mesa-dev \ 
        libegl1-mesa-dev libgles2-mesa-dev && \
    apt-get install -y pkgconf libglfw3 libglfw3-dev libglew2.0 libglew-dev 

# RealSense2 SDK 
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \ 
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" && apt-get update && \
    apt install -y librealsense2 librealsense2-dkms \ 
        librealsense2-utils librealsense2-dev librealsense2-utils librealsense2-gl && \
    apt-get install -y ros-melodic-realsense2-camera



# CMD ["firefox"]

# RUN python3 -m pip install --upgrade pip && \
#     pip3 install rosdep rosinstall-generator vcstool rosinstall catkin-pkg

# RUN rosdep init && rosdep update && apt-get install -y python3-catkin-pkg && \
#     mkdir -p "${ROS_INSTALL_DIR}/src" && cd ${ROS_INSTALL_DIR} && \
#     rosinstall_generator desktop_full --rosdistro melodic --deps --tar > melodic-desktop-full.rosinstall && \
#     vcs import src < melodic-desktop-full.rosinstall

# RUN cd ${ROS_INSTALL_DIR} && rosdep install --from-paths src --ignore-src --rosdistro melodic -y