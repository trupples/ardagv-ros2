FROM arm64v8/ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ARG USER=runner
ARG BUILD_TARGET=arm64

RUN apt-get update && \
	apt-get install -y apt-utils sudo git wget pkg-config make python3

RUN groupadd -g 1000 -r $USER && \
	useradd -u 1000 -g 1000 --create-home -r $USER

#Change password
RUN echo "$USER:$USER" | chpasswd

#Make sudo passwordless
RUN echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-$USER && \
	usermod -aG sudo $USER && \
	usermod -aG plugdev $USER


USER $USER
WORKDIR /home/${USER}

# set up environment : apt packages, libiio, tof sdk
COPY ./prep_env.sh /home/$USER/
COPY ./resources /home/$USER/
RUN  /home/$USER/prep_env.sh install_apt
RUN  /home/$USER/prep_env.sh build_libiio
RUN  /home/$USER/prep_env.sh build_tof
RUN  /home/$USER/prep_env.sh setup_ros2ws
RUN /home/$USER/prep_env.sh setup_user
# set up ROS2 : build & install packages and deps
COPY ./build_ros2_pkgs.sh /home/$USER/
RUN  /home/runner/build_ros2_pkgs.sh set_rosdep
#RUN  /home/runner/build_ros2_pkgs.sh build_ros2_pkgs
