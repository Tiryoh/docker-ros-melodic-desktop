FROM ubuntu:xenial
MAINTAINER tiryoh

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -k https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
RUN apt-get update -q && \
    apt-get install -y ros-kinetic-desktop-full &&\
    rm -rf /var/lib/apt/lists/*
RUN rosdep init
RUN locale-gen en_US.UTF-8
RUN useradd -m -d /home/ubuntu ubuntu -p `perl -e 'print crypt("ubuntu", "salt"),"\n"'` && \
    echo "ubuntu ALL=(ALL) ALL" >> /etc/sudoers
USER ubuntu
WORKDIR /home/ubuntu
ENV HOME=/home/ubuntu \
    CATKIN_SHELL=bash
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN rosdep update
RUN mkdir -p ~/catkin_ws/src \
    && /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace $HOME/catkin_ws/src' \
    && /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd $HOME/catkin_ws; catkin_make'
RUN echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc \
    && echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
