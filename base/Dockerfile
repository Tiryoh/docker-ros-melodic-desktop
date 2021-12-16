FROM ubuntu:bionic-20210930 AS builder
RUN apt-get update
RUN apt-get install -y curl
RUN apt-get install -y --no-install-recommends gcc libc-dev
RUN curl -o /usr/local/bin/su-exec.c https://raw.githubusercontent.com/ncopa/su-exec/master/su-exec.c
RUN gcc -Wall /usr/local/bin/su-exec.c -o/usr/local/bin/su-exec
RUN chown root:root /usr/local/bin/su-exec
RUN chmod 0755 /usr/local/bin/su-exec

FROM ubuntu:bionic-20210930
ARG maintainer
ARG org.opencontainers.image.title
ARG org.opencontainers.image.description
ARG org.opencontainers.image.url
ARG org.opencontainers.image.source
ARG org.opencontainers.image.version
ARG org.opencontainers.image.revision
ARG org.opencontainers.image.created

LABEL maintainer="${maintainer}"
LABEL org.opencontainers.image.title="${org.opencontainers.image.title}"
LABEL org.opencontainers.image.description="${org.opencontainers.image.description}"
LABEL org.opencontainers.image.url="${org.opencontainers.image.url}"
LABEL org.opencontainers.image.source="${org.opencontainers.image.source}"
LABEL org.opencontainers.image.version="${org.opencontainers.image.version}"
LABEL org.opencontainers.image.revision="${org.opencontainers.image.version}"
LABEL org.opencontainers.image.created="${org.opencontainers.image.created}"

COPY --from=builder /usr/local/bin/su-exec /sbin/
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -k https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
RUN apt-get update -q && \
    apt-get install -y ros-melodic-desktop-full python-rosdep &&\
    apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools python3-vcstool &&\
    rm -rf /var/lib/apt/lists/*
RUN rosdep init
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
RUN rosdep update
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
