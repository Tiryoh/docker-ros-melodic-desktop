# docker_ros-melodic-desktop

![](https://img.shields.io/docker/automated/tiryoh/ros-melodic-desktop.svg)
![](https://img.shields.io/docker/build/tiryoh/ros-melodic-desktop.svg)
![](https://img.shields.io/docker/pulls/tiryoh/ros-melodic-desktop.svg)

## Docker Hub

https://hub.docker.com/r/tiryoh/ros-melodic-desktop/

## Usage

building ROS package `<package_name>` located in `~/repo/ros_ws/`:

```
$ docker run --rm -it -v ~/repo/ros_ws/<package_name>:/home/ubuntu/catkin_ws/src/<package_name> tiryoh/ros-melodic-desktop catkin_make
```

## License

The Apache V2.0 License

2018 (C) Tiryoh
