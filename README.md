# docker_ros-melodic-desktop

![](https://img.shields.io/docker/automated/tiryoh/ros-melodic-desktop.svg)
![](https://img.shields.io/docker/build/tiryoh/ros-melodic-desktop.svg)
![](https://img.shields.io/docker/pulls/tiryoh/ros-melodic-desktop.svg)

## Docker Hub

https://hub.docker.com/r/tiryoh/ros-melodic-desktop/

## Usage

* move into your ROS package, and just run:

  ```
  $ docker run --rm -it -v $(pwd):/ws tiryoh/ros-melodic-desktop catkin_make
  ```

  * `/ws` directory is simbolic linked to `/home/ubuntu/catkin_ws/src/ws`

* building ROS package `<package_name>` located in `~/workspace/ros/`:

  ```
  $ docker run --rm -it -v ~/workspace/ros/<package_name>:/home/ubuntu/catkin_ws/src/<package_name> tiryoh/ros-melodic-desktop catkin_make
  ```

## License

The Apache V2.0 License

2018 (C) Tiryoh
