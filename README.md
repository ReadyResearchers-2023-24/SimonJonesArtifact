# Research Artifact - Simon Jones

This is work toward simulating the COEX Clover drone in ROS2.

### Setup

Run the following to set up your system. This information is taken from the [ros configuration docs](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

```sh
. setup.sh
```

#### ROS2 Rolling Installation - Ubuntu 22.04

* Set locale
  ```sh
  locale  # check for UTF-8

  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  locale  # verify settings
  ```
* Ensure Ubuntu Universe repository is enabled
  ```sh
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  ```
* Add ROS2 GPG key with apt
  ```sh
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```
* Add ROS2 repository to your sources list
  ```sh
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```
* Update your apt repository caches after setting up the repositories. Do not skip this step.
  ```sh
  sudo apt update
  sudo apt upgrade
  ```
* Install `ros-rolling-desktop`
  ```sh
  sudo apt install ros-rolling-desktop
  ```
* Install `ros-rolling-ros-base`
  ```sh
  sudo apt install ros-rolling-desktop
  ```
* Set up environment by sourcing ROS2 `setup.bash`, `setup.sh`, etc.
  ```sh
  . /opt/ros/rolling/setup.bash
  ```
* Try out examples and read tutorials.

#### ROS Installation - NixOS

This section is a work in progress. Feel free to contribute.  

* FIXME
    ```sh
    FIXME
    ```

### docs

* [COEX Clover ROS docs](http://wiki.ros.org/Robots/clover)
* Creating a new package in ROS2 (see [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package))
  * ```sh
    ros2 pkg create --build-type ament_cmake <package_name>
    ros2 pkg create --build-type <build_type> --node-name <node_name> <package_name> --license <license>
    ```

* [ROS2 Command Line Syntax](https://design.ros2.org/articles/ros_command_line_arguments.html)
* [Colcon](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) is a tool used for managing and building ROS2 packages.
* LSP Integration
  * In order to get `clangd` to work with the libraries/headers in your project, set the following at the top of each `CMakeLists.txt` file in the projects you're working on:
    ```cmake
    # CMakeLists.txt
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    ...
    ```
* `clangd`: changing C++ standard being used: [this link](https://stackoverflow.com/questions/73758291/is-there-a-way-to-specify-the-c-standard-of-clangd-without-recompiling-it)
