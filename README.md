# Research Artifact - Simon Jones

This is work toward simulating the COEX Clover drone in ROS2.  

See [link to project proposal](https://simon-jones.netlify.app/comp/2023-10-06) for further details.  

### Setup

Run the following to set up your system. This information is taken from the [ros configuration docs](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

```sh
. setup.sh
```

### ROS2 Rolling Installation - Ubuntu 22.04

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

### ROS1 Noetic Installation - Ubuntu 22.04

* Because ROS Noetic does not support Ubuntu 22.04 you either have to
  1. Build from source
  2. Use a docker container (recommended)
* This assumes you have `docker` installed. See [Installing docker - Ubuntu 22.04](#installing-docker---ubuntu-2204) for details.

* Pull docker image with tag for `noetic-desktop-full`. For more tags, see [this link](https://hub.docker.com/r/osrf/ros/tags)
  ```sh
  docker pull osrf/ros:noetic-desktop-full
  ```

### Using `clover_vm` for simulating Clover

The [clover_vm](https://github.com/CopterExpress/clover_vm) image is helpful in getting started simulating clover.

* Setup
  * Download `clover_vm` image from [releases page](https://github.com/CopterExpress/clover_vm/releases/). Select the latest release and download. These are multigigabyte files, so they will be time consuming to download. Ensure you have enough space.
  * Ensure you have VirtualBox installed. See [Installing VirtualBox - Ubuntu 22.04](#installing-virtualbox---ubuntu-2204) for details.
  * Set up the `clover_vm`. Note that this can be done through using `virtualbox`'s GUI.
    ```sh
    vboxmanage import /path/to/clover_vm.ova
    ```
  * Launch `virtualbox`
    ```sh
    virtualbox
    ```
  * Select the image named **clover-devel** and select **Start**.
* Usage
  * FIXME

### ROS Installation - NixOS

This section is a work in progress. Feel free to contribute.  

* FIXME
    ```sh
    FIXME
    ```

## Reference

### Installing VirtualBox - Ubuntu 22.04

[See here](https://www.virtualbox.org/wiki/Linux_Downloads#Debian-basedLinuxdistributions) for info from virtualbox.  

```sh
# download virtualbox public key, convert to GPG key, and add to keyring
wget -O- https://www.virtualbox.org/download/oracle_vbox_2016.asc | sudo gpg --dearmor --yes --output /usr/share/keyrings/oracle-virtualbox-2016.gpg

# add package list to system
sudo echo "deb [arch=amd64 signed-by=/usr/share/keyrings/virtualbox.gpg] https://download.virtualbox.org/virtualbox/debian jammy contrib" > /etc/apt/sources.list.d/virtualbox.list

# install virtualbox
sudo apt update
sudo apt install virtualbox-7.0

# NOTE: before running, check if virtualbox-dkms is installed (you don't want it installed) (see https://askubuntu.com/questions/900794/virtualbox-rtr3initex-failed-with-rc-1912-rc-1912)
dpkg -l | grep virtualbox-dkms
# if yes, then delete and install `dkms`
sudo apt-get purge virtualbox-dkms
sudo apt-get install dkms
# and rebuild virtualbox kernel modules
sudo /sbin/vboxconfig

# otherwise, you can now run virtualbox
virtualbox
```

### Installing Docker - Ubuntu 22.04

* Update the system
  ```sh
  sudo apt update
  ```
* Add dependencies to install docker
  ```sh
  sudo apt install apt-transport-https ca-certificates curl software-properties-common
  ```
* Add the official Docker key to avoid non-authentic packages
  ```sh
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
  ```
* Add the official Docker repo
  ```sh
  sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu jammy stable"
  ```
* Install Docker
  ```sh
  sudo apt install docker-ce
  ```
* Docker should be installed, the daemon started, and the proces enabled to start on boot. Check that its running
  ```sh
  sudo systemctl status docker
  # it should say `active (running)`
  ```

* [COEX Clover ROS docs](http://wiki.ros.org/Robots/clover)
* [COEX Clover Simulation VM](https://github.com/CopterExpress/clover_vm): simple way to get familiarized with simulating the Clover without installation process.
* Creating a new package in ROS2 (see [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package))
  ```sh
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
