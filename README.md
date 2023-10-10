# Research Artifact - Simon Jones

This is work toward simulating the COEX Clover drone in ROS2.

### Setup

Run the following to set up your system. This information is taken from the [ros configuration docs](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

```sh
. setup.sh
```

### docs

* [COEX Clover ROS docs](http://wiki.ros.org/Robots/clover)
* Createing a new package in ROS2 (see [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package))
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
