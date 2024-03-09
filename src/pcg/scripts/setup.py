#!/usr/bin/env python3

import os
import subprocess
import sys
import rospkg
import rospy

rospy.init_node("setup")

force = bool(rospy.get_param("/force"))

ros_package_name = "pcg"

project_root_abs = rospkg.RosPack().get_path(ros_package_name)

# Define the name of the requirements file
requirements_file = os.path.join(project_root_abs, "requirements.txt")

# Define the file name of the virtual environment
venv_file = os.path.join(project_root_abs, "venv")


def create_venv():
    """Create a virtual environment."""
    print("Creating virtual environment...")
    subprocess.run([sys.executable, "-m", "venv", venv_file], check=True)


def install_dependencies():
    """Install dependencies from requirements.txt."""
    print("Installing dependencies...")
    subprocess.run(
        [os.path.join(venv_file, "bin", "pip"), "install", "-r", requirements_file],
        check=True,
    )


def install_deps_if_not_installed():
    venv_initialized = os.path.exists(venv_file)
    # Check if the virtual environment already exists
    if not venv_initialized:
        create_venv()
    # only install packages if venv file is not created or force is set to true
    if not venv_initialized:
        # Install dependencies
        install_dependencies()


if __name__ == "__main__":
    install_deps_if_not_installed()
