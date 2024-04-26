# Development of Quadcopter for Autonomous Navigation

## Overview

Autonomous navigation is necessary for a robotic system to interact with its
surroundings in a real world environment, and it is necessary to realize
technologies such as fully autonomous unmanned aerial vehicles (UAVs) and land
vehicles. Reinforcement Learning (RL) has proven to be a novel and effective
method for autonomous navigation and control, as it is capable of optimizing a
method of converting its instantaneous state to an action at a point in time
(Gugan, 2023; Song, 2023; Doukhi, 2022). Here we use a Deep Deterministic Policy
Gradient (DDPG) RL algorithm to train the COEX Clover quadcopter system to
perform autonomous navigation. With the advent of solid state lasers,
miniaturized optical ranging systems have become ubiquitous for aerial robotics
because of their low power and accuracy (Raj, 2020). By equipping the Clover with
ten Time of Flight (ToF) ranging sensors, we supply continuous spatial data in
combination with inertial data to determine the quadcopter's state, which is
then mapped to its control output. Our results suggest that, while the DDPG
algorithm is capable of training a quadcopter system for autonomous navigation,
its computation-heavy nature leads to delayed convergence, and relying on
discretized algorithms may permit more rapid convergence across episodes.

---

Simon J. Jones  
Daniel Willey, PhD  
Janyl Jumadinova, PhD

Spring 2024

*Department of Physics*  
*Department of Computer and Information Science*  
*Allegheny College, Meadville, PA 16335*

## Goals of This Project

This project aims to train the COEX Clover quadcopter equipped with an array of
Time of Flight (ToF) sensors to perform basic navigation and obstacle avoidance
in randomized scenarios using a Deep Deterministic Policy Gradient (DDPG)
reinforcement learning algorithm. Using randomized environments tests the
effectiveness of curriculum learning for reinforcement learning and the overall
strengths and weaknesses of DDPG for quadcopter control.  By training the
quadcopter to explore randomized environments, this also demonstrates how using
simpler, more economically affordable sensors could potentially enable a
quadcopter to fly in a GPS-denied environment without the use of LiDAR, which is
typically an order of magnitude more expensive.

## Quick Start

The [clover_vm](https://github.com/CopterExpress/clover_vm) image is used to
perform all of the simulations in this project. In addition, it is helpful in
getting started simulating the Clover. The documentation can be found
[here](https://clover.coex.tech/en/simulation_vm.html).

For any information related to setup, see [Guides](#guides). If you feel that
something should be documented and isn't feel free to create an issue using
[this
link](https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact/issues/new).

Assuming you have the `clover_vm` downloaded and have followed
[the setup guide for `clover_train`](#clover_vm---setting-up-clover_train), you
can begin training by running the following command:

```sh
roslaunch clover_train train.py
```

This will run 1,000 training episodes in the 10 procedurally generated worlds
located in `src/pcg/resources/worlds`. If you feel adventurous, try [generating
your own worlds using the `pcg`
package](#procedurally-generating-rooms-using-pcg-module-and-pcg_gazebo), which
is also a part of this project!

### Project Design

The chosen quadcopter platform is the COEX Clover drone, which uses open source
software, ideal for our purpose (Express). The Clover can be
integrated with any sensor thanks to its on-board Raspberry Pi 4 (RPi4); thus,
we will opt to use an array of ToF sensors to measure spatial data. This is, by
definition, a LiDAR system. The Clover supports the MAVROS protocol, which
permits a communication channel between the on-board computer (RPi4) and the
drone’s flight controller. The Clover also supports the *Robotic Operating
System* (ROS), which is a collection of software and methodologies that
generalize robotics development. (Stanford Artificial Intelligence Laboratory et
al.)

<figure>
<img src="images/clover.png" id="fig:clover" style="width:40.0%"
alt="COEX Clover quadcopter. (“COEX Clover,” n.d.)" />
<figcaption aria-hidden="true">COEX Clover quadcopter. <span
class="citation" data-cites="clover">(<span>Express</span>
)</span></figcaption>
</figure>

## Guides

### Installing VirtualBox - Ubuntu 22.04

VirtualBox is the platform used to run all of the programs listed in this
project. In addition, all of the simulation was performed using the `clover_vm`
VirtualBox environment, which can be found at
[https://github.com/CopterExpress/clover_vm](https://github.com/CopterExpress/clover_vm).
[See
here](https://www.virtualbox.org/wiki/Linux_Downloads#Debian-basedLinuxdistributions)
for information from VirtualBox.  

In order to install VirtualBox, one can follow these steps:

* Download VirtualBox public key, convert to a GPG key, and add to keyring.

  ```sh
  wget -O- https://www.virtualbox.org/download/oracle_vbox_2016.asc | sudo gpg --dearmor --yes --output /usr/share/keyrings/oracle-virtualbox-2016.gpg
  ```

* Add VirtualBox's package list to the system.

  ```sh
  sudo echo "deb [arch=amd64 signed-by=/usr/share/keyrings/virtualbox.gpg] https://download.virtualbox.org/virtualbox/debian jammy contrib" > /etc/apt/sources.list.d/virtualbox.list
  ```

* Install VirtualBox.

  ```sh
  sudo apt update
  sudo apt install virtualbox-7.0
  ```

* NOTE: before running, check if `virtualbox-dkms` is installed. You don't want
  it installed. (see [this askubuntu article](https://askubuntu.com/questions/900794/virtualbox-rtr3initex-failed-with-rc-1912-rc-1912))

  ```sh
  dpkg -l | grep virtualbox-dkms
  ```

  * If this command shows that `virtualbox-dkms` was found in your system,
    uninstall it and install the package `dkms`.

    ```sh
    sudo apt-get purge virtualbox-dkms
    sudo apt-get install dkms
    ```

  * Now, rebuild VirtualBox kernel modules.

    ```sh
    sudo /sbin/vboxconfig
    ```

* Otherwise, you can now run VirtualBox

  ```sh
  VirtualBox
  # or
  VirtualBoxVM --startvm <vm-name>
  ```

### Using `clover_vm` for simulating Clover

The [clover_vm](https://github.com/CopterExpress/clover_vm) image is used to
perform all of the simulations in this project. In addition, it is helpful in
getting started simulating the Clover. The documentation can be found
[here](https://clover.coex.tech/en/simulation_vm.html).

#### `clover_vm` - Setup

* Download `clover_vm` image from
  [releases page](https://github.com/CopterExpress/clover_vm/releases/). Select
  the latest release and download. These are multigigabyte files, so they will be
  time consuming to download. Ensure you have enough space.
* Ensure you have VirtualBox installed. See
  [Installing VirtualBox - Ubuntu 22.04](#installing-virtualbox---ubuntu-2204)
  for details.
* Set up the `clover_vm`. Note that this can be done through using
  `virtualbox`'s GUI.

  ```sh
  vboxmanage import /path/to/clover_vm.ova
  ```

* Launch `virtualbox`

  ```sh
  VirtualBoxVM --startvm clover-devel
  ```

* Select the image named **clover-devel**.
* Change its settings such that it has at least 4GB of memory and, preferably,
  as many cores as your system.
* Now that the image is fully configured, select **Start**.

#### `clover_vm` - General Usage

* In the virtual machine, Open a terminal and launch the simulation. Sourcing
  will already be done, because the virtual machine is preconfigured. This opens
  a Gazebo instance and a PX4 SITL simulation in the console. The Gazebo
  instance is what you'll want to refer back to.

  ```sh
  roslaunch clover_simulate simulator.launch
  ```

* In a new terminal, run one of the python scripts in `~/examples/` using `python3`.

  ```sh
  python3 examples/flight.py # this is a fun one
  ```

* Refer back to the open Gazebo instance to see the drone begin to arm. The
  expected behavior is that the drone takes off, moves one meter horizontally,
  and lands again.
* Now you've demonstrated that your system can simulate the Clover!

#### `clover_vm` - Setting up `clover_train`

In the Clover VM, open up a terminal and clone the repository for this project:

```sh
git clone --recursive https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact.git /home/clover/SimonJonesArtifact
```

Then run `catkin_make` and source the development shell file to add the ROS
packages to your PATH:

```sh
cd /home/clover/SimonJonesArtifact
catkin_make
source devel/setup.bash
```

Once this has finished building, you can now install the python files used in
the `clover_train` package:

```sh
python3 -m pip install tensorflow[and-cuda]
python3 -m pip install numpy
```

The rest of the python modules are made available directly through `catkin`. You
can verify if you have successfully set up the packages by running the
following:

```sh
rosrun clover_train launch_clover_simulation
```

This will open a Gazebo instance and spawn the Clover into it. Any issues
encountered during this process can be posted to
[this link](https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact/issues/new).

### Preparing `.STL` Files for Simulation

In order to use an `.STL` file in a robotics simulation, its inertial and
collision properties must first be known. If the geometry of the object is
symmetric and a physical model has been fabricated, this process is much more
straightforward; however, in most cases, processing the mesh will be necessary.
In this project, the inertia tensor and collision box of the custom 3D model
used for mounting the ToF sensors was calculated using Blender 4.0.2 and Meshlab
2022.02.

#### Exporting to COLLADA using Blender

Assuming the `.STL` file already exists, it can first be imported into
blender by navigating to `Import -> STL (.stl) (experimental)`. Make sure to
remove any pre-existing models from the scene by deleting them from the scene
collection window.

If there is complex geometry in the part, it may be worth simplifying the number
of vertices by decimating the mesh. This simplifies the geometry by removing
edges and vertices that may be redundant. A part can be decimated by navigating
to `Modifiers -> Add Modifier -> Generate -> Decimate`. Pictured below, an
example part is decimated using the "Planar" method, but other methods may be
used. By increasing the `Angle Limit` parameter, the value of `Face Count` is
greatly reduced. After the desired number of faces is achieved, typing
`Ctrl + A` will apply the modifier to the part.

![Decimating an object in Blender. Increasing the `Angle Limit` parameter greatly reduces the number of faces in the output mesh.](images/decimate.png)

Once the mesh is simplified to one's needs, it can be exported in the COLLADA
format by navigating to `File -> Export -> Collada (.dae)`.

#### Calculating Inertial Values using MeshLab

![Using MeshLab to calculate the physical properties of a model.](images/meshlab.png)

After opening MeshLab, navigate to `File -> Import Mesh` to import the COLLADA
file. Then, selecting

```txt
Filters
-> Quality Measure and Computations
  -> Compute Geometric Measures
```

will print the physical properties of the mesh in the lower-right log:

```txt
Mesh Bounding Box Size 101.567337 101.567337 30.500050
Mesh Bounding Box Diag 146.840393
Mesh Bounding Box min -50.783676 -50.783672 -0.000002
Mesh Bounding Box max 50.783665 50.783669 30.500048
Mesh Surface Area is 60501.800781
Mesh Total Len of 41916 Edges is 175294.890625 Avg Len 4.182052
Mesh Total Len of 41916 Edges is 175294.890625 Avg Len 4.182052 (including faux edges))
Thin shell (faces) barycenter: -0.011143 0.026900 15.249686
Vertices barycenter 0.036033 -0.086995 15.250006
Mesh Volume is 121008.421875
Center of Mass is -0.008342 0.020133 15.250009
Inertia Tensor is :
| 105930536.000000 261454.750000 566.153809 |
| 261454.750000 105407640.000000 241.684921 |
| 566.153809 241.684921 180192080.000000 |
Principal axes are :
| -0.382688 0.923878 -0.000000 |
| 0.923878 0.382688 -0.000008 |
| -0.000008 -0.000003 -1.000000 |
axis momenta are :
| 105299344.000000 106038840.000000 180192080.000000 |
Applied filter Compute Geometric Measures in 117 msec
```

The inertia tensor is displayed assuming that $m_{\text{object}} =
V_{\text{object}}$, so re-scaling the values is required. An explanation of how
to do so can be found at
[https://classic.gazebosim.org/tutorials?tut=inertia](https://classic.gazebosim.org/tutorials?tut=inertia).

### Procedurally Generating Rooms Using `pcg` Module and `pcg_gazebo`

In order to test the robustness of a model, it is helpful to evaluate its
performance in random environments. In the Clover VM, this can be done by using
the `pcg_gazebo` package, created by Bosch Research [@manhaes2024]. A wrapper
for this package exists under
[https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact](https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact)
in the directory `src/pcg`.

#### Using `pcg` for Room Generation

After cloning
[https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact](https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact)
to the Clover VM, create a Python virtualenv in the `pcg` root directory and
install from `requirements.txt`:

```sh
cd /path/to/SimonJonesArtifact/src/pcg
python3 -m virtualenv venv
pip install -r requirements.txt
```

Now that you have all of the necessary packages, assuming that you have properly
sourced your shell in `SimonJonesArtifact/devel`, you can run the `generate`
script under `pcg`:

```sh
rosrun pcg generate -h
```

If this works correctly, you should see a help output describing the possible
CLI flags. To generate ten worlds, saving them to the `.gazebo/` directory, you
can run the following command:

```sh
rosrun pcg generate \
  --models-dir=/home/clover/.gazebo/models \
  --worlds-dir=/home/clover/.gazebo/worlds \
  --num-worlds=1
```

However, by default, the generated worlds are stored to
`/path/to/SimonJonesArtifact/src/pcg/resources/worlds`, and the models are
stored at `/path/to/SimonJonesArtifact/src/pcg/models`. This allows them to be
incorporated into the project's ROS path by default.

#### Installing `pcg_gazebo`

For manually using `pcg_gazebo` without the custom `pcg` module, you must
install `pcg_gazebo`. To install `pcg_gazebo` on the Clover VM, start by
updating the system:

```sh
sudo apt-get update
sudo apt upgrade
```

Then install supporting packages:

```sh
sudo apt install libspatialindex-dev pybind11-dev libgeos-dev
pip install "pyglet<2"
pip install markupsafe==2.0.1
pip install trimesh[easy]==3.16.4
```

Then, you may need to update `pip`, as the version that comes by default in the
VM is not up to date:

```sh
sudo pip install --upgrade pip
```

Then install the `pcg_gazebo` package:

```sh
pip install pcg_gazebo
```

Before running, make sure to create the default directory where the tool will
save the world files, `~/.gazebo/models`:

```sh
mkdir -p ~/.gazebo/models
```

#### Running `pcg_gazebo`

For a basic cuboid room, one can run the following:

```sh
pcg-generate-sample-world-with-walls \
  --n-rectangles 1 \
  --world-name <your-world-name> \
  --preview
```

This will generate the world file `~/.gazebo/models/<your-world-name>.world`
that contains a cuboid.

Other examples, that incorporate randomly placed obstacles, are shown in the
following:

```sh
pcg-generate-sample-world-with-walls \
  --n-rectangles 10 \
  --n-cubes 10 \
  --world-name <your-world-name> \
  --preview
pcg-generate-sample-world-with-walls \
  --n-rectangles 10 \
  --x-room-range 6 \
  --y-room-range 6 \
  --n-cubes 15 \
  --wall-height 6 \
  --world-name <your-world-name> \
  --preview
pcg-generate-sample-world-with-walls \
  --n-rectangles 10 \
  --n-cylinders 10 \
  --world-name <your-world-name> \
  --preview
pcg-generate-sample-world-with-walls \
  --n-rectangles 10 \
  --n-spheres 10 \
  --world-name <your-world-name> \
  --preview
```

## References

(1) Raj, T.; Hanim Hashim, F.; Baseri Huddin, A.; Ibrahim, M. F.; Hussain, A. A Survey on LiDAR Scanning Mechanisms. Electronics 2020, 9 (5), 741.  
(2) Song, Y.; Romero, A.; Müller, M.; Koltun, V.; Scaramuzza, D. Reaching the Limit in Autonomous Racing: Optimal Control versus Reinforcement Learning. Science Robotics 2023, 8 (82), eadg1462.  
(3) Gugan, G.; Haque, A. Path Planning for Autonomous Drones: Challenges and Future Directions. Drones 2023, 7 (3), 169.  
(4) Doukhi, O.; Lee, D. J. Deep Reinforcement Learning for Autonomous Map-Less Navigation of a Flying Robot. IEEE Access 2022, 10, 82964–82976. [https://doi.org/10.1109/ACCESS.2022.3162702](https://doi.org/10.1109/ACCESS.2022.3162702).  
