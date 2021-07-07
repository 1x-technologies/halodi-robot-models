# Eve model

## Description

The Eve model is built from input files in the ```urdf.in``` folder. 

Models use Xinclude (https://www.w3.org/TR/xinclude/) to include submodels. We are currently using Xinclude instead of xacro because it'll run on every Linux system (even without ROS) and ROS2 does not include xacro yet.

## Building models

CMake is used to generate the final urdf and sdf versions of the models. These are generated in the source directory and should be published to git to allow easy use of the models.

### Installing requirements

```bash
sudo apt install gazebo9  libxml2-utils xsltproc
```

### Building models without ROS

```bash
mkdir build
cd build
cmake ..
make
```

## Visualizing the EveR3 URDF

### Dependencies

To run the urdf visualizer, you need the [ros2 foxy desktop version](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

Install the following

```bash
sudo apt install ros-foxy-joint-state-publisher-gui ros-foxy-xacro
```

### Build

```bash
mkdir -p test1_ws/src
cd test_ws/src

# Clone this repo
## ssh
git clone git@gitlab.com:halodi/controls/halodi-robot-models.git

## Building
cd test_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

### Run

```bash
source ~/test_ws/install/setup.bash
ros2 launch eve_r3_description urdf_viz.launch.py
```

This spawns 2 windows, rviz2 and joint_state_publisher, use the latter for controlling the joint angles.
