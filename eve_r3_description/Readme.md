# Easy Mode

### Set up an SSH key with GitLab

[Follow these instructions](https://docs.gitlab.com/ee/ssh/)

### Get the setup script

```
git clone git@gitlab.com:halodi/tooling/halodi-developer-fresh-install.git
cd halodi-developer-fresh-install
git checkout CTRL-68 
bash setup.sh
```

This will run for a bit and when its done Eclipse will be installed, all the code will be checked out.

### Import the code into Eclipse with Buildship

File -> Import -> Gradle -> Existing gradle project

Select the folder `$HOME/git/repository-group`

Select use the Gradle Wrapper

for Java Home select `$HOME/bin/java-8`

Hit Finish and wait.

You should be all set!

# Hard Mode

This set of instructions is Optional, it is here to describe what the Easy Mode script is doing. 

## Description

The Eve model is built from input files in the ```urdf.in``` folder. 

Models use Xinclude (https://www.w3.org/TR/xinclude/) to include submodels. We are currently using Xinclude instead of xacro because it'll run on every Linux system (even without ROS) and ROS2 does not include xacro yet.

## Building models

CMake is used to generate the final urdf and sdf versions of the models. These are generated in the source directory and should be published to git to allow easy use of the models.

### OS requirements 

You should be running Ubuntu 20.04 to get the correct ROS version

To check run

```
lsb_release -d
```

and look for "20.04" in the result

```
Description:	Ubuntu 20.04.5 LTS
````


### Installing requirements

```
sudo apt install gazebo9  libxml2-utils xsltpro make cmakec
```

### JVM depenancies

Download https://cdn.azul.com/zulu/bin/zulu8.54.0.21-ca-fx-jdk8.0.292-linux_x64.tar.gz 

extract it to $HOME/bin/java-8/

```
JVERELEASE=zulu8.54.0.21-ca-fx-jdk8.0.292-linux_x64
JVMDIR=$HOME/bin/java-8/
mkdir -p $JVMDIR
wget https://cdn.azul.com/zulu/bin/$JVERELEASE.tar.gz -O $JVMDIR/$JVERELEASE
tar -xzf $JVMDIR/$JVERELEASE.tar.gz -C $JVMDIR
mv $JVMDIR/$JVERELEASE/* $JVMDIR/
rmdir $JVMDIR/$JVERELEASE/

```

When set up correctly verify that the extraction is in the correct place:

```
# ls $HOME/bin/java-8/
ASSEMBLY_EXCEPTION        DISCLAIMER  LICENSE          OPENJFX_THIRD_PARTY_README  src.zip
bin                       include     man              readme.txt                  THIRD_PARTY_README
CLASSPATH_EXCEPTION_NOTE  jre         OPENJFX_LICENSE  release                     Welcome.html
demo                      lib         openjfx-src.zip  sample                      zulu8.54.0.21-ca-fx-jdk8.0.292-linux_x64.tar.gz

```
If the file structure looks good, make sure the Java version is running ok

```
# $HOME/bin/java-8/bin/java -version

openjdk version "1.8.0_292"
OpenJDK Runtime Environment (Zulu 8.54.0.21-CA-linux64) (build 1.8.0_292-b10)
OpenJDK 64-Bit Server VM (Zulu 8.54.0.21-CA-linux64) (build 25.292-b10, mixed mode)

```

### Set up an SSH key with GitLab

[Follow these instructions](https://docs.gitlab.com/ee/ssh/)

### Clone the source code

```
ROBOTSLUG=halodi-robot-models
ROBOTSURL=git@gitlab.com:halodi/controls/$ROBOTSLUG.git
mkdir $HOME/git
git clone $ROBOTSURL
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
mkdir -p test_ws/src
cd test_ws/src

# Clone this repo
## ssh
git clone git@gitlab.com:halodi/controls/halodi-robot-models.git

## Building
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

### Run

```bash
source ~/test_ws/install/setup.bash
ros2 launch eve_r3_description urdf_viz.launch.py
```

This spawns 2 windows, rviz2 and joint_state_publisher, use the latter for controlling the joint angles.
