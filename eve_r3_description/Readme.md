# Eve model

## Description

The Eve model is built from input files in the ```urdf.in``` folder. 

Models use Xinclude (https://www.w3.org/TR/xinclude/) to include submodels. We are currently using Xinclude instead of xacro because it'll run on every Linux system (even without ROS) and ROS2 does not include xacro yet.

## Building models

CMake is used to generate the final urdf and sdf versions of the models. These are generated in the source directory and should be published to git to allow easy use of the models.

### Installing requirements

```
sudo apt install gazebo9  libxml2-utils xsltproc
```

### Building models without ROS

```
mkdir build
cd build
cmake ..
make
```
