# Halodi Robot Models

This repository contains the open source robot models from Halodi. 

## Available models

- Eve R3

## Supported platforms

Build systems and support is provided for the following software packages

- ROS2
- Unity 2019+ (halodi-robot-models-unity-support)
- IHMC Simulation Construction Set (Java)

## License

The Halodi Robot Models are licensed under the Apache License, Version 2.0. 

## Usage

### ROS2

Create a ROS2 workspace and place this repository in the `src` folder. Use `colcon build` to generate the models and install them into your workspace.

`colcon build` is also used to regenerate the `.urdf` and `.sdf` models based on the partial models. Always run `colcon build` before recompiling Unity and Simulation Construction Set support.

### Unity 2019+

A Unity project with prefabs is available in the halodi-robot-models-unity-support folder. 

For installation instructions, see [halodi-robot-models-unity-support/Packages/halodi-robot-models/](halodi-robot-models-unity-support/Packages/halodi-robot-models/).

#### Regenerate Unity models

After regenerting the models using `colcon build` in your ROS2 workspace, you need to reimport the models into the Unity project. Load the Unity project, and from the menu bar run [Halodi] -> [Reimport Halodi Robot Models].

After importing, prefabs for the robot models can be found in `Runtime/halodi/models/`.

The Unity project includes the halodi-unity-package-creator plugin to publish to a NPM server.

### IHMC Simulation Construction Set (Java)

The Halodi Robot Models are compatible with IHMC Simulation Construction set.

#### Adding as dependency

To add this as a dependency to your Java project, add the following to your build.gradle

```
repositories {
    mavenCentral()
    maven { url "http://dl.bintray.com/halodirobotics/maven-release" }
    mavenLocal()
}
```
```
dependencies {
    compile group: "com.halodi", name: "halodi-robot-models", version: "0.1.0"
}

```


#### Publishing to maven repository

To compile and publishing to your local maven repository, run

```
gradle publishToMavenLocal
```

To upload to bintray, set bintrayUsername and bintrayApiKey in ~/.gradle/gradle.properties and run

```
gradle bintrayUpload
```


## Folder structure

Each robot is placed in a folder named `[robot]_description`. The structure of each folder contains

```
package.xml
Readme.md
License.md
CMakeLists.txt
model.config
meshes/
sdf/
urdf/
urdf.in/
cmake/

```

The files `package.xml`, `CMakeLists.txt`, `urdf.in/`, `cmake/` and `model.config` are not included when generating Java and Unity packages.

