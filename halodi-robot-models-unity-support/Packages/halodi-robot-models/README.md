# Halodi Robot models - Unity Support

This package contains the Halodi Robot Models 


## Installation

First, add "Halodi Unity Package Registry Manager" to your project. In Unity, go to Window -> Package manager, and "Add package from git URL" (Press + in the top left corner)

```
https://github.com/Halodi/halodi-unity-package-registry-manager.git
```

A new menu "Packages" will show up. Then add the "Halodi Unity Package Creator" to your project.

Go to "Packages -> Manage scoped registries" and add registry with the following settings

```
Name: Halodi OSS
Url: https://artifacts.halodi.com/repository/upm-open-source-group/
Scope: com.halodi
Always auth: false
Token: [Leave empty]
```

Then go to Packages -> Add Packages (Bulk) and enter

```
com.halodi.halodi-robot-models
```

Press Add packages to add "Halodi Unity Package Creator" to your project.


## Usage 

Prefabs for all the robot models can be found in the Runtime/Halodi/Models directory.


## Updating robot models


## More information

https://github.com/Halodi/halodi-3d-model-sources

## Notice

This package uses software from

- Siemens ROS# for importing the URDF models into Unity. https://github.com/siemens/ros-sharp

## License

The Halodi Robot Models are licensed under the Apache 2.0 license.
