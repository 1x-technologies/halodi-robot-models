# Prerequisites


### Set up an SSH key with GitLab and Github

[Follow these instructions for gitlab](https://docs.gitlab.com/ee/ssh/)

[Follow these instructions for github](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

### Get Access to the Halodi Source code projects

Ask you manager to give you access to:

```
gitlab.com:halodi/controls/
```


# Easy Mode

The easy mode will set up a fresh install with one script instead of an hour's worth of steps.

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

### Run the Eve Simulation

Select the project 

```
halodi->eve-simulation
```

run the simulation class Main in:

```
src/main/java/com/halodi/eve/simulation/SCSEveSimulation.java
```

You should be all set!

# Hard Mode

This set of instructions is Optional, it is here to describe what the Easy Mode script is doing. 


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


### Clone the source code

```
mkdir ~/git
cd ~/git
git clone git@github.com:ihmcrobotics/repository-group.git
cd repository-group
git clone git@github.com:ihmcrobotics/ihmc-open-robotics-software.git

git clone git@gitlab.com:halodi/controls/wbc/halodi-whole-body-controller.git halodi
git clone git@gitlab.com:halodi/controls/wbc/ros2-websocket-bridge.git
git clone git@gitlab.com:halodi/controls/wbc/halodi-controller-build-system.git
gradle wrapper --gradle-version 6.8.1 --distribution-type all

```

### Install Eclipse

Download Eclipse

```
ECLIPSE_RELEASE=2020-12
ECLIPSE_ARTIFACT=eclipse-java-$ECLIPSE_RELEASE-R-linux-gtk-x86_64.tar.gz
ECLIPSE_URL=http://www.mirrorservice.org/sites/download.eclipse.org/eclipseMirror/technology/epp/downloads/release-$ECLIPSE_RELEASE/R/$ECLIPSE_ARTIFACT
wget $ECLIPSE_URL -O $HOME/bin/$ECLIPSE_ARTIFACT
tar -xzf $HOME/bin/$ECLIPSE_ARTIFACT -C $HOME/bin/
mv $HOME/bin/eclipse $HOME/bin/eclipse-halodi
```

### Set Up Eclipse

Run Eclipse. 

Open

```
Window->Preferences->Java->Installed JREs 
```

Select Add

Select the folder 

```
$HOME/bin/java-8/
```

Remove the other JRE.




Get the Halodi formatter from

```
wget https://bitbucket.ihmc.us/projects/LIBS/repos/ihmc-open-robotics-software/raw/ihmc-java-toolkit/CodeFormatTemplates/IHMCEclipseFormatter.xml
```

Open the Eclipse menu

```
Window > Preferences > Java ->Code Style -> Formatter

```

Select import and select the file you downloaded above. 

Change the depricated API warning


```
Window > Preferences > Java ->Compiler Error/Warnings -> Depricated and Restricted API ->Forbidded References (access rules)

```

And switch from `Error` to `Ignore`


Set the content assists

```
Window > Preferences > Java ->Editor -> Content Assist -> Favorites

```
Click "New Type..." and add `org.junit.jupiter.api.Assertions`



### Import code into Eclipse with Buildship

```
File -> import -> Gradle -> Existing Gradle Project
```

Select the folder at

```
$HOME/git/repository-group
```

Select Gragle Wrapper

Select the java home:

```
$HOME/bin/java-8/
```

Finish. 


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
