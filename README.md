# roma-small-rover
ROS workspace for small rover. A course project in 2021 Fall.


Here are some docs from Wujie Shi in /docs

mini_rover_rtos is a package to control motors on the rover.\
loadcell is used to collect data from force sensors on wheels.\
wheel-camera is used to collect data from cameras on wheels.



## Getting started

### Install ROS on PC
Please follow [ROS tutorial](wiki.ros.org) to install corresponding version of ROS

### Setup ROS workspace
```bash
git clone --recurse-submodules -j8 https://github.com/RomaLab/roma-small-rover.git
cd roma-small-rover
catkin_make
```
### startup controller node
Connet NUC and 12 motors with U2D2 and Authorize serial port files.
```bash
sudo chmod 777 /dev/ttuUSB0 
```
launch the controller node.
```bash
source devel/setup.bash
roslaunch mini_rover_rtos rover_control.launch
```
If startup succeeds, the program will promts how to control the rover.


### startup sensor nodes
If we need to take the Ground-Rotation experiment, 
```bash
source devel/setup.bash
roslaunch rover-experiment Ground-Rotation.launch
```

### execute experiment and collect data
If we need to take the Ground-Rotation experiment,
1. Ensure that controller node initialization success.
2. Ensure that the camera nodes has popped up the window.
3. Enter `2` in the  controller node command window.
4. Enter `Y` in the  controller node command window if you want the wheel to turn back.

