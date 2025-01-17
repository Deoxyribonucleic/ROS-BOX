**Requirements for RaspiMouse V3**
# ROS Documentation

## Prerequisites

To build this you need to install

* make
* graphviz
* ROS Noetic
* ar_track_overhead_alvar
* Ubuntu Desktop 20.04


### Installation
#Downloading Required Packages
```
git clone https://github.com/keeratifts/ar_track_overhead_camera.git
```
See [ROS wiki](http://wiki.ros.org/ar_track_alvar) for the users document.

In our case: marker 0 ~ 8 is being used for the robots.













### Simulations.
#Download Simulation Package
```cd ~/catkin_ws/src
git clone https://github.com/rt-net/raspimouse_sim.git
```
#Installing exsitent package
```git clone https://github.com/rt-net/raspimouse.git
git clone https://github.com/rt-net/raspimouse_description.git
rosdep install -r -y -i --from-paths raspimouse*
```
#Build Package
```
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```
#Download required hardware model
```
rosrun raspimouse_gazebo download_gazebo_models.sh
```
## Commands

#Running gazebo
```
roslaunch raspimouse_gazebo raspimouse_with_emptyworld.launch  ```//to start up the gazebo with empty world.
```

#Running codes <In new terminal>.
```
rosrun <launch file> <script.py>

rosrun ros_box main.py

```




**NB:** Ensure that you have all the required dependencies installed and that the correct ROS workspace is sourced before running any commands. If you encounter issues during the installation, it might be helpful to check for missing packages or resolve any conflicts in your ROS workspace, especially with multiple package versions.


