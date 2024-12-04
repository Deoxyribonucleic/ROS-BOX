**Requirements for RaspiMouse V3**
ROS 2 Humble: Ubuntu Desktop 22.04

ROS Noetic: Ubuntu Desktop 20.04

**Download Simulation Package**
```
cd ~/catkin_ws/src
git clone https://github.com/rt-net/raspimouse_sim.git
```

**Installing exsitent package**
```
git clone https://github.com/rt-net/raspimouse.git
git clone https://github.com/rt-net/raspimouse_description.git
rosdep install -r -y -i --from-paths raspimouse*
```

**Build Package**
```
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash

```

**Download required hardware model**

```
rosrun raspimouse_gazebo download_gazebo_models.sh
```


**Running ROS simulation command.**

```
roslaunch raspimouse_gazebo raspimouse_with_emptyworld.launch   //to start up the gazebo with empty world.

```
**Then open new terminal:**

```
rosrun <launch file> <script.py>
```


