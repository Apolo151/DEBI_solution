##### This repo contains our team's solution for the DEBI roboitcs challenge
[Competition Guide](https://drive.google.com/file/d/1BXq8UASasAKcVyFiAwk3sP4nw0m5puhp/view)
## How to run simulator 
**Install Dependencies**

```
rosdep install --from-paths src --ignore-src -r -y
```
**Buid the Workspace**
```
catkin_make
```
**Run the simulator**  
⚠️ **It's easy to put** ```export TURTLEBOT3_MODEL=waffle_pi``` **in the .bashrc**  
**Treminal 1**
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```
**Terminal 2**
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
**Terminal 3**
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```


### Run Solution Packages

**Robot Control package**

```bash
rosrun move_robot move_robot.py
```
