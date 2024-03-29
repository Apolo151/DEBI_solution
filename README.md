##### This repo contains our team's solution for the DEBI robotics challenge
[Competition Guide](https://drive.google.com/file/d/1BXq8UASasAKcVyFiAwk3sP4nw0m5puhp/view)

[Data for ML approaches](https://drive.google.com/drive/u/3/folders/10ELHnNTLzk8KlJmtVNdc6GsWTaqzyflx)

## How to run the simulator 
**Install Dependencies**

> Make sure you installed ROS noetic or install it following the steps here: [ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

```bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-noetic-moveit-*
sudo apt-get install ros-noetic-joint-* 
```
**Buid the Workspace**
```
catkin_make
```
**Run the simulator**  
⚠️ **It's easier to put** ```export TURTLEBOT3_MODEL=waffle_pi``` **in the .bashrc**  
**Terminal 1**
```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```
**Terminal 2**
```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
**Terminal 3**
```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```


### Run Solution Packages

**Robot Control package**

```bash
rosrun move_robot move_robot.py
```

#### Gather Image Data
**Open Three Terminals**

```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch solution gather_image_data.launch
```

```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
cd src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/scripts
python3 spawn_three_balls.py
```

```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
cd src/perception/image_data
rosrun perception get_image_data.py
```

- Start the simulator
- Repeat for each test case

#### Gather Distance and Radius Data
**Open Two Terminals**

```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch solution gather_distance_radius_data.launch
```
- Set the blue ball coordinates from the simulator to (x=2.5, y=0.0)
- Start the simulator


- from the directory you want the .csv file to be saved to 
```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
rosrun solution radius_to_distance.py
```

#### Run The Final Solution

```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch control solution.launch
```
