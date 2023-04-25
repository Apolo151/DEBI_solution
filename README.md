##### This repo contains our team's solution for the DEBI roboitcs challenge
[Competition Guide](https://drive.google.com/file/d/1BXq8UASasAKcVyFiAwk3sP4nw0m5puhp/view)

[Data for ML approaches](https://drive.google.com/drive/u/3/folders/10ELHnNTLzk8KlJmtVNdc6GsWTaqzyflx)

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

#### Gather Image Data
**Three Terminals**

```bash
DEBI_solution$ roslaunch solution gather_image_data.launch
```

```bash
DEBI_solution/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/scripts$ python3 spawn_three_balls.py
```
```bash
DEBI_solution/src/perception/scripts/img_data$ rosrun perception get_image_data.py
```
- Start the simulator
- Repeat for each test case

#### Gather Distance and Radius Data
**Two Terminals**

```bash
$ roslaunch solution gather_distance_radius_data.launch
```
- Set blue ball coordinates from simulator to (x=2.5, y=0.0)
- Start the simulator

```bash
$ rosrun solution radius_to_distance.py
```
- a ".csv" file will be saved in the same directory of the second terminal

#### Run Final Solution (In Progress)

```bash
roslaunch solution final_solution.launch
```

