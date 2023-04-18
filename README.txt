---Install Dependencies---
    $ rosdep install --from-paths src --ignore-src -r -y
    $ sudo apt-get install ros-noetic-moveit-* ```
    $ sudo apt-get install ros-noetic-joint-* ```


---Run Solution---
- Add the following line to your .bashrc file: "export TURTLEBOT3_MODEL=waffle_pi"

- Open two terminals in the debi_ws directory.

in the first terminal, run the following commands:
    $ catkin_make
    $ source devel/setup.bash
    $ roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

in the second terminal, run the following commands:
    $ source devel/setup.bash
    $ rosrun move_robot e5bt_ball.py

> Note: make sure the file e5bt_ball.py is executable. If not, run the following command:
    $ chmod +x e5bt_ball.py

- Start the simulation in Gazebo by pressing the play button

