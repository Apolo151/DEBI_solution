---Install Dependencies---

make sure you installed ROS noetic or install it following the steps here: [ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

- python version used: 3.8.10

- Install the following dependencies:

    $ rosdep install --from-paths src --ignore-src -r -y
    $ sudo apt-get install ros-noetic-moveit-* ```
    $ sudo apt-get install ros-noetic-joint-* ```


- Then install python dependencies:
    $ pip3 install -r requirements.txt

> Note:
It is better to use a virtual environment for python dependencies. You can use the following commands:
    $ sudo apt-get install python3-venv
    $ python3 -m venv venv
    $ source venv/bin/activate
    $ pip3 install -r requirements.txt

Or using conda:
    Install conda by following the steps from here: [conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html)
    $ conda create --name myenv
    $ conda activate myenv
    $ pip3 install -r requirements.txt

---Run Solution---
- Add the following line to your .bashrc file: "export TURTLEBOT3_MODEL=waffle_pi"

- Open one terminal in the debi_ws directory.

- run the following commands:
    $ catkin_make
    $ source devel/setup.bash
    $ roslaunch control solution.launch


> Note: make sure the files perception/get_circles.py and control/go_to_ball.py are executable. If not, use the following command:
    $ chmod +x file_name.py

- Start the simulation in Gazebo by pressing the play button

