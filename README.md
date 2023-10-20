# information

<p align="center">
    <img src="./img/architecture.jpg" alt="architecture"/>
</p>

# misc

create package:
ros2 pkg create linefollower

build:
cd ros2_ws
colcon build

gazebo :
- https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros#InstallROS2

```
sudo apt install ros-humble-gazebo-ros-pkgs
```

Our ros Package :


URDF robot creation :

create a robot model:
https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html

- clone this repo https://github.com/joshnewans/my_bot this a template for creating urdf xacro robots in ros2
- create the robot joint and link using xacro to generate the XML URDF file


to build :
colcon build --symlink-install

to launch our urdf use : 
ros2 launch bocchi_bot rsp.launch.py

Run the created robot in Gazebo :

launch urdf in gazeboas it is
 - launch robot_state_publisher in sim mode
 - launch gazebo with ROS compatibility
 - spawn robot in gazebo with spawn script

 
we need to launch with simulation time so it works well in gazebo

launch robot_state_publisher in sim mode : 
ros2 launch bocchi_bot rsp.launch.py use_sim_timem:=true

launch gazebo with ROS compatibility :
sudo apt install ros-<distro>-gazebo-ros-pkgs
ros2 launch gazebo_ros gazebo.launch.py

spawn robot in gazebo with spawn script :
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity bot_name

create a launch file that will do all the above in one and launch it:
colcon build
ros2 launch bocchi_bot launch_sim.launch.py

with world as parameter :
ros2 launch bocchi_bot launch_sim.launch.py world:=./src/bocchi_bot/worlds/lfm1.world
