elevator
===========

<h1> Packages </h1>

<h2>elevator_node</h2>

This package contains the ros node creating the services of the elevator which are publishing in the ros_control topics of the corresponding elevator joints.

<h2>elevator_control</h2>

This package contains the control parameters configuration of the elevator joints and the launch file that is loading the controllers. 

<h2>elevator_description</h2>

The urdf needed in the description are contained here. This package includes the description of the elevator.
The package includes also a launch file to spawn the elevator at a desired pose.

<h2>elevator_gazebo</h2>

The package contains the gazebo world and models. The simulation launch file is included in this package.

<h1>Simulating elevator</h1>

1. Install the following dependencies:
  - rb1_base_common [link](https://github.com/RobotnikAutomation/rb1_base_common)
  - robotnik_msgs [link](https://github.com/RobotnikAutomation/robotnik_msgs)
  - robotnik_sensors [link](https://github.com/RobotnikAutomation/robotnik_sensors)

    In the workspace install the packages dependencies:
    ```
    rosdep install --from-paths src --ignore-src -r -y
    ```  
  - Copy the "$(find elevator_gazebo)/models" in your gazebo models folder

2. Launch elevator simulation :

  roslaunch elevator_gazebo elevator_world.launch


<h1>Services</h1>

- rosservice call /get_control: displays that the control of the elevator has been taken by the user

- rosservice call /set_door open: open the doors of the elevator

-     "        "      "     close: close the doors of the elevator

- rosservice call /go_to_floor n: lifts the elevator to the nth floor (which height is the nth multiple of the parameter floor_height - see parameters hereunder)

- rosservice call /release_control: displays that the control of the elevator has been released by the user


<h1>Parameters</h1>

- To use the elevator in a other environment 
  1. Copy the following lines in the simulation launch file:
	<!-- Load the URDF into the ROS Parameter Server -->
  	<include file="$(find elevator_description)/launch/spawn_elevator.launch">
    	    <arg name="x" value="$(arg x)"/>
     	    <arg name="y" value="$(arg y)"/>
	    <arg name="z" value="$(arg z)"/>
	    <arg name="roll" value="$(arg roll)"/>
	    <arg name="pitch" value="$(arg pitch)"/>
	    <arg name="yaw" value="$(arg yaw)"/>
  	</include>
	<!-- ros_control elevator launch file -->
	<include file="$(find elevator_control)/launch/elevator_control.launch" />
	
	<node name="elevator_node" pkg="elevator_node" type="elevator_node" output="screen">
	</node>
  2. Replace the 6 pose arguments by the desired initial pose of the elevator 

- To change the geometry of the elevator: 1. change the xacro:properties at the beginning of the urdf model "$(find elevator_description)/urdf/elevator.xacro"
					  2. the rest of the file doesn't need any changes, all the geometry is parameterized

- To change the height of the floor : 1. change the global variable floor_height in "$(find elevator)/elevator_node.cpp"
				      2. change the elevatorcab_joint upper limit if needed in the urdf model "$(find elevator_description)/urdf/elevator.xacro"





