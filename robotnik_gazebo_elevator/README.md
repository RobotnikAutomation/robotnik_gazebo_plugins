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

The package contains the gazebo world. The simulation launch file is included in this package.

<h1>Simulating elevator</h1>

1. To rund the demo, install the following dependencies:
  - [rb1_base_common](https://github.com/RobotnikAutomation/rb1_base_common)
  - [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs)
  - [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors)
  - [robotnik_elevator_interface](https://github.com/RobotnikAutomation/robotnik_elevator_interface)
  - [rcomponent](https://github.com/RobotnikAutomation/rcomponent)
  - [robotnik_gazebo_models](https://github.com/RobotnikAutomation/robotnik_gazebo_models)

    In the workspace install the packages dependencies:
    ```
    rosdep install --from-paths src --ignore-src -r -y
    ```  
  - Copy the "$(find elevator_gazebo)/models" in your gazebo models folder

2. Launch elevator simulation:

  roslaunch elevator_gazebo elevator_world.launch


<h1>Topics & Services</h1>

- See [robotnik_elevator_component](https://github.com/RobotnikAutomation/robotnik_elevator_interface/tree/master/robotnik_elevator_component).


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
	    <param name="elevator_id" value="elevator"/>
	    <param name="floor_height" value="40.0"/>
	</node>
  2. Replace the 6 pose arguments by the desired initial pose of the elevator 

- To change the geometry of the elevator: 1. change the xacro:properties at the beginning of the urdf model "$(find elevator_description)/urdf/elevator.xacro"
					  2. the rest of the file doesn't need any changes, all the geometry is parameterized

- To change the height of the floor : 1. change the global variable floor_height in "$(find elevator)/elevator_node.cpp"
				      2. change the elevatorcab_joint upper limit if needed in the urdf model "$(find elevator_description)/urdf/elevator.xacro"





